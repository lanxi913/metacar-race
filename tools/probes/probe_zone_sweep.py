"""VLA 功能区 —— 巡场采数探针（v3）：按航点扫一遍功能区，高频抓帧。

为什么需要它：v2 证明「全程静止」什么也看不到——场内车辆不在 `msg.obstacles` 里
（43 s 一直是 0），所以唯一的观测手段是相机。而车顶那块牌子（视觉标识/贴纸）在
30 m 外的帧里只有几个像素，必须开过去、在 5~10 m 内抓帧才可能读出编号。

做法：
  * 三个来回（y=-12 / 0 / +12）覆盖 50x50 的功能区；
  * 纯跟踪（与 VLA 文本导航同一套：yaw 顺时针为正、steer = -delta/MAX_STEER）；
  * 油门固定 0.02（实测这个值就已经顶到一挡力平衡 4.78 m/s，再小不减速，
    要减速只能靠刹车——采数不在乎加速度扣分）；
  * 每 0.15 s 存 4 路帧，同时把 (t, x, y, yaw, v, 帧前缀) 写进 poses.csv，
    事后可以用位姿把每帧落到地图上；
  * 开头打印 4 路相机的内参/外参（position / orientation / fov / intrinsic），
    有了它才能把像素反投影到地面算出车辆坐标。
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import cv2
from metacar import GearMode, SceneAPI, VehicleControl

OUT_DEFAULT = Path(r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\zone_sweep")

WHEELBASE_M = 2.6
MAX_STEER_RAD = 0.6
THROTTLE = 0.02

WAYPOINTS = [
    (22.0, -12.0), (-20.0, -12.0),
    (-20.0, 0.0), (20.0, 0.0),
    (20.0, 12.0), (-18.0, 12.0),
]


def save_png(path: Path, img) -> bool:
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(buf.tobytes())
    return True


class Follower:
    def __init__(self, wps, lookahead=6.0):
        self.wps = [tuple(map(float, w)) for w in wps]
        self.lookahead = lookahead
        self.seg = 0
        self.seg_off = 0.0
        self.uturn = 0.0

    def _len(self, i):
        (ax, ay), (bx, by) = self.wps[i], self.wps[i + 1]
        return math.hypot(bx - ax, by - ay)

    def update(self, x, y):
        best = None
        for i in range(self.seg, len(self.wps) - 1):
            (ax, ay), (bx, by) = self.wps[i], self.wps[i + 1]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            if L2 < 1e-9:
                continue
            t = ((x - ax) * vx + (y - ay) * vy) / L2
            t = max(0.0, min(1.0, t))
            px, py = ax + vx * t, ay + vy * t
            d = math.hypot(x - px, y - py)
            if best is None or d < best[1]:
                best = (t * math.sqrt(L2), d, i)
        if best:
            self.seg_off, _, self.seg = best
        return best[1] if best else 0.0

    def target(self):
        want = self.seg_off + self.lookahead
        acc = 0.0
        for i in range(self.seg, len(self.wps) - 1):
            L = self._len(i)
            if acc + L >= want:
                (ax, ay), (bx, by) = self.wps[i], self.wps[i + 1]
                f = (want - acc) / L if L > 1e-9 else 0.0
                return (ax + (bx - ax) * f, ay + (by - ay) * f)
            acc += L
        return self.wps[-1]

    def remaining(self):
        return max(0.0, sum(self._len(i) for i in range(self.seg, len(self.wps) - 1))
                   - self.seg_off)

    def steering(self, x, y, yaw_deg):
        tx, ty = self.target()
        dx, dy = tx - x, ty - y
        dist = max(math.hypot(dx, dy), 1e-3)
        rad = math.radians(-yaw_deg)          # 场景 yaw 顺时针为正
        lx = dx * math.cos(rad) + dy * math.sin(rad)
        ly = -dx * math.sin(rad) + dy * math.cos(rad)
        if lx < 0.0:
            if self.uturn == 0.0:
                self.uturn = 1.0 if ly > 0 else -1.0
            return self.uturn
        self.uturn = 0.0
        curvature = 2.0 * ly / (dist * dist)
        delta = math.atan(curvature * WHEELBASE_M)
        return max(-1.0, min(1.0, -delta / MAX_STEER_RAD))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=95.0)
    ap.add_argument("--shot-every", type=float, default=0.15)
    ap.add_argument("--lookahead", type=float, default=6.0)
    ap.add_argument("--hold", type=float, default=6.0, help="跑完航点后原地停多久")
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    ap.add_argument("--self-check", action="store_true")
    args = ap.parse_args()

    out = Path(args.out)
    if args.self_check:
        f = Follower(WAYPOINTS)
        f.update(34.6, -4.05)
        print(f"self-check OK target={f.target()} steer={f.steering(34.6, -4.05, 180.0):+.3f}")
        return 0
    out.mkdir(parents=True, exist_ok=True)

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    vla = getattr(static, "vla_extension", None)
    if vla is None:
        print("!! 这不是 VLA 场景")
        return 2
    for r in vla.regions:
        print(f"区域 {r.type} id={r.id} ({r.pos_x:.1f},{r.pos_y:.1f}) "
              f"{r.length:.1f}x{r.width:.1f} ori_z={r.ori_z:.1f}", flush=True)

    follower = Follower(WAYPOINTS, lookahead=args.lookahead)
    csv_f = (out / "poses.csv").open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_f)
    writer.writerow(["t", "x", "y", "yaw", "v", "tag", "mode"])

    t0 = time.time()
    prev_shot = -1e9
    prev_log = -1e9
    prev_v = None
    prev_t = None
    tag = 0
    n_frames = 0
    cams_printed = False
    stuck_t = 0.0
    reversing_until = -1.0
    done_t = -1.0

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))

        if not cams_printed:
            cams_printed = True
            print(f"trajectory 点={len(msg.trajectory)} "
                  f"v2x_cams={len(msg.sensor.v2x_cams)}", flush=True)
            for cam in msg.sensor.ego_rgb_cams:
                o = cam.orientation
                im = ", ".join(f"{q:.1f}" for q in cam.intrinsic_matrix)
                print(f"  cam id={cam.id} pos=({cam.position.x:.2f},{cam.position.y:.2f},"
                      f"{cam.position.z:.2f}) ori=({o.ori_x:.1f},{o.ori_y:.1f},{o.ori_z:.1f}) "
                      f"fov={cam.fov:.1f} {cam.image_width}x{cam.image_height} "
                      f"K=[{im}]", flush=True)

        lat = follower.update(x, y)
        steer = follower.steering(x, y, yaw)
        remaining = follower.remaining()

        if now >= args.seconds or (remaining < 1.5 and done_t < 0):
            done_t = now if done_t < 0 else done_t
        if done_t > 0:
            mode = "hold"
            vc = VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0, steering=0.0)
            if now - done_t > args.hold:
                print(f"完成：{now:.1f}s 存帧 {n_frames} 张", flush=True)
                break
        elif now < reversing_until:
            mode = "rev"
            vc = VehicleControl(gear=GearMode.REVERSE, throttle=0.02, brake=0.0,
                                steering=-steer * 0.5)
            stuck_t = 0.0
        else:
            mode = "drive"
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=THROTTLE, brake=0.0,
                                steering=steer)
            if v < 0.15:
                stuck_t += 0.05
            else:
                stuck_t = 0.0
            if stuck_t > 1.5:
                print(f"[{now:5.1f}s] 疑似卡住（v={v:.2f}），倒车 2 s", flush=True)
                reversing_until = now + 2.0
                stuck_t = 0.0

        if frames and now - prev_shot >= args.shot_every:
            prev_shot = now
            tag += 1
            for cam, fr in zip(msg.sensor.ego_rgb_cams, frames):
                if fr.frame is None:
                    continue
                if save_png(out / "f" / f"w{tag:04d}_cam{cam.id}.png", fr.frame):
                    n_frames += 1
            writer.writerow([f"{now:.2f}", f"{x:.2f}", f"{y:.2f}", f"{yaw:.1f}",
                             f"{v:.2f}", f"w{tag:04d}", mode])

        if now - prev_log >= 1.0:
            prev_log = now
            print(f"[{now:5.1f}s] {mode:<5} ({x:7.2f},{y:7.2f}) yaw={yaw:6.1f} v={v:5.2f} "
                  f"steer={steer:+.2f} seg={follower.seg} lat={lat:5.2f} "
                  f"剩余={remaining:6.1f} 帧={n_frames}", flush=True)

        if prev_v is not None and now - prev_t > 1e-3:
            acc = (v - prev_v) / (now - prev_t)
            if abs(acc) > 8.0:
                print(f"     !! |a|={acc:.1f} thr={vc.throttle:.3f} brk={vc.brake:.2f}",
                      flush=True)
        prev_v, prev_t = v, now

        api.set_vehicle_control(vc)

    csv_f.close()
    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except OSError:
        pass
    print(f"结束：帧 {out / 'f'}  位姿 {out / 'poses.csv'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
