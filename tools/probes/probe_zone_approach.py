"""VLA 功能区 —— 靠近一辆车拍它的车顶牌（v4）。

为什么要「靠近」而不是等：v3 的教训是没有直接证据，只能算出
30 m 外的车顶牌只有 3~5 px 宽，任何 OCR 都读不出来。而实测（pitch=0 的针孔模型
+ cam0 内参 fx=184.8 fy=138.6）这个场景的车辆其实就在 10~20 m 内，
开到 3 m 时 1.8 m 宽的车身会有 ~110 px，车顶牌也能有 30~40 px，才有得读。

做法：
  * 纯跟踪走一串航点，从起点 (34.6,-4.05) 切进蓝区里车辆最密的一带；
  * 前视相机做「车辆色块」检测（饱和蓝 / 很暗的车体），色块宽度超过阈值就说明
    已经贴近，立刻刹停；
  * 停稳后原地连拍 10 s（4 路 × 3 Hz），并打印每帧里色块的位置，
    事后离线裁图放大 OCR 车顶牌；
  * 到最后一个航点还没触发也停下拍。
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np
from metacar import GearMode, SceneAPI, VehicleControl

OUT_DEFAULT = Path(r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\zone_approach")

WHEELBASE_M = 2.6
MAX_STEER_RAD = 0.6
THROTTLE = 0.02
CX, CY = 320.0, 240.0

WAYPOINTS = [(30.0, -4.0), (25.0, 1.0), (20.0, 3.0), (15.0, 5.0), (11.0, 6.0)]
STOP_BLOB_W = 110.0      # 车体色块宽（px），约等于 3 m 处的 1.8 m 车
STOP_AT_LAST_M = 3.0


def save_png(path: Path, img) -> bool:
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(buf.tobytes())
    return True


def vehicle_blobs(frame):
    """返回 [(bbox_x, bbox_y, w, h, cx, cy, kind)]，按面积降序。

    掩码两条：① 饱和蓝（车漆，apron 的饱和度只有 ~130，车漆 >175）；
    ② 很暗（三轮车墨绿/黑车厢）；都限制在地平线以下 v>245。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    blue = (s > 175) & (v > 60) & (v < 250) & (h > 95) & (h < 135)
    dark = (v < 95)
    mask = (blue | dark).astype(np.uint8) * 255
    mask[:245, :] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        x, y, w, hh, area = stats[i]
        if area < 200 or w < 12:
            continue
        out.append((x, y, w, hh, float(cents[i][0]), float(cents[i][1]), "blue" if blue[y + hh // 2, x + w // 2] else "dark"))
    out.sort(key=lambda r: -r[2] * r[3])
    return out


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
        rad = math.radians(-yaw_deg)
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
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--hold", type=float, default=10.0)
    ap.add_argument("--shot-every", type=float, default=0.3)
    ap.add_argument("--stop-width", type=float, default=STOP_BLOB_W)
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    ap.add_argument("--self-check", action="store_true")
    args = ap.parse_args()

    if args.self_check:
        f = Follower(WAYPOINTS)
        f.update(34.6, -4.05)
        print(f"self-check OK target={f.target()} steer={f.steering(34.6, -4.05, 180.0):+.3f}")
        return 0

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    if getattr(static, "vla_extension", None) is None:
        print("!! 不是 VLA 场景")
        return 2

    follower = Follower(WAYPOINTS)
    t0 = time.time()
    stop_t = -1.0
    prev_shot = -1e9
    prev_log = -1e9
    tag = 0
    n_saved = 0
    first_stop_w = None

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))

        if stop_t < 0 and frames:
            blobs = vehicle_blobs(frames[0].frame)
            if blobs:
                bw = blobs[0][2]
                if now - prev_log >= 0.5:
                    print(f"[{now:5.1f}s] 车体色块 {len(blobs)} 个，最大宽 {bw:.0f} px "
                          f"@({blobs[0][4]:.0f},{blobs[0][5]:.0f})", flush=True)
            if blobs and blobs[0][2] >= args.stop_width:
                stop_t = now
                first_stop_w = blobs[0][2]
                print(f"[{now:5.1f}s] 贴近目标（宽 {blobs[0][2]:.0f} px ≥ "
                      f"{args.stop_width:.0f}），刹停拍牌", flush=True)
            elif follower.remaining() < STOP_AT_LAST_M:
                stop_t = now
                print(f"[{now:5.1f}s] 到达航点末端，刹停拍牌", flush=True)

        if stop_t >= 0:
            mode = "hold"
            vc = VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0, steering=0.0)
            if now - stop_t > args.hold:
                print(f"结束：{now:.1f}s，存帧 {n_saved}", flush=True)
                break
        else:
            mode = "drive"
            steer = follower.steering(x, y, yaw)
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=THROTTLE, brake=0.0,
                                steering=steer)

        if frames and now - prev_shot >= args.shot_every:
            prev_shot = now
            tag += 1
            for cam, fr in zip(msg.sensor.ego_rgb_cams, frames):
                if fr.frame is None:
                    continue
                if save_png(out / "f" / f"a{tag:04d}_cam{cam.id}.png", fr.frame):
                    n_saved += 1
            with (out / "poses.csv").open("a", encoding="utf-8") as fh:
                fh.write(f"{now:.2f},{x:.2f},{y:.2f},{yaw:.1f},{v:.2f},a{tag:04d},{mode}\n")

        if now - prev_log >= 1.0:
            prev_log = now
            print(f"[{now:5.1f}s] {mode:<5} ({x:7.2f},{y:7.2f}) yaw={yaw:6.1f} v={v:5.2f} "
                  f"剩余={follower.remaining():5.1f} 帧={n_saved}", flush=True)

        api.set_vehicle_control(vc)
        if now > args.seconds:
            print("超时退出", flush=True)
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except OSError:
        pass
    print(f"帧目录 {out / 'f'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
