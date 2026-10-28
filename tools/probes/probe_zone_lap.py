"""VLA 功能区 —— 「整场跑一趟」采集（v6.1，修好转向/航点跟踪）。

v6 的教训：115 s 只在 (23,−4) 附近画了个半径 4.5 m 的圈，`剩余` 一直是 175.6 ——
原因是老的跟踪器「投影到全局最近线段 + 沿线段前视 6 m」，三条车道只隔 12 m，
投影会跳到别的车道，目标点跑到车侧后，转向饱和 → 原地绕圈。
现在换成「逐航点 + 定点纯跟踪」：只朝下一个航点开，到点半径 4 m 就前进一个；
已在 `accel_window_probe/_vla_ref/sim_follower2.py` 用自行车模型离线验证：
lookahead 6 m / accept 4 m 时 31.6 s 依次走完 11 个航点（三条车道全覆盖）。

转向符号按 v6 实测标定：`steer > 0` 时自车 yaw 增大（= 车头往 +y 侧摆），
纯跟踪曲率求出的 δ>0 对应「该往 ly 正方向转」，所以下发值取 −δ/0.6。
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import cv2
import numpy as np

OUT = Path(r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\zone_lap")
WHEELBASE_M = 2.6
MAX_STEER_RAD = 0.6

WAYPOINTS = [
    (30.0, -4.0), (25.0, -1.0), (21.0, 4.0),          # 入口进场
    (18.0, 12.0), (-18.0, 12.0),                       # 车道 1（+y）
    (-18.0, 0.0), (18.0, 0.0),                         # 车道 2（中线）
    (18.0, -12.0), (-18.0, -12.0),                     # 车道 3（-y）
    (-16.0, -6.0), (0.0, -2.0),                        # 收尾回中心
]


def save_jpg(path: Path, img, q: int = 88) -> bool:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
    if not ok:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(buf.tobytes())
    return True


def plate_candidates(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    mask = ((h >= 10) & (h <= 45) & (s >= 60) & (v >= 100)).astype(np.uint8) * 255
    mask[:200, :] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        x, y, w, hh, area = stats[i]
        if area < 20 or hh < 3 or w < 6:
            continue
        if not (0.7 <= w / float(hh) <= 8.0):
            continue
        if area / float(w * hh) < 0.5:
            continue
        out.append((int(x), int(y), int(w), int(hh), int(area)))
    out.sort(key=lambda r: -r[4])
    return out[:8]


class WpFollower:
    """逐航点纯跟踪：目标 = 下一个航点（≥ lookahead 时取车前方 lookahead 处）。"""

    def __init__(self, wps, lookahead=6.0, accept=4.0):
        self.wps = [tuple(map(float, w)) for w in wps]
        self.lookahead = lookahead
        self.accept = accept
        self.seg = 0

    def done(self):
        return self.seg >= len(self.wps) - 1

    def remain_m(self, x, y):
        if self.done():
            return 0.0
        total = math.hypot(self.wps[self.seg][0] - x, self.wps[self.seg][1] - y)
        for i in range(self.seg, len(self.wps) - 1):
            (ax, ay), (bx, by) = self.wps[i], self.wps[i + 1]
            total += math.hypot(bx - ax, by - ay)
        return total

    def update(self, x, y):
        while not self.done():
            wx, wy = self.wps[self.seg]
            if math.hypot(wx - x, wy - y) < self.accept:
                self.seg += 1
            else:
                break

    def target(self, x, y):
        wx, wy = self.wps[self.seg]
        dx, dy = wx - x, wy - y
        d = math.hypot(dx, dy)
        if d <= self.lookahead or d < 1e-6:
            return wx, wy
        return x + dx / d * self.lookahead, y + dy / d * self.lookahead

    def steering(self, x, y, yaw_deg):
        if self.done():
            return 0.0
        tx, ty = self.target(x, y)
        dx, dy = tx - x, ty - y
        dist = max(math.hypot(dx, dy), 1e-3)
        rad = math.radians(-yaw_deg)
        ly = -dx * math.sin(rad) + dy * math.cos(rad)
        delta = math.atan((2.0 * ly / (dist * dist)) * WHEELBASE_M)
        return max(-1.0, min(1.0, -delta / MAX_STEER_RAD))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=110.0)
    ap.add_argument("--shot-every", type=float, default=0.5)
    ap.add_argument("--self-check", action="store_true")
    args = ap.parse_args()

    if args.self_check:
        f = WpFollower(WAYPOINTS)
        f.update(34.6, -4.05)
        print(f"self-check OK 目标={f.target(34.6, -4.05)} "
              f"剩余={f.remain_m(34.6, -4.05):.1f} "
              f"steer={f.steering(34.6, -4.05, 180.0):+.3f}")
        return 0

    from metacar import GearMode, SceneAPI, VehicleControl

    OUT.mkdir(parents=True, exist_ok=True)
    csv_path = OUT / "frames.csv"
    fh = csv_path.open("w", newline="", encoding="utf-8")
    wr = csv.writer(fh)
    wr.writerow(["t", "ego_x", "ego_y", "yaw", "v", "tag", "cam", "u", "vp", "w", "h", "area"])

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    if getattr(static, "vla_extension", None) is None:
        print("!! 不是 VLA 场景")
        return 2

    follower = WpFollower(WAYPOINTS)
    t0 = time.time()
    tag = 0
    prev_shot = -1e9
    prev_log = -1e9
    n_img = 0
    n_det = 0
    done_t = -1.0
    last_seg = 0
    seg_t = time.time()

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))

        if frames and now - prev_shot >= args.shot_every:
            prev_shot = now
            tag += 1
            cams = msg.sensor.ego_rgb_cams if msg.sensor else []
            for idx, fr in enumerate(frames):
                if fr.frame is None:
                    continue
                cam_id = cams[idx].id if idx < len(cams) else idx
                save_jpg(OUT / "f" / f"t{tag:04d}_cam{cam_id}.jpg", fr.frame)
                n_img += 1
                for (u, vp, w, hh, area) in plate_candidates(fr.frame):
                    wr.writerow([f"{now:.2f}", f"{x:.2f}", f"{y:.2f}", f"{yaw:.1f}",
                                 f"{v:.2f}", f"t{tag:04d}", cam_id, u, vp, w, hh, area])
                    n_det += 1

        follower.update(x, y)
        if follower.seg != last_seg:
            print(f"[{now:5.1f}s] 到点 {follower.wps[last_seg]} → 下一个 "
                  f"{follower.wps[follower.seg]} 剩余 {follower.remain_m(x, y):.1f} m", flush=True)
            last_seg = follower.seg
            seg_t = now
        if not follower.done() and now - seg_t > 25.0:
            print(f"!! 25 s 没到点，强制跳到下一个航点", flush=True)
            follower.seg = min(follower.seg + 1, len(follower.wps) - 1)
            last_seg = follower.seg
            seg_t = now

        if follower.done() and done_t < 0:
            done_t = now
            print(f"[{now:5.1f}s] 三趟车道跑完，驻车等场景自然结束", flush=True)

        if done_t >= 0:
            vc = VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0, steering=0.0)
        else:
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.02, brake=0.0,
                                steering=follower.steering(x, y, yaw))
        api.set_vehicle_control(vc)

        if now - prev_log >= 2.0:
            prev_log = now
            print(f"[{now:5.1f}s] ({x:7.2f},{y:7.2f}) yaw={yaw:6.1f} v={v:5.2f} "
                  f"steer={vc.steering:+.2f} 剩余={follower.remain_m(x, y):5.1f} "
                  f"图={n_img} 候选={n_det}", flush=True)
        if now > args.seconds:
            print("到时退出", flush=True)
            break

    fh.close()
    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except OSError:
        pass
    print(f"图片 {n_img} 张在 {OUT / 'f'}；检测记录 {csv_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
