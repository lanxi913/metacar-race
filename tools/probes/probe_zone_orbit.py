"""整场探测 v7：绕场一圈（不穿车群），4 路相机 3 Hz 采图 + 车牌候选记录。

与 v6 的区别
------------
* 航线改成「绕中圈」：入口 → 东南 → 北 → 西 → 南 绕回来，避开已知车辆（上一局在
  (20,7.5) 撞上一辆蓝车）。
* 只记录候选（不跑 OCR），OCR 全部离线做，跑次里不做耗时操作。
* 每 0.34 s 存 4 路 JPEG；候选框写 CSV，含 ego 位姿，便于离线反投影。

用法（先起脚本，再去平台点启动场景）
    python tools/probe_zone_orbit.py --seconds 100
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from probe_zone_lap import WpFollower  # noqa: E402  (复用已验证的控制器)

from plate import find_plates  # noqa: E402

RING = [
    (31.0, -7.0), (28.0, -2.0), (25.0, 4.0), (21.0, 11.0),
    (12.0, 14.5), (2.0, 14.5), (-6.0, 9.0), (-9.0, 1.0),
    (-6.0, -7.0), (2.0, -11.0), (12.0, -12.5), (22.0, -11.5), (29.0, -8.0),
]
ENTRY = (34.6, -4.0)


def build_waypoints(laps: int) -> list[tuple[float, float]]:
    return [ENTRY] + RING * max(1, laps) + [ENTRY]


def save_jpg(path: Path, img: np.ndarray, quality: int = 82) -> bool:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return False
    path.write_bytes(buf.tobytes())
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=100.0)
    ap.add_argument("--shot-every", type=float, default=0.34)
    ap.add_argument("--lookahead", type=float, default=6.0)
    ap.add_argument("--accept", type=float, default=4.0)
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--out", default=r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\zone_orbit")
    ap.add_argument("--self-check", action="store_true")
    args = ap.parse_args()

    pts = build_waypoints(args.laps)
    out = Path(args.out)

    if args.self_check:
        f = WpFollower(pts, args.lookahead, args.accept)
        f.update(34.6, -4.0)
        print(f"self-check ok: 航点 {len(pts)} 目标 {f.target(34.6, -4.0)} "
              f"steer {f.steering(34.6, -4.0, 180.0):+.3f} 剩余 {f.remain_m(34.6, -4.0):.1f} m")
        return 0

    from metacar import GearMode, SceneAPI, VehicleControl

    (out / "f").mkdir(parents=True, exist_ok=True)
    fcsv = (out / "frames.csv").open("w", newline="", encoding="utf-8")
    wtr = csv.writer(fcsv)
    wtr.writerow(["t", "ego_x", "ego_y", "yaw", "v", "tag", "cam", "u", "v0", "w", "h", "area"])

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"场景: {[s.name for s in static.sub_scenes]}", flush=True)

    follower = WpFollower(pts, args.lookahead, args.accept)
    t0 = time.time()
    tag = 0
    last_shot = -1e9
    last_log = -1e9
    n_cand = 0
    last_seg = 0
    last_prog = 0.0

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))

        if now > args.seconds or follower.done():
            print(f"[{now:5.1f}s] 收工（{'到时间' if now > args.seconds else '走完航点'}），"
                  f"挂 P 挡驻车", flush=True)
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            time.sleep(0.5)
            break

        follower.update(x, y)
        if follower.seg != last_seg:
            last_seg = follower.seg
            last_prog = now
            print(f"[{now:5.1f}s] 到点 {pts[last_seg - 1]} → 下一个 "
                  f"{pts[min(follower.seg, len(pts) - 1)]} 剩余 {follower.remain_m(x, y):.1f} m",
                  flush=True)
        if now - last_prog > 22.0:
            print(f"[{now:5.1f}s] !! 22 s 没到点，强制跳下一个", flush=True)
            follower.seg = min(follower.seg + 1, len(pts) - 1)
            last_prog = now

        steer = follower.steering(x, y, yaw)
        api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=0.03,
                                               brake=0.0, steering=steer))

        if frames and now - last_shot >= args.shot_every:
            last_shot = now
            tag += 1
            cams = msg.sensor.ego_rgb_cams if msg.sensor else []
            for idx, fr in enumerate(frames):
                if fr.frame is None:
                    continue
                cam_id = cams[idx].id if idx < len(cams) else idx
                save_jpg(out / "f" / f"t{tag:04d}_cam{cam_id}.jpg", fr.frame)
                for (u, v0, w, h, a) in find_plates(fr.frame)[:4]:
                    wtr.writerow([f"{now:.2f}", f"{x:.2f}", f"{y:.2f}", f"{yaw:.1f}",
                                  f"{v:.2f}", f"t{tag:04d}", cam_id, u, v0, w, h, a])
                    n_cand += 1
            fcsv.flush()

        if now - last_log >= 2.0:
            last_log = now
            print(f"[{now:5.1f}s] ({x:7.2f},{y:7.2f}) yaw={yaw:6.1f} v={v:4.2f} "
                  f"剩余={follower.remain_m(x, y):5.1f} m 图={tag * 4} 候选={n_cand}", flush=True)

    fcsv.close()
    print(f"图片 {tag * 4} 张在 {out / 'f'}；候选记录 {out / 'frames.csv'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
