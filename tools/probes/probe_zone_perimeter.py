"""绕功能区边缘跑一圈（勘察局，v2）：沿区域内侧走一圈、卡死会倒车绕开。

为什么要跑
----------
用户观察：车基本停在功能区边缘处。沿周长一圈 = 每辆车都在 2~4 m 侧距内从侧面掠过，
四路相机（前/左/右/后，各 120°，均 1.91 m 高）能把这些车的侧门编号一次收下，同时把
"边缘有几辆车、各在哪个方位"量清楚。

v2 相对 v1 的三处改动
--------------------
1. **航线内缩到 ±15 m**：v1 走 ±18 m，在 `(9.7,-17.4)` 被南边一辆蓝色车顶住（v=0）。
   内缩后距已知车位 ≥4.5 m。
2. **卡死恢复**：油门 >0 且 |v|<0.15 持续 1.5 s ⇒ P 挡连拍 2 s → 倒车 2.5 s → 跳过当前
   航点继续（v1 卡死后就停在原地 13 s 直到场景结束）。
3. **进场避让路牌杆**：杆在 `(30.0,-3.9)`，先压到 `(32.5,-8.5)` 再折西，仿真最近距杆 2.85 m。

用法
----
    python tools/probe_zone_perimeter.py --seconds 60 --out D:\\...\\runs\\vla-perimeter-r2
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
sys.path.insert(0, str(_REPO / "tools"))

from probe_zone_lap import WpFollower  # noqa: E402

ENTRY = (34.60, -4.05)
SIGN_POLE = (30.0, -3.9)
WAYPOINTS = [
    (32.5, -8.5),    # 进场：先往南压，避开路牌杆
    (26.0, -12.0),
    (12.0, -15.0),   # 南边（内缩到 -15）
    (-12.0, -15.0),
    (-16.0, -2.0),   # 西边
    (-16.0, 10.0),
    (-6.0, 16.0),    # 北边
    (8.0, 16.0),
    (16.0, 6.0),     # 东边（回程）
    (16.0, -10.0),
    (22.0, -14.0),   # 收尾
]
ZONE_HALF = 25.0
BOUND = 32.0
THROTTLE = 0.03
PIN_SPEED = 0.15
PIN_SECONDS = 1.5
BURST_S = 2.0
BACK_S = 2.5


def save_jpg(path: Path, img: np.ndarray, quality: int = 90) -> bool:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return False
    path.write_bytes(buf.tobytes())
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default=str(Path.home() / "zone_perimeter"))
    ap.add_argument("--shot-every", type=float, default=0.3)
    ap.add_argument("--lookahead", type=float, default=6.0)
    ap.add_argument("--accept", type=float, default=4.0)
    ap.add_argument("--self-check", action="store_true")
    args = ap.parse_args()

    if args.self_check:
        f = WpFollower(WAYPOINTS, args.lookahead, args.accept)
        f.update(ENTRY[0], ENTRY[1])
        plen = sum(math.hypot(WAYPOINTS[i + 1][0] - WAYPOINTS[i][0],
                              WAYPOINTS[i + 1][1] - WAYPOINTS[i][1])
                   for i in range(len(WAYPOINTS) - 1))
        d0 = math.hypot(WAYPOINTS[0][0] - ENTRY[0], WAYPOINTS[0][1] - ENTRY[1])
        tot = plen + d0
        dp = min(math.hypot(p[0] - SIGN_POLE[0], p[1] - SIGN_POLE[1]) for p in WAYPOINTS)
        print(f"self-check OK 航点={len(WAYPOINTS)} 总程≈{tot:.1f} m "
              f"4.78 m/s 估时≈{tot / 4.78:.1f} s 最近航点距杆 {dp:.1f} m "
              f"内圈最大|x|,|y|={max(max(abs(p[0]), abs(p[1])) for p in WAYPOINTS):.1f}")
        return 0

    out = Path(args.out)
    (out / "f").mkdir(parents=True, exist_ok=True)
    for old in (out / "f").glob("*.jpg"):
        old.unlink()

    from metacar import GearMode, SceneAPI, VehicleControl

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"场景: {[s.name for s in static.sub_scenes]}", flush=True)
    if static.vla_extension is not None:
        for reg in static.vla_extension.regions:
            print(f"  区域 id={reg.id} type={reg.type} pos=({reg.pos_x},{reg.pos_y}) "
                  f"{reg.length}x{reg.width} ori={reg.ori_z}", flush=True)

    follower = WpFollower(WAYPOINTS, args.lookahead, args.accept)
    t0 = time.time()
    tag = 0
    last_shot = -1e9
    last_log = -1e9
    last_seg = -1
    poses = []
    pins = []
    perim_min = 1e9
    perim_max = 0.0
    pin_since: float | None = None
    burst_until = 0.0
    back_until = 0.0

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))

        dist_pole = math.hypot(x - SIGN_POLE[0], y - SIGN_POLE[1])
        if now > args.seconds:
            print(f"[{now:5.1f}s] 到时间，驻车", flush=True)
            break
        if now > 8.0 and (abs(x) > BOUND or abs(y) > BOUND):
            print(f"[{now:5.1f}s] !! 越界 ({x:.1f},{y:.1f})，驻车", flush=True)
            break

        follower.update(x, y)
        if follower.seg != last_seg:
            last_seg = follower.seg
            if not follower.done():
                nxt = WAYPOINTS[min(follower.seg, len(WAYPOINTS) - 1)]
                print(f"[{now:5.1f}s] 到点 → 下一个 ({nxt[0]:.1f},{nxt[1]:.1f}) "
                      f"剩余≈{follower.remain_m(x, y):5.1f} m v={v:.2f}", flush=True)

        bursting = now < burst_until
        backing = burst_until <= now < back_until

        if backing:
            vc = VehicleControl(gear=GearMode.REVERSE, throttle=0.035, brake=0.0,
                                steering=0.0)
            mode = "back"
        elif bursting:
            vc = VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0,
                                steering=0.0)
            mode = "burst"
        else:
            if now > 6.0 and v < PIN_SPEED:
                if pin_since is None:
                    pin_since = now
                elif now - pin_since > PIN_SECONDS:
                    print(f"[{now:5.1f}s] !! 卡死 ({x:.2f},{y:.2f}) → "
                          f"连拍 + 倒车", flush=True)
                    burst_until = now + BURST_S
                    back_until = burst_until + BACK_S
                    pin_since = None
                    pins.append((round(now, 2), round(x, 2), round(y, 2),
                                 round(yaw, 1), follower.seg))
                    follower.seg = min(follower.seg + 1, len(WAYPOINTS) - 1)
            else:
                pin_since = None
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=THROTTLE, brake=0.0,
                                steering=follower.steering(x, y, yaw))
            mode = "drive"
        api.set_vehicle_control(vc)

        rad = math.hypot(x, y)
        if now > 3.0:
            perim_min = min(perim_min, rad)
            perim_max = max(perim_max, rad)

        every = 0.22 if mode == "burst" else args.shot_every
        if frames and now - last_shot >= every:
            last_shot = now
            tag += 1
            for cam in frames:
                img = getattr(cam, "frame", None)
                if img is None:
                    continue
                save_jpg(out / "f" / f"t{tag:04d}_cam{int(cam.id)}.jpg", img)
            poses.append((round(now, 2), round(x, 2), round(y, 2), round(yaw, 1),
                          round(v, 2), f"t{tag:04d}", mode))

        if now - last_log >= 1.0:
            last_log = now
            print(f"[{now:5.1f}s] {mode:<5} ({x:6.2f},{y:6.2f}) yaw={yaw:6.1f} "
                  f"v={v:4.2f} r={rad:5.1f} 距杆={dist_pole:5.1f} 帧={tag}", flush=True)

        if follower.done() and now > 12.0 and not backing:
            print(f"[{now:5.1f}s] 航点走完，驻车", flush=True)
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except OSError:
        pass

    with (out / "poses.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "yaw", "v", "tag", "mode"])
        w.writerows(poses)
    with (out / "pins.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "yaw", "seg"])
        w.writerows(pins)
    print(f"图片 {tag * 4} 张（{tag} 个时刻）在 {out / 'f'}", flush=True)
    print(f"卡死 {len(pins)} 次 {[(p[1], p[2]) for p in pins]}", flush=True)
    print(f"绕行半径范围 {perim_min:.1f}~{perim_max:.1f} m（区域半宽 {ZONE_HALF} m）",
          flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
