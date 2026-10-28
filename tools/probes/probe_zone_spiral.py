"""功能区探针 v8：到中心 → 螺旋向外扫，撞到车就停拍 + 倒车侧移继续。

用户观察：场内 4 辆蓝轿车 + 1 辆三轮车，基本停在**功能区边缘**。所以路线不是
"横穿车群"，而是"先到中心，再以中心为圆心、半径逐渐增大往外扫"。

行为
----
* 起点 (34.6,-4.0) → 直线到中心 (0,0)；
* 之后阿基米德螺旋：每 15° 半径 +0.18 m，从 r=4 到 r=21，约 4.5 圈；
* 卡死判据：油门 > 0 且 |v| < 0.15 持续 1.2 s ⇒ 视为撞到车：
  原地 P 挡连拍 6 s（0.25 s 一张，四路）→ 倒车 2.2 s → 余下航点整体侧移 5 m 继续；
* 单航点 22 s 没到就强制跳下一个（防止原地耗死）；
* 0.4 s 一张四路 JPEG ⇒ `<out>/f/tNNNN_camK.jpg`；位姿 `<out>/poses.csv`；卡死点 `<out>/pins.csv`。

用法：python tools/probe_zone_spiral.py --seconds 110 --out <目录>
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
from plate import find_plates  # noqa: E402

ENTRY = (34.6, -4.0)
CENTER = (0.0, 0.0)
THROTTLE = 0.03
PIN_SPEED = 0.15
PIN_SECONDS = 1.2
BURST_S = 6.0
RECOVER_BACK_S = 2.2
RECOVER_SHIFT_M = 5.0
ZONE_HALF = 23.0
WAYPOINT_TIMEOUT_S = 22.0


def build_spiral(r0: float = 4.0, r1: float = 21.0, step_deg: float = 15.0,
                 turns: float = 4.5) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    n = int(360.0 * turns / step_deg)
    dr = (r1 - r0) / max(n, 1)
    r, deg = r0, 0.0
    for _ in range(n + 1):
        rad = math.radians(deg)
        pts.append((r * math.cos(rad), r * math.sin(rad)))
        deg += step_deg
        r += dr
    return pts


def offset_remaining(follower: WpFollower, x: float, y: float, dist: float, side: float) -> None:
    """把 follower 余下航点整体沿「当前→当前航点」的法向平移，并夹在功能区里。"""
    if follower.seg >= len(follower.wps):
        return
    wx, wy = follower.wps[follower.seg]
    dx, dy = wx - x, wy - y
    n = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / n, dx / n
    for i in range(follower.seg, len(follower.wps)):
        px, py = follower.wps[i]
        qx = max(-ZONE_HALF, min(ZONE_HALF, px + side * nx * dist))
        qy = max(-ZONE_HALF, min(ZONE_HALF, py + side * ny * dist))
        follower.wps[i] = (qx, qy)


def save_jpg(path: Path, img: np.ndarray, quality: int = 82) -> bool:
    try:
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return False
        path.parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_bytes(buf.tobytes())
        return True
    except OSError:
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=110.0)
    ap.add_argument("--out", default=str(Path.home() / "zone_spiral"))
    ap.add_argument("--shot-every", type=float, default=0.4)
    ap.add_argument("--lookahead", type=float, default=6.0)
    ap.add_argument("--accept", type=float, default=4.0)
    args = ap.parse_args()

    pts = [CENTER] + build_spiral()
    out = Path(args.out)
    (out / "f").mkdir(parents=True, exist_ok=True)
    for old in (out / "f").glob("*.jpg"):
        old.unlink()

    print(f"航点 {len(pts)} 个：中心 → 螺旋 r=4→21 m，约 4.5 圈", flush=True)

    from metacar import GearMode, SceneAPI, VehicleControl

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"场景: {[s.name for s in static.sub_scenes]}", flush=True)
    if static.vla_extension is not None:
        for reg in static.vla_extension.regions:
            print(f"  区域 id={reg.id} type={reg.type} pos=({reg.pos_x},{reg.pos_y}) "
                  f"{reg.length}x{reg.width} ori_z={reg.ori_z}", flush=True)

    follower = WpFollower(pts, args.lookahead, args.accept)
    t0 = time.time()
    tag = 0
    last_shot = -1e9
    last_log = -1e9
    last_seg = 0
    last_prog = 0.0
    pin_since: float | None = None
    burst_until = 0.0
    recover_until = 0.0
    shift_side = 1.0
    poses: list[tuple] = []
    pins: list[tuple] = []
    n_cand = 0

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))
        cams = msg.sensor.ego_rgb_cams if msg.sensor else []

        if now > args.seconds:
            print(f"[{now:5.1f}s] 到时间，驻车", flush=True)
            break

        follower.update(x, y)
        if follower.seg != last_seg:
            last_seg = follower.seg
            last_prog = now
            print(f"[{now:5.1f}s] 到点 → 下一个 {pts[min(follower.seg, len(pts) - 1)]} "
                  f"剩余 {follower.remain_m(x, y):.1f} m", flush=True)
        if follower.done():
            print(f"[{now:5.1f}s] 航点走完", flush=True)
            break
        if now - last_prog > WAYPOINT_TIMEOUT_S:
            print(f"[{now:5.1f}s] !! {WAYPOINT_TIMEOUT_S:.0f} s 没到点，强制跳下一个", flush=True)
            follower.seg = min(follower.seg + 1, len(pts) - 1)
            last_prog = now

        # 卡死 → 连拍 + 倒车 + 侧移
        if now >= recover_until and now > 3.0 and v < PIN_SPEED:
            if pin_since is None:
                pin_since = now
            elif now - pin_since > PIN_SECONDS:
                print(f"[{now:5.1f}s] !! 卡死 ({x:.2f},{y:.2f}) yaw={yaw:.1f} "
                      f"→ 连拍 {BURST_S:.0f}s + 倒车", flush=True)
                pins.append((f"{now:.2f}", f"{x:.2f}", f"{y:.2f}", f"{yaw:.1f}"))
                burst_until = now + BURST_S
                recover_until = burst_until + RECOVER_BACK_S
                pin_since = None
                shift_side = -shift_side
                offset_remaining(follower, x, y, RECOVER_SHIFT_M, shift_side)
                print(f"[{now:5.1f}s] 余下航点侧移 {RECOVER_SHIFT_M:.0f} m（side={shift_side:+.0f}）",
                      flush=True)
        else:
            pin_since = None

        if now < burst_until:
            vc = VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0, steering=0.0)
            mode = "burst"
        elif now < recover_until:
            vc = VehicleControl(gear=GearMode.REVERSE, throttle=0.035, brake=0.0, steering=0.0)
            mode = "back"
        else:
            steer = follower.steering(x, y, yaw)
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=THROTTLE, brake=0.0,
                                steering=steer)
            mode = "drive"
        api.set_vehicle_control(vc)

        if frames and now - last_shot >= args.shot_every:
            last_shot = now
            tag += 1
            for idx, fr in enumerate(frames):
                if fr.frame is None:
                    continue
                cam_id = cams[idx].id if idx < len(cams) else idx
                save_jpg(out / "f" / f"t{tag:04d}_cam{cam_id}.jpg", fr.frame)
                n_cand += len(find_plates(fr.frame))
            poses.append((f"{now:.2f}", f"{x:.3f}", f"{y:.3f}", f"{yaw:.1f}",
                          f"{v:.3f}", f"t{tag:04d}", mode))

        if now - last_log >= 2.0:
            last_log = now
            print(f"[{now:5.1f}s] ({x:7.2f},{y:7.2f}) yaw={yaw:6.1f} v={v:4.2f} {mode:5s} "
                  f"剩余={follower.remain_m(x, y):5.1f} m 图={tag * 4} 牌={n_cand}", flush=True)

    with (out / "poses.csv").open("w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "yaw", "v", "tag", "mode"])
        w.writerows(poses)
    with (out / "pins.csv").open("w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "yaw"])
        w.writerows(pins)

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except OSError:
        pass
    print(f"图片 {tag * 4} 张在 {out / 'f'}；卡死 {len(pins)} 次；牌候选 {n_cand}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
