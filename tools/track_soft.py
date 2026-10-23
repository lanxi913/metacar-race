# -*- coding: utf-8 -*-
"""绝对保守版循迹：用最小油门恒定速度跑，几乎不用刹车，结构上保证 |a| 很小。

设计（目标是「绝对」不超 5 m/s²，宁可慢）：
  1. 目标速度固定 VMAX（默认 2.0 m/s），不做任何速度突变
  2. 油门 = 0.02 + 0.30 * (VMAX - v)，上限 0.12，每帧只允许变 0.0005
  3. 只有明显超速（v > VMAX + 0.4）才给 0.03 刹车，每帧只允许变 0.001
  4. 方向盘每帧最多变 0.02（避免横向加速度突变）
  5. 不做终点急减速：接近终点时只把目标速度线性降到 0.3

用法：
    .venv\\Scripts\\python.exe -u tools/track_soft.py [--vmax 2.0]
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

LOOKAHEAD = 4.0


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    return math.radians(-yaw_deg)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vmax", type=float, default=2.0)
    ap.add_argument("--seconds", type=float, default=240)
    args = ap.parse_args()

    api = SceneAPI()
    print("等待场景连接...")
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    route = [(p.x, p.y) for p in static.route]
    print(f"路线点 {len(route)}  子场景 {[s.name for s in static.sub_scenes]}")

    idx = 0
    t0 = time.time()
    last = 0.0
    lim = {"thr": 0.0, "brk": 0.0, "steer": 0.0}
    peak_a = 0.0
    v_prev = 0.0
    t_prev = t0

    def send(vc):
        lim["thr"] += max(-0.0005, min(0.0005, vc.throttle - lim["thr"]))
        lim["brk"] += max(-0.001, min(0.001, vc.brake - lim["brk"]))
        lim["steer"] += max(-0.02, min(0.02, vc.steering - lim["steer"]))
        lim["thr"] = max(0.0, min(1.0, lim["thr"]))
        lim["brk"] = max(0.0, min(1.0, lim["brk"]))
        lim["steer"] = max(-1.0, min(1.0, lim["steer"]))
        vc.throttle, vc.brake, vc.steering = lim["thr"], lim["brk"], lim["steer"]
        api.set_vehicle_control(vc)

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()
        dt = min(0.10, max(now - t_prev, 0.02))
        t_prev = now
        peak_a = max(peak_a, abs((v - v_prev) / dt))
        v_prev = v

        while idx < len(route) - 1:
            if math.hypot(route[idx][0] - pose.pos_x, route[idx][1] - pose.pos_y) < 3.0:
                idx += 1
            else:
                break

        tx, ty = route[min(idx, len(route) - 1)]
        for j in range(idx, len(route)):
            if math.hypot(route[j][0] - pose.pos_x, route[j][1] - pose.pos_y) >= LOOKAHEAD:
                tx, ty = route[j]
                break

        dx, dy = tx - pose.pos_x, ty - pose.pos_y
        d = max(math.hypot(dx, dy), 1e-3)
        yaw = heading_rad(pose.ori_z)
        lx = dx * math.cos(yaw) + dy * math.sin(yaw)
        ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
        if lx < 0:
            delta = 0.5 if ly > 0 else -0.5
        else:
            delta = math.atan(2.0 * ly / d)
        steer = max(-1.0, min(1.0, -delta / 0.6))

        dist_left = math.hypot(route[-1][0] - pose.pos_x, route[-1][1] - pose.pos_y)
        vt = args.vmax
        if dist_left < 6.0:
            vt = max(0.3, min(args.vmax, 0.35 * dist_left))

        thr = min(0.12, 0.02 + 0.30 * max(vt - v, 0.0))
        brk = 0.03 if v > vt + 0.4 else 0.0

        # 起步：先用刹车压住，再按 0.06/s 线性放开，让车速慢慢爬升
        if now - t0 < 5.0:
            brk = max(brk, 0.28 - 0.055 * (now - t0))
            thr = min(thr, 0.02)
        send(VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer))

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) v={v:5.2f} "
                  f"vt={vt:4.1f} thr={lim['thr']:5.3f} brk={lim['brk']:5.3f} left={dist_left:5.1f}")

        if dist_left < 0.8 and abs(v) < 0.25:
            print(f"到达终点，用时 {now - t0:.1f}s，逐帧加速度峰值 {peak_a:.2f}")
            break
        if now - t0 > args.seconds:
            print(f"超时退出（剩余 {dist_left:.1f} m）")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.2))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
