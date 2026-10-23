# -*- coding: utf-8 -*-
"""泊车 v3：开环定圆弧倒车，用来测定真实转弯半径与转向符号。

几何推导（垂直车位）：
    车位 (sx, sy)=(-7.8,-0.12)，轴线 +x；车道 x=lane_x=-2.16
    倒车圆弧半径 R = lane_x - sx = 5.64
    起点（对位点）= (lane_x, sy + R) = (-2.16, 5.52)
    恒定转角 δ 满足 R = L / tan(δ)，L=2.6 → tanδ=0.461 → δ≈0.432 rad
    steering = δ / 0.6 ≈ 0.72

用法：
    .venv\\Scripts\\python.exe -u tools/park_arc.py [--sign 1]
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = 30
WHEELBASE = 2.6
MAX_STEER = 0.6


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    return math.radians(-yaw_deg)


def find_slot(obs):
    for o in obs:
        v = getattr(getattr(o, "type", None), "value", getattr(o, "type", None))
        if v == PARKING_SLOT_TYPE:
            return o
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sign", type=float, default=1.0, help="转向符号，反了就传 -1")
    ap.add_argument("--radius", type=float, default=5.64)
    args = ap.parse_args()

    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    slot = None
    stage = None
    phase = "stage"
    steer_fixed = 0.0
    t0 = time.time()
    last = 0.0

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()

        if slot is None:
            slot = find_slot(msg.obstacles)
            if slot is None:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                continue
            R = abs(pose.pos_x - slot.pos_x)
            stage = (pose.pos_x, slot.pos_y + R)
            delta = math.atan(WHEELBASE / R)
            steer_fixed = args.sign * min(1.0, delta / MAX_STEER)
            print(f"车位({slot.pos_x:.2f},{slot.pos_y:.2f})  R={R:.2f}  "
                  f"steer_fixed={steer_fixed:+.2f}  对位点({stage[0]:.2f},{stage[1]:.2f})")

        d_center = math.hypot(slot.pos_x - pose.pos_x, slot.pos_y - pose.pos_y)
        dyaw = abs(math.degrees(wrap(heading_rad(pose.ori_z) - heading_rad(slot.ori_z))))

        if phase == "stage":
            d_stage = math.hypot(stage[0] - pose.pos_x, stage[1] - pose.pos_y)
            if d_stage < 0.7:
                phase = "arc"
                print(f"到位(d={d_stage:.2f})，切倒挡，定转角 {steer_fixed:+.2f}")
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                time.sleep(0.6)
            else:
                steer = 0.0
                dx, dy = stage[0] - pose.pos_x, stage[1] - pose.pos_y
                err = wrap(math.atan2(dy, dx) - heading_rad(pose.ori_z))
                steer = max(-1.0, min(1.0, -1.6 * err))
                thr = 0.30 if d_stage > 3 else 0.12
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, steering=steer)
                if v > 4.5:
                    vc.throttle, vc.brake = 0.0, 0.6
                api.set_vehicle_control(vc)

        elif phase == "arc":
            if d_center < 0.45 and dyaw < 20:
                phase = "hold"
                print(f"入库：中心 {d_center:.2f} m 航向 {dyaw:.1f}°")
            else:
                vc = VehicleControl(gear=GearMode.REVERSE, throttle=0.13, steering=steer_fixed)
                if abs(v) > 1.2:
                    vc.throttle, vc.brake = 0.0, 0.35
                api.set_vehicle_control(vc)

        else:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if now - t0 > 0:
                pass
            if d_center < 0.45 and dyaw < 20:
                print(f"完成：中心 {d_center:.2f} m，航向 {dyaw:.1f}°，用时 {now - t0:.1f}s")
                break

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] {phase:<6} pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={v:5.2f} d={d_center:5.2f} dyaw={dyaw:5.1f}")

        if now - t0 > 90:
            print("超时")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
