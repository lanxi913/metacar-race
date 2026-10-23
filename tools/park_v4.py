# -*- coding: utf-8 -*-
"""垂直泊车 v4：满舵单圆弧倒车 + 直线补齐。

实测（probe_max.py）：steer=+1.0 → R=4.20 m，比车位横向距离 5.64 m 小，几何可行。

路径：
    1) 车道内开到 (x=lane_x, y=slot_y+4.2)  ← 圆弧起点，正好 R 个车身
    2) 倒车、满舵 steer=+1.0，直到车头朝 +x（与车位轴线平行）
    3) 继续保持倒车、回正方向，直线退到车位中心
    4) 挂 P 保持 3 秒

用法：
    .venv\\Scripts\\python.exe -u tools/park_v4.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = 30
ARC_RADIUS = 5.26         # 倒车满舵实测半径（probe_reverse）
STAGE_SPEED = 0.28
REVERSE_SPEED = 0.13
TARGET_TOL = 0.30
HEADING_TOL_DEG = 8.0
HOLD_SECONDS = 3.0


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
    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    slot = None
    stage = None
    phase = "stage"
    t0 = time.time()
    hold_t = 0.0
    last = 0.0
    prev_d = 99.0

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()

        if slot is None:
            slot = find_slot(msg.obstacles)
            if slot is None:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                continue
            # 倒车左转会往 +y 方向退，所以起点要放在车位「下方」R 处
            stage = (pose.pos_x, slot.pos_y - ARC_RADIUS)
            print(f"车位({slot.pos_x:.2f},{slot.pos_y:.2f})  R={ARC_RADIUS}  "
                  f"圆弧起点({stage[0]:.2f},{stage[1]:.2f})")

        d_center = math.hypot(slot.pos_x - pose.pos_x, slot.pos_y - pose.pos_y)
        dyaw = abs(math.degrees(wrap(heading_rad(pose.ori_z) - heading_rad(slot.ori_z))))

        if phase == "stage":
            d_stage = math.hypot(stage[0] - pose.pos_x, stage[1] - pose.pos_y)
            if d_stage < 0.6:
                phase = "arc"
                print(f"到位(d={d_stage:.2f})，切倒挡满舵")
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                time.sleep(0.5)
            else:
                dx, dy = stage[0] - pose.pos_x, stage[1] - pose.pos_y
                err = wrap(math.atan2(dy, dx) - heading_rad(pose.ori_z))
                steer = max(-1.0, min(1.0, -1.5 * err))
                thr = STAGE_SPEED if d_stage > 3 else 0.10
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, steering=steer)
                if v > 4.5:
                    vc.throttle, vc.brake = 0.0, 0.6
                api.set_vehicle_control(vc)

        elif phase == "arc":
            # 倒车满舵，直到车头与车位轴线平行
            if dyaw < HEADING_TOL_DEG:
                phase = "straight"
                print(f"车头已摆正(dyaw={dyaw:.1f}°)，回正倒直线")
            vc = VehicleControl(gear=GearMode.REVERSE, throttle=REVERSE_SPEED, steering=1.0)
            if abs(v) > 0.9:
                vc.throttle, vc.brake = 0.0, 0.3
            api.set_vehicle_control(vc)

        elif phase == "straight":
            # 越接近车位越慢，避免冲过中心
            if d_center < TARGET_TOL or (prev_d < 0.9 and d_center > prev_d + 0.10):
                phase = "hold"
                hold_t = now
                print(f"入库：中心 {d_center:.2f} m 航向 {dyaw:.1f}°")
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            else:
                # 直退的同时用陀螺仪修正朝向
                err = wrap(heading_rad(slot.ori_z) - heading_rad(pose.ori_z))
                steer = max(-0.4, min(0.4, 1.2 * err))
                thr = min(REVERSE_SPEED, 0.02 + 0.035 * d_center)
                vc = VehicleControl(gear=GearMode.REVERSE, throttle=thr, steering=steer)
                if d_center < 1.0:
                    vc.brake = 0.25
                if abs(v) > 0.6:
                    vc.throttle, vc.brake = 0.0, 0.5
                api.set_vehicle_control(vc)

        else:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if now - hold_t > HOLD_SECONDS:
                print(f"保持完成，用时 {now - t0:.1f}s，中心误差 {d_center:.2f} m，"
                      f"航向误差 {dyaw:.1f}°")
                break

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] {phase:<8} pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={v:5.2f} d={d_center:5.2f} dyaw={dyaw:5.1f}")

        if now - t0 > 90:
            print("超时")
            break

        prev_d = d_center

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
