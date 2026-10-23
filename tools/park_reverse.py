# -*- coding: utf-8 -*-
"""垂直泊车 v2：倒车入库。

实测几何（垂直车道泊车）：
    车位 center=(-7.80, -0.12) ori_z=0（轴线沿 +x，开口朝 +x）
    车道 x≈-2.16（自车起点 x=-2.16, y=30，朝 -y）
    两侧已停车 (-8.37, ±3.0)

流程：
    1) 沿车道直行到车位侧前方（y = slot_y - 4.6）
    2) 停稳，切倒挡
    3) 倒车时用「反向纯跟踪」把车尾送进车位中心
    4) 中心误差 <0.35 m 且航向误差 <12° → 挂 P 保持 3 秒

用法：
    .venv\\Scripts\\python.exe -u tools/park_reverse.py
    （先跑脚本，再点「启动场景」）
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = 30

# 倒车时 steering 的正负号：先给 +1，观察 yaw 是否朝目标转，反了改 -1
REVERSE_STEER_SIGN = 1.0

STAGE_SPEED = 0.30        # 前进油门
REVERSE_SPEED = 0.16      # 倒车油门
STAGE_GAP = 5.7           # 对位点：车位中心 +5.7 m（圆弧半径 5.64，正好切进来）
TARGET_TOL = 0.35
HEADING_TOL_DEG = 12.0
HOLD_SECONDS = 3.0
MAX_SECONDS = 100


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    return math.radians(-yaw_deg)


def find_slot(obstacles):
    for ob in obstacles:
        v = getattr(getattr(ob, "type", None), "value", getattr(ob, "type", None))
        if v == PARKING_SLOT_TYPE:
            return ob
    return None


def steer_to_point(px, py, pose, gain=1.6):
    """正向纯跟踪到单点。"""
    dx, dy = px - pose.pos_x, py - pose.pos_y
    err = wrap(math.atan2(dy, dx) - heading_rad(pose.ori_z))
    return max(-1.0, min(1.0, -gain * err))


def steer_reverse(px, py, pose, gain=1.5):
    """倒车：让车尾朝目标点走。

    车头朝向 heading，倒车运动方向是 heading+180°。
    所以目标方位角应该等于 (heading + 180°) 才能直线倒退。
    """
    dx, dy = px - pose.pos_x, py - pose.pos_y
    bearing = math.atan2(dy, dx)
    desired = wrap(bearing - math.pi)          # 车头应朝的方向
    err = wrap(desired - heading_rad(pose.ori_z))
    return max(-1.0, min(1.0, REVERSE_STEER_SIGN * gain * err))


def main() -> int:
    print("等待场景连接（先跑脚本，再点启动场景）...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    print(f"路线点 {len(static.route)}  子场景 {[s.name for s in static.sub_scenes]}")

    slot = None
    phase = "stage"
    stage_pt = None
    t0 = time.time()
    hold_start = 0.0
    last_log = 0.0

    for msg, _frames in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()
        dt = now - t0

        if slot is None:
            slot = find_slot(msg.obstacles)
            if slot is None:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                continue
            stage_pt = (pose.pos_x, slot.pos_y + STAGE_GAP)
            print(f"车位 ({slot.pos_x:.2f},{slot.pos_y:.2f}) ori={slot.ori_z:.1f}  "
                  f"对位点 ({stage_pt[0]:.2f},{stage_pt[1]:.2f})")

        d_center = math.hypot(slot.pos_x - pose.pos_x, slot.pos_y - pose.pos_y)
        dyaw = abs(math.degrees(wrap(heading_rad(pose.ori_z) - heading_rad(slot.ori_z))))

        # ---------- 阶段 1：沿车道开到对位点 ----------
        if phase == "stage":
            d_stage = math.hypot(stage_pt[0] - pose.pos_x, stage_pt[1] - pose.pos_y)
            if d_stage < 0.8:
                phase = "reverse"
                print(f"到位（d={d_stage:.2f}），切倒挡")
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                time.sleep(0.8)
            else:
                steer = steer_to_point(stage_pt[0], stage_pt[1], pose)
                thr = STAGE_SPEED if d_stage > 3 else 0.12
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, steering=steer)
                if v > 4.0:
                    vc.throttle = 0.0
                    vc.brake = 0.6
                api.set_vehicle_control(vc)

        # ---------- 阶段 2：倒车入库 ----------
        elif phase == "reverse":
            if d_center < TARGET_TOL and dyaw < HEADING_TOL_DEG:
                phase = "hold"
                hold_start = now
                print(f"入库完成：中心 {d_center:.2f} m，航向 {dyaw:.1f}°，开始保持")
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            else:
                # 先对准车位开口，再直线倒进去；避免带着横向偏差扎进去
                if dyaw > 10.0:
                    aim_x, aim_y = slot.pos_x + 2.6, slot.pos_y
                else:
                    aim_x, aim_y = slot.pos_x, slot.pos_y
                steer = steer_reverse(aim_x, aim_y, pose)
                thr = REVERSE_SPEED if d_center > 1.2 else 0.09
                vc = VehicleControl(gear=GearMode.REVERSE, throttle=thr, steering=steer)
                if abs(v) > 1.1:
                    vc.throttle = 0.0
                    vc.brake = 0.4
                api.set_vehicle_control(vc)

        # ---------- 阶段 3：驻车保持 ----------
        else:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if now - hold_start > HOLD_SECONDS:
                print(f"保持 {HOLD_SECONDS:.0f} 秒完成，退出。中心误差 {d_center:.2f} m，"
                      f"航向误差 {dyaw:.1f}°")
                break

        if now - last_log > 0.5:
            last_log = now
            print(f"[{dt:5.1f}s] {phase:<7} pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={v:5.2f} d={d_center:6.2f} dyaw={dyaw:5.1f}")

        if dt > MAX_SECONDS:
            print("超时退出")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
