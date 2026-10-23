# -*- coding: utf-8 -*-
"""垂直泊车：一次性可执行版本，用来快速标定。

策略（按实测几何）：
  车道沿 y，车位轴线沿 x（ori_z=0），车位居于车道左侧
  1) 沿车道直行到车位正前方（y ≈ slot_y + lead）
  2) 用一串虚拟路点拐 90°，指向 -x 方向
  3) 低速开进车位，中心误差 <0.25 m 且航向误差 <10° 时刹停挂 P

用法：
    .venv\\Scripts\\python.exe -u tools/park_once.py
    （先跑脚本，再点客户端「启动场景」）
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = 30

# ---- 标定参数（先给保守值，跑完按日志改）----
SPEED_THROTTLE = 0.30      # 巡航油门
LOW_THROTTLE = 0.14        # 转弯/入库油门
LOOKAHEAD = 3.5            # 纯跟踪前视距离
BRAKE_GAIN = 0.6
TARGET_TOL = 0.25          # 中心误差阈值 m
HEADING_TOL_DEG = 10.0
LEAD_Y = 8.0               # 开始拐弯时，距车位中心的 y 距离
MAX_SECONDS = 120


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    """场景 yaw 顺时针为正，转成数学角。"""
    return math.radians(-yaw_deg)


def find_slot(obstacles):
    for ob in obstacles:
        v = getattr(getattr(ob, "type", None), "value", getattr(ob, "type", None))
        if v == PARKING_SLOT_TYPE:
            return ob
    return None


def build_waypoints(pose, slot):
    """沿车道 -> 拐角 -> 车位中心 -> 车位里再往里 1.5 m（保证车头朝 -x）。"""
    sx, sy = slot.pos_x, slot.pos_y
    lane_x = pose.pos_x
    pts = []

    # 1) 沿车道走到车位前方
    y = pose.pos_y
    while y > sy + LEAD_Y:
        y -= 3.0
        pts.append((lane_x, y))
    pts.append((lane_x, sy + LEAD_Y))

    # 2) 拐弯：四分之一圆弧
    r = 4.0
    cx, cy = lane_x, sy + LEAD_Y          # 圆弧起点
    for i in range(1, 7):
        # 从正前方转向 -x：圆心在左前方
        ang = (math.pi / 2) * (i / 6.0)
        px = cx - r + r * math.cos(ang)
        py = cy - r * math.sin(ang)
        pts.append((px, py))

    # 3) 直线进库
    pts.append((sx + 1.6, sy))
    pts.append((sx, sy))
    pts.append((sx - 1.2, sy))
    return pts


def steer_to(route, idx, pose, lookahead):
    """纯跟踪：取前视点算转角，符号与 tracking.py 一致（output 取负）。"""
    n = len(route)
    if idx >= n - 1:
        tx, ty = route[-1]
    else:
        tx, ty = route[idx]
        for j in range(idx, n):
            px, py = route[j]
            if math.hypot(px - pose.pos_x, py - pose.pos_y) >= lookahead:
                tx, ty = px, py
                break

    dx, dy = tx - pose.pos_x, ty - pose.pos_y
    dist = max(math.hypot(dx, dy), 1e-3)
    yaw = heading_rad(pose.ori_z)
    local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
    local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

    curvature = 2.0 * local_y / (dist * dist)
    delta = math.atan(curvature * 2.6)
    if local_x < 0:
        delta = 0.6 if local_y > 0 else -0.6
    return max(-1.0, min(1.0, -delta / 0.6))


def main() -> int:
    print("等待场景连接（先跑脚本，再点启动场景）...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    print(f"路线点 {len(static.route)}，子场景 {[s.name for s in static.sub_scenes]}")

    route = None
    slot_seen = None
    idx = 0
    start = time.time()
    last_log = 0.0

    for msg, _frames in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()

        if slot_seen is None:
            slot_seen = find_slot(msg.obstacles)
            if slot_seen is None:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                continue
            route = build_waypoints(pose, slot_seen)
            print(f"车位 ({slot_seen.pos_x:.2f}, {slot_seen.pos_y:.2f}) ori={slot_seen.ori_z:.1f}  "
                  f"路点 {len(route)} 个")
            for i, p in enumerate(route):
                print(f"  wp{i:>2} ({p[0]:7.2f}, {p[1]:7.2f})")

        # 到达判定
        dist_center = math.hypot(slot_seen.pos_x - pose.pos_x, slot_seen.pos_y - pose.pos_y)
        in_slot_x = abs(pose.pos_x - slot_seen.pos_x) < 2.2
        if in_slot_x and dist_center < TARGET_TOL:
            print(f"入库完成：中心误差 {dist_center:.2f} m，用时 {now - start:.1f}s")
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            break

        # 推进路点
        while idx < len(route) - 1:
            wx, wy = route[idx]
            if math.hypot(wx - pose.pos_x, wy - pose.pos_y) < 2.0:
                idx += 1
            else:
                break

        steer = steer_to(route, idx, pose, LOOKAHEAD)

        # 接近车位前减速
        near = math.hypot(route[-1][0] - pose.pos_x, route[-1][1] - pose.pos_y) < 8.0
        thr = LOW_THROTTLE if near else SPEED_THROTTLE
        if abs(steer) > 0.5:
            thr = min(thr, LOW_THROTTLE)

        vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, steering=steer)
        if v > 3.0:
            vc.throttle = 0.0
            vc.brake = min(1.0, BRAKE_GAIN * (v - 3.0))
        api.set_vehicle_control(vc)

        if now - last_log > 0.5:
            last_log = now
            print(f"[{now - start:5.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={v:4.2f} thr={vc.throttle:4.2f} "
                  f"str={steer:+5.2f} wp={idx} d={dist_center:6.2f}")

        if now - start > MAX_SECONDS:
            print("超时退出")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
