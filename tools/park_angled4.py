# -*- coding: utf-8 -*-
"""异形泊车 v4：先规划路径，再用纯跟踪跑（路径驱动，不再用 if-else 拼阶段）。

路径（世界坐标折线）：
  A 前进段   当前位姿 → 左车道直行 → 车位前方 staging 点 S
  B 倒车段   S →(三次贝塞尔)→ 车位中心 C，起点切线沿车道、终点切线沿车位轴线

跟踪：沿用已验证过的纯跟踪公式（track_bonus 那套，拿到过 110）
    yaw = -ori_z（场景顺时针为正）
    lx =  dx*cos + dy*sin ,  ly = -dx*sin + dy*cos
    delta = atan2(2*ly, d) ,  steer = -delta/0.6      # 前进
倒车段把 steering 取反（REV_SIGN），因为车在往后走。

速度：只用油门开关调速，不踩刹车；换向时先滑行到 |v|<0.08 再换挡。
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

R_REV = 5.26          # 满舵倒车半径
LANE_OFFSET = 2.15    # 车道中心相对车位中心的横向偏移
CRUISE = 0.55
LOOKAHEAD = 3.0
REV_SIGN = -1.0       # 倒车时转向取反


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    return math.radians(-yaw_deg)


def find_slot(obstacles):
    for ob in obstacles or []:
        t = getattr(ob, "type", None)
        name = getattr(t, "name", None) or str(t)
        if "PARK" in name.upper():
            return (ob.pos_x, ob.pos_y, ob.ori_z)
    return None


def bezier(p0, p1, p2, p3, n=40):
    pts = []
    for i in range(n + 1):
        t = i / n
        mt = 1 - t
        x = (mt**3)*p0[0] + 3*(mt**2)*t*p1[0] + 3*mt*(t**2)*p2[0] + (t**3)*p3[0]
        y = (mt**3)*p0[1] + 3*(mt**2)*t*p1[1] + 3*mt*(t**2)*p2[1] + (t**3)*p3[1]
        pts.append((x, y))
    return pts


def follow(path, idx, pose, direction, ld=LOOKAHEAD):
    """返回 (steer, 新idx, 剩余距离)。direction=+1 前进 / -1 倒车。"""
    n = len(path)
    # 找最近点
    best_i, best_d = idx, 1e9
    for i in range(idx, min(n, idx + 40)):
        d = math.hypot(path[i][0] - pose.pos_x, path[i][1] - pose.pos_y)
        if d < best_d:
            best_d, best_i = d, i
    # 前视点
    j = best_i
    acc = 0.0
    while j + direction >= 0 and j + direction < n and acc < ld:
        acc += math.hypot(path[j + direction][0] - path[j][0], path[j + direction][1] - path[j][1])
        j += direction
    tx, ty = path[j]
    dx, dy = tx - pose.pos_x, ty - pose.pos_y
    d = max(math.hypot(dx, dy), 1e-3)
    yaw = heading_rad(pose.ori_z)
    lx = dx * math.cos(yaw) + dy * math.sin(yaw)
    ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
    delta = math.atan2(2.0 * ly, d)
    steer = -delta / 0.6
    if direction < 0:
        steer *= REV_SIGN
    return max(-1.0, min(1.0, steer)), best_i, math.hypot(path[-1][0] - pose.pos_x, path[-1][1] - pose.pos_y)


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...")
    api.connect()
    print("已连接。")

    lim = {"thr": 0.0, "brk": 0.0}

    def send(gear, thr, brk, steer):
        lim["thr"] += max(-0.02, min(0.02, thr - lim["thr"]))
        lim["brk"] += max(-0.02, min(0.02, brk - lim["brk"]))
        lim["thr"] = max(0.0, min(1.0, lim["thr"]))
        lim["brk"] = max(0.0, min(1.0, lim["brk"]))
        api.set_vehicle_control(VehicleControl(gear=gear, throttle=lim["thr"],
                                               brake=lim["brk"], steering=steer))

    path_f = None
    path_r = None
    idx_f = idx_r = 0
    phase = "FWD"
    t0 = time.time()
    last = 0.0

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()

        if path_f is None:
            s = find_slot(msg.obstacles)
            if s is None:
                send(GearMode.PARKING, 0.0, 0.2, 0.0)
                continue
            cx, cy, ori = s
            th = heading_rad(ori)
            ux, uy = math.cos(th), math.sin(th)          # 车位轴线（指向车位内部）
            nx, ny = -uy, ux                              # 法向
            # 车道方向：车头当前朝向（yaw 90 → 沿 -y）
            lane = heading_rad(pose.ori_z)
            lx_, ly_ = math.cos(lane), math.sin(lane)
            # staging 点：车位中心沿轴线外 R，再沿车道法向偏一个车道
            sx = cx - ux * (R_REV * 0.6) - nx * LANE_OFFSET * (1 if (pose.pos_x - cx) * nx > 0 else -1)
            sy = cy - uy * (R_REV * 0.6) - ny * LANE_OFFSET * (1 if (pose.pos_x - cx) * nx > 0 else -1)
            # 前进段：当前点 → 左车道直行 → S
            p0 = (pose.pos_x, pose.pos_y)
            p1 = (pose.pos_x + (1 if (pose.pos_x - cx) * nx < 0 else -1) * 2.0, pose.pos_y + ly_ * 8.0)
            path_f = [p0, p1, (sx + lx_ * 6.0, sy + ly_ * 6.0), (sx, sy)]
            # 倒车段：S → C，起点切线沿车道反向，终点切线沿 -u（倒进车位）
            path_r = bezier((sx, sy), (sx - lx_ * 4.0, sy - ly_ * 4.0),
                            (cx + ux * 2.0, cy + uy * 2.0), (cx, cy), 40)
            print(f"车位({cx:.2f},{cy:.2f}) ori={ori:.1f}  车道方向({lx_:.2f},{ly_:.2f})")
            print(f"staging=({sx:.2f},{sy:.2f})  前进段 {len(path_f)} 点  倒车段 {len(path_r)} 点")

        if phase == "FWD":
            steer, idx_f, left = follow(path_f, idx_f, pose, +1)
            thr = 0.10 if v < CRUISE else 0.0
            send(GearMode.DRIVE, thr, 0.0, steer)
            if now - last > 0.8:
                last = now
                print(f"[{now - t0:5.1f}s] FWD pos=({pose.pos_x:.2f},{pose.pos_y:.2f}) "
                      f"yaw={pose.ori_z:.1f} v={v:.2f} steer={steer:+.2f} 剩余 {left:.1f} m")
            if left < 0.5:
                print(f"[{now - t0:5.1f}s] 到达 staging，滑行停稳后倒车")
                phase = "STOP1"
            continue

        if phase == "STOP1":
            send(GearMode.DRIVE, 0.0, 0.0, 0.0)
            if abs(v) < 0.08:
                phase = "REV"
                print(f"[{now - t0:5.1f}s] 已停稳，开始倒车")
            continue

        if phase == "REV":
            steer, idx_r, left = follow(path_r, idx_r, pose, -1)
            thr = 0.10 if abs(v) < CRUISE else 0.0
            send(GearMode.REVERSE, thr, 0.0, steer)
            if now - last > 0.8:
                last = now
                print(f"[{now - t0:5.1f}s] REV pos=({pose.pos_x:.2f},{pose.pos_y:.2f}) "
                      f"yaw={pose.ori_z:.1f} v={v:.2f} steer={steer:+.2f} 剩余 {left:.1f} m")
            if left < 0.35:
                phase = "HOLD"
            continue

        if phase == "HOLD":
            if abs(v) < 0.06:
                send(GearMode.PARKING, 0.0, 0.30, 0.0)
                print(f"[{now - t0:5.1f}s] 驻车 pos=({pose.pos_x:.2f},{pose.pos_y:.2f}) yaw={pose.ori_z:.1f}")
                break
            send(GearMode.REVERSE, 0.0, 0.10, 0.0)
            continue

        if now - t0 > 180:
            print("超时")
            break

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
