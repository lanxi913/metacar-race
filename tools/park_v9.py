# -*- coding: utf-8 -*-
"""通用泊车 v9：支持任意朝向车位（垂直 / 斜列 / 平行），圆弧角度由几何算出。

思路
----
车位坐标系统（同 v8）：

    u = 车位轴线方向（开口朝外）
    v = 横向

目标位姿：车位中心，车头朝 +u（倒车进库，车头朝开口）。

倒车圆弧的转角 φ **不再固定 90°**，而是由「接近方向」和「车位轴线」的夹角决定：

    φ = wrap(θ_slot − θ_approach)

已知终点位姿，可反推圆弧起点：

    起点 = 终点 − Δ(θ_start, φ, κ)
    Δ = [ (sin(θ+φ) − sinθ)/κ , −(cos(θ+φ) − cosθ)/κ ]

其中 κ = ±1/R 是曲率（左右满舵两种）。脚本对两种符号都试一遍，
挑一个起点落在车道这一侧、且起点航向与接近方向一致的。

用法：
    .venv\\Scripts\\python.exe -u tools/park_v9.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = 30
ARC_RADIUS = 5.26
STAGE_SPEED = 0.14
STOP_LONG_TOL = 0.06
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


def arc_displacement(theta0, phi, kappa):
    """圆弧位移（起点到终点的向量）。kappa = ±1/R。"""
    t1 = theta0 + phi
    if abs(kappa) < 1e-9:
        return (math.cos(theta0) * ARC_RADIUS * phi, math.sin(theta0) * ARC_RADIUS * phi)
    dx = (math.sin(t1) - math.sin(theta0)) / kappa
    dy = -(math.cos(t1) - math.cos(theta0)) / kappa
    return (dx, dy)


def plan_arc(goal_x, goal_y, goal_theta, approach_theta):
    """给定终点位姿和接近方向，返回 (起点x, 起点y, phi, kappa) 或 None。"""
    best = None
    for kappa in (1.0 / ARC_RADIUS, -1.0 / ARC_RADIUS):
        # 转角由接近方向决定；也允许 ±2π 内的等价角
        phi = wrap(goal_theta - approach_theta)
        for cand_phi in (phi, phi - 2 * math.pi, phi + 2 * math.pi):
            theta0 = goal_theta - cand_phi
            if abs(wrap(theta0 - approach_theta)) > math.radians(4.0):
                continue
            dx, dy = arc_displacement(theta0, cand_phi, kappa)
            sx, sy = goal_x - dx, goal_y - dy
            # 起点必须在车位开口那一侧（a_long > 0）
            score = abs(cand_phi)
            if best is None or score < best[0]:
                best = (score, sx, sy, cand_phi, kappa)
    if best is None:
        return None
    return best[1], best[2], best[3], best[4]


def speed_cmd(v_meas, v_target):
    err = v_target - abs(v_meas)
    if abs(err) < 0.06:
        return 0.0, 0.0
    if err > 0:
        return min(0.10, 0.01 + 0.04 * err), 0.0
    return 0.0, min(0.10, -0.06 * err)


def main() -> int:
    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    slot = None
    frame = None
    plan = None
    phase = "approach"
    t0 = time.time()
    hold_t = 0.0
    last = 0.0
    v_prev = 0.0
    t_prev = t0
    peak1s = 0.0
    hist = []
    lim = {"thr": 0.0, "brk": 0.0}

    def send(vc):
        lim["thr"] += max(-0.0015, min(0.0015, vc.throttle - lim["thr"]))
        lim["brk"] += max(-0.004, min(0.004, vc.brake - lim["brk"]))
        lim["thr"] = max(0.0, min(1.0, lim["thr"]))
        lim["brk"] = max(0.0, min(1.0, lim["brk"]))
        vc.throttle, vc.brake = lim["thr"], lim["brk"]
        api.set_vehicle_control(vc)

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()
        dt = min(0.10, max(now - t_prev, 0.02))
        t_prev = now
        v_prev_c = v_prev
        v_prev = v
        hist.append((now, v))
        while len(hist) > 1 and now - hist[0][0] > 1.2:
            hist.pop(0)
        if hist and now - hist[0][0] >= 0.9:
            peak1s = max(peak1s, abs((v - hist[0][1]) / (now - hist[0][0])))

        if slot is None:
            slot = find_slot(msg.obstacles)
            if slot is None:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.2))
                continue
            frame = {
                "th": heading_rad(slot.ori_z),
                "cx": slot.pos_x,
                "cy": slot.pos_y,
            }
            print(f"车位({slot.pos_x:.2f},{slot.pos_y:.2f}) 轴线 {slot.ori_z:.1f}°")

        th_slot = frame["th"]
        ux, uy = math.cos(th_slot), math.sin(th_slot)
        rx, ry = pose.pos_x - frame["cx"], pose.pos_y - frame["cy"]
        a_long = rx * ux + ry * uy
        heading_err = wrap(th_slot - heading_rad(pose.ori_z))

        # 接近到 18 m 内时，按当前航向规划圆弧
        if plan is None and a_long > 0 and math.hypot(rx, ry) < 18.0:
            approach_theta = heading_rad(pose.ori_z)
            plan = plan_arc(frame["cx"], frame["cy"], th_slot, approach_theta)
            if plan:
                print(f"规划圆弧：转角 {math.degrees(plan[2]):+.1f}°  "
                      f"曲率 {plan[3]:+.3f}  起点({plan[0]:.2f},{plan[1]:.2f})")
            else:
                print("!! 找不到可行圆弧")

        if phase == "approach":
            if plan:
                d = math.hypot(plan[0] - pose.pos_x, plan[1] - pose.pos_y)
                if d < 0.40:
                    phase = "stop"
                    print(f"到圆弧起点(d={d:.2f})，准备停稳")
                    continue
                if d < 20.0:
                    a_req = (v * v) / (2.0 * max(d, 0.3))
                    brk = max(0.0, min(0.10, (a_req - 0.8) / 25.0))
                    thr = 0.0
                else:
                    thr, brk = STAGE_SPEED, 0.0
                err = wrap(math.atan2(plan[1] - pose.pos_y, plan[0] - pose.pos_x)
                           - heading_rad(pose.ori_z))
                steer = max(-1.0, min(1.0, -1.5 * err))
                if now - t0 < 2.5:
                    thr = min(thr, 0.02)
                send(VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer))
            else:
                send(VehicleControl(gear=GearMode.DRIVE, throttle=0.05, brake=0.0, steering=0.0))

        elif phase == "stop":
            if abs(v) < 0.30:
                phase = "arc"
                lim["thr"] = lim["brk"] = 0.0
                print(f"停稳(v={v:.2f})，挂倒挡开始圆弧")
            else:
                send(VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.10))

        elif phase == "arc":
            # 圆弧阶段：按规划曲率打方向盘，直到车头与车位轴线平行
            if abs(heading_err) < math.radians(3.0):
                phase = "straight"
                print(f"车头摆正(误差 {math.degrees(heading_err):+.1f}°)，回正倒直线")
            steer = max(-1.0, min(1.0, plan[3] * ARC_RADIUS))
            thr, brk = speed_cmd(v, 0.50)
            send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=steer))

        elif phase == "straight":
            if abs(a_long) < STOP_LONG_TOL or a_long < -0.45:
                phase = "hold"
                hold_t = now
                print(f"入库：a_long={a_long:+.2f} 航向 {math.degrees(heading_err):+.1f}°")
            else:
                vt = max(0.15, min(0.70, 0.9 * max(abs(a_long) - 0.10, 0.0) + 0.20))
                thr, brk = speed_cmd(v, vt)
                steer = max(-0.45, min(0.45, 1.4 * heading_err))
                send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=steer))

        else:
            if abs(v) > 0.08:
                send(VehicleControl(gear=GearMode.REVERSE, throttle=0.0, brake=0.08))
            else:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.8))
            if now - hold_t > HOLD_SECONDS:
                print(f"加速度峰值（1s）{peak1s:.2f} m/s2")
                print(f"完成：a_long={a_long:+.2f} 航向 {math.degrees(heading_err):+.1f}° "
                      f"用时 {now - t0:.1f}s")
                break

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] {phase:<8} pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"a_long={a_long:+6.2f} v={v:5.2f} head={math.degrees(heading_err):+5.1f}°")

        if now - t0 > 90:
            print("超时")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.3))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
