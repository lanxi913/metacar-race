# -*- coding: utf-8 -*-
"""垂直车道泊车 v5：全程速度闭环，追求平滑（|a| 小、无顿挫）。

设计要点：
  1. 所有阶段都用「目标速度 -> 油门/刹车」的连续映射，不再 超速就刹、不超就踩
  2. 目标速度随距离平滑衰减，刹车只在快到位时用，且是线性加压
  3. 换挡前必须先停稳（避免 ±13 m/s² 的换挡冲击）
  4. 每帧输出 |a|，结束时汇总峰值

几何（实测）：
  车位 (-7.80, -0.12)，轴线 +x，开口朝 +x
  车道 x ≈ -2.16，倒车满舵半径 R = 5.26 m
  对位点 = (车道 x, 车位 y - R + 0.75)   ← 0.75 是圆弧终点补偿

用法：
    .venv\\Scripts\\python.exe -u tools/park_v5.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = 30
ARC_RADIUS = 5.26
ARC_END_Y_FIX = 0.75

V_APPROACH = 4.2       # 接近段目标速度 m/s
V_ARC = 0.85           # 圆弧段目标速度
V_MIN = 0.22           # 最小爬行速度
STOP_TOL = 0.18        # 深度判据 |dx|
BRAKE_DIST = 7.0       # 提前减速距离
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


def speed_cmd(v_meas, v_target):
    """连续速度闭环：返回 (throttle, brake)，两者不会同时非零。"""
    err = v_target - abs(v_meas)
    if err > 0:
        return min(0.32, 0.03 + 0.16 * err), 0.0
    return 0.0, min(0.70, -0.40 * err)


def main() -> int:
    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    slot = None
    stage = None
    phase = "approach"
    t0 = time.time()
    hold_t = 0.0
    last = 0.0
    v_prev = 0.0
    t_prev = t0
    accel = 0.0
    peak = 0.0
    # 控制量状态，用于限斜率
    thr_state = 0.0
    brk_state = 0.0

    def send(vc, dt):
        nonlocal thr_state, brk_state
        thr_state += max(-0.05, min(0.05, vc.throttle - thr_state))
        brk_state += max(-0.10, min(0.10, vc.brake - brk_state))
        thr_state = max(0.0, min(1.0, thr_state))
        brk_state = max(0.0, min(1.0, brk_state))
        vc.throttle, vc.brake = thr_state, brk_state
        api.set_vehicle_control(vc)

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()
        dt = max(now - t_prev, 1e-3)
        t_prev = now
        a_raw = (v - v_prev) / dt
        v_prev = v
        accel = 0.85 * accel + 0.15 * max(-12.0, min(12.0, a_raw))
        peak = max(peak, abs(accel))

        if slot is None:
            slot = find_slot(msg.obstacles)
            if slot is None:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                continue
            stage = (pose.pos_x, slot.pos_y - ARC_RADIUS + ARC_END_Y_FIX)
            print(f"车位({slot.pos_x:.2f},{slot.pos_y:.2f})  R={ARC_RADIUS}  "
                  f"对位点({stage[0]:.2f},{stage[1]:.2f})")

        dx = pose.pos_x - slot.pos_x
        dy = pose.pos_y - slot.pos_y
        d_center = math.hypot(dx, dy)
        dyaw = abs(math.degrees(wrap(heading_rad(pose.ori_z) - heading_rad(slot.ori_z))))
        d_stage = math.hypot(stage[0] - pose.pos_x, stage[1] - pose.pos_y)

        # ---------------- 1) 接近段：平滑减速到对位点 ----------------
        if phase == "approach":
            if d_stage < 0.35:
                phase = "stop"
                print(f"到达对位点(d={d_stage:.2f})，准备停稳")
                continue
            # 目标速度：远处全速，近处按 sqrt 曲线收
            vt = V_APPROACH
            if d_stage < BRAKE_DIST:
                vt = max(V_MIN * 0.6, min(V_APPROACH, 0.9 * math.sqrt(2 * 1.2 * max(d_stage - 0.2, 0.05))))
            err_h = wrap(math.atan2(stage[1] - pose.pos_y, stage[0] - pose.pos_x) - heading_rad(pose.ori_z))
            steer = max(-1.0, min(1.0, -1.6 * err_h))
            thr, brk = speed_cmd(v, vt)
            if now - t0 < 1.2:
                thr = min(thr, 0.03)
            send(VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer), dt)

        # ---------------- 2) 停稳后换倒挡 ----------------
        elif phase == "stop":
            if abs(v) < 0.08:
                phase = "arc"
                thr_state = brk_state = 0.0
                print(f"已停稳(v={v:.2f})，挂倒挡")
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                time.sleep(0.4)
            else:
                send(VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.45), dt)

        # ---------------- 3) 倒车圆弧 ----------------
        elif phase == "arc":
            if dyaw < 7.5:
                phase = "straight"
                print(f"车头摆正(dyaw={dyaw:.1f}°)，回正倒直线")
            thr, brk = speed_cmd(v, V_ARC)
            send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=1.0), dt)

        # ---------------- 4) 直线倒进车位 ----------------
        elif phase == "straight":
            if abs(dx) < STOP_TOL and abs(dy) < 0.30:
                phase = "hold"
                hold_t = now
                print(f"入库：dx={dx:+.2f} dy={dy:+.2f} 航向 {dyaw:.1f}°")
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            else:
                # 目标速度随剩余深度平滑衰减
                vt = max(V_MIN, min(0.85, 1.15 * max(abs(dx) - 0.10, 0.0) + 0.22))
                err = wrap(heading_rad(slot.ori_z) - heading_rad(pose.ori_z))
                steer = max(-0.35, min(0.35, 1.0 * err))
                thr, brk = speed_cmd(v, vt)
                send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=steer), dt)

        # ---------------- 5) 驻车保持 ----------------
        else:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if now - hold_t > HOLD_SECONDS:
                print(f"完成：dx={dx:+.2f} dy={dy:+.2f} 航向 {dyaw:.1f}°  用时 {now - t0:.1f}s")
                print(f"加速度峰值（平滑后）: {peak:.2f} m/s²")
                break

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] {phase:<8} pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={v:5.2f} a={accel:+5.2f} d={d_center:5.2f} "
                  f"dx={dx:+5.2f} dy={dy:+5.2f}")

        if now - t0 > 90:
            print("超时")
            break

    print(f"峰值加速度 {peak:.2f} m/s²")
    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
