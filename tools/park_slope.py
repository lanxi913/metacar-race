# -*- coding: utf-8 -*-
"""斜坡泊车：先用强油门 + 纯跟踪爬上坡道到 staging，再跑两段圆弧侧方停车。

与 parallel_parking.py 的差异：
  1. APPROACH 用 PI 速度环（油门上限 0.35，不升挡），沿 static.route 纯跟踪上坡；
  2. 全程打印 pose.z / ori_x / ori_y，便于判断坡度影响；
  3. staging 判定不变（远端车 + 0.15 m）。
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

SLOT_TYPE = 30
WHEELBASE = 2.60
MAX_STEER = 0.60
STEER = 0.75
CAR_LEN = 4.81
STAGING_MARGIN = 0.15
V_APPROACH = 3.0
V_REVERSE = 0.9
BRAKE_CMD = 0.10
SETTLE_BRAKE = 0.06
HOLD_S = 5.0
TOL_DX = 0.35
TOL_DY = 0.35


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def turning_radius(steer):
    s = max(0.05, min(1.0, abs(steer)))
    return WHEELBASE / math.tan(MAX_STEER * s)


def main() -> int:
    print("waiting for scene ...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    route = [(float(p.x), float(p.y)) for p in static.route]
    print("connected; route pts", len(route), "sub", [s.name for s in static.sub_scenes], flush=True)

    slot = None
    cars = []
    idx = 0
    phase = "APPROACH"
    phase_t = 0.0
    hold = 0.0
    integ = 0.0
    last_t = None
    done = False
    theta = 0.0
    stage_y = 0.0
    v_peak = 0.0

    try:
        for msg, _ in api.main_loop():
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
            z = float(getattr(pose, "pos_z", 0.0) or 0.0)
            speed = float(car.speed)
            dt = 1 / 30 if last_t is None else max(st.used_time - last_t, 1e-3)
            last_t = st.used_time
            v_peak = max(v_peak, abs(speed))

            if slot is None:
                for o in msg.obstacles:
                    tv = getattr(getattr(o, "type", None), "value", None)
                    tn = getattr(getattr(o, "type", None), "name", "")
                    if tv == SLOT_TYPE:
                        slot = (float(o.pos_x), float(o.pos_y), float(o.ori_z), float(o.length), float(o.width))
                    elif tn == "CAR":
                        cars.append((float(o.pos_x), float(o.pos_y), float(o.length)))
                if slot:
                    sx, sy, sori, slen, swid = slot
                    lateral = abs(sx - x)
                    R = turning_radius(STEER)
                    theta = math.degrees(math.acos(max(-0.99, min(0.99, 1.0 - lateral / (2 * R)))))
                    far = [c for c in cars if abs(c[0] - sx) < 1.5 and c[1] < sy]
                    stage_y = (far[0][1] + STAGING_MARGIN) if far else (sy - slen / 2 - CAR_LEN / 2 - 1.0)
                    print(f"GEOM slot=({sx:.2f},{sy:.2f}) ori={sori:.1f} {slen:.2f}x{swid:.2f} "
                          f"lateral={lateral:.2f} R={R:.2f} theta={theta:.1f} stage_y={stage_y:.2f}", flush=True)

            steer = thr = brk = 0.0
            gear = GearMode.DRIVE

            if phase == "APPROACH":
                rem = y - stage_y
                # pure pursuit on route
                best, bd = idx, 1e9
                for j in range(idx, min(len(route), idx + 40)):
                    d = math.hypot(route[j][0] - x, route[j][1] - y)
                    if d < bd:
                        best, bd = j, d
                idx = max(idx, best)
                tx, ty = route[min(len(route) - 1, idx + 6)][0], route[min(len(route) - 1, idx + 6)][1]
                desired = -math.degrees(math.atan2(ty - y, tx - x))
                err_yaw = math.radians(wrap180(desired - yaw))
                steer = max(-1.0, min(1.0, 2.0 * err_yaw))
                v_des = min(V_APPROACH, math.sqrt(max(2 * 0.8 * max(rem, 0.0), 0.0)))
                err = v_des - speed
                integ = max(-3.0, min(20.0, integ + err * dt))
                u = 0.12 * err + 0.05 * integ
                if u > 0:
                    thr, brk = min(0.35, u), 0.0
                else:
                    thr, brk = 0.0, min(1.0, -u * 1.2)
                # 爬到坡顶就改直行，不要跟着 route 拐进车位区
                if y < 5.0 and z > 1.5:
                    phase, phase_t = "STRAIGHT", 0.0
                    print("== top of ramp, switch to STRAIGHT ==", flush=True)
            elif phase == "STRAIGHT":
                rem = y - stage_y
                desired = 90.0
                err_yaw = math.radians(wrap180(desired - yaw))
                steer = max(-1.0, min(1.0, 2.5 * err_yaw))
                v_des = min(2.5, math.sqrt(max(2 * 0.6 * max(rem, 0.0), 0.0)))
                err = v_des - speed
                if err < -0.25:
                    thr, brk = 0.0, BRAKE_CMD
                else:
                    thr, brk = min(0.20, max(0.0, 0.03 + 0.25 * err)), 0.0
                if rem <= 0.12:
                    phase, phase_t = "STOP", 0.0
            elif phase == "STOP":
                gear, brk = GearMode.DRIVE, BRAKE_CMD
                if abs(speed) <= 0.08:
                    phase, phase_t = "PRE", 0.0
            elif phase == "PRE":
                # 换向之前先把车彻底摁住 0.4 s，避免 DRIVE->REVERSE 插值造成加速度尖峰
                gear, brk = GearMode.PARKING, BRAKE_CMD
                if phase_t >= 0.4:
                    phase, phase_t = "ARC1", 0.0
            elif phase == "ARC1":
                gear = GearMode.REVERSE
                k = min(1.0, phase_t / 1.2)
                steer = STEER * k
                # 坡道倒车是下坡，开环油门会被重力带到 3 m/s 以上，必须闭环控速
                if abs(speed) > 1.05:
                    thr, brk = 0.0, 0.14
                elif abs(speed) > 0.85:
                    thr, brk = 0.0, 0.0
                elif abs(speed) < 0.45:
                    thr, brk = 0.030, 0.0
                else:
                    thr, brk = 0.010, 0.0
                if 90.0 - yaw >= theta:
                    phase, phase_t = "ARC2", 0.0
            elif phase == "ARC2":
                gear = GearMode.REVERSE
                steer = -STEER * min(1.0, phase_t / 0.8)
                if abs(speed) > 1.05:
                    thr, brk = 0.0, 0.14
                elif abs(speed) > 0.85:
                    thr, brk = 0.0, 0.0
                elif abs(speed) < 0.45:
                    thr, brk = 0.030, 0.0
                else:
                    thr, brk = 0.010, 0.0
                yaw_err = min(abs(wrap180(yaw - slot[2])), abs(wrap180(yaw - slot[2] - 180.0)))
                if yaw_err <= 3.0:
                    phase, phase_t = "FIT", 0.0
            elif phase == "FIT":
                dx, dy = x - slot[0], y - slot[1]
                if abs(dy) <= TOL_DY and abs(dx) <= TOL_DX:
                    phase, phase_t = "SETTLE", 0.0
                elif abs(speed) > 0.35:
                    brk = 0.10
                else:
                    # 上坡方向（dy>0 要往回走 -y）油门给足，否则会被坡度顶住
                    gear = GearMode.DRIVE if dy > 0 else GearMode.REVERSE
                    thr = 0.060 if abs(speed) < 0.30 else 0.0
                    if abs(speed) > 0.45:
                        thr, brk = 0.0, 0.10
            elif phase == "SETTLE":
                if abs(speed) > 0.06:
                    gear, brk = GearMode.DRIVE, SETTLE_BRAKE
                else:
                    gear, brk = GearMode.PARKING, BRAKE_CMD
                    hold += dt
                    if hold >= HOLD_S:
                        done = True
            if done:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                print(f"DONE final pos=({x:.2f},{y:.2f}) yaw={yaw:.2f} slot=({slot[0]:.2f},{slot[1]:.2f}) "
                      f"dx={x - slot[0]:+.2f} dy={y - slot[1]:+.2f}", flush=True)
                break

            api.set_vehicle_control(VehicleControl(gear=gear, throttle=float(thr), brake=float(brk), steering=float(steer)))
            phase_t += dt

            if int(st.used_time * 2) != int((st.used_time - dt) * 2):
                dxs = x - slot[0] if slot else 0.0
                dys = y - slot[1] if slot else 0.0
                print(f"[{st.used_time:5.1f}s] {phase:8s} pos=({x:7.2f},{y:7.2f},z={z:5.2f}) yaw={yaw:6.1f} "
                      f"v={speed:5.2f} thr={thr:4.2f} brk={brk:4.2f} st={steer:+4.2f} dx={dxs:+5.2f} dy={dys:+5.2f}", flush=True)

            if st.used_time > 190:
                print("timeout", flush=True)
                break
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:
            pass
    print(f"RESULT phase={phase} v_peak={v_peak:.2f}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
