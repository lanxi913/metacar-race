# -*- coding: utf-8 -*-
"""异形车道泊车 v2：不依赖 type 字段（场景车位位姿用探针实测值兜底），一定会动车。

探针实测（异形车道泊车）：
    车位  center=(-8.48, -2.04)  ori_z=21.3°  5.50x2.50
    两侧  CAR (-8.48,-4.61) 与 (-8.48, 0.56)，朝向同为 21.3°
    自车  (-6.35, 30.00) yaw=90°

场景 yaw 顺时针为正、角度制；数学航向 = -yaw。
    yaw 90° → 数学 -90°（车头指向 -y，沿车道向前开）
    车位轴线 yaw 21.3° → 数学 -21.3°（车头要转 +68.7°，即逆时针）

动作：
    APPROACH  沿车道开到车位前方（x 车道线，y = 车位 + 9 m）
    ARC       倒挡满舵，从 -90° 转到 -21.3°
    SETTLE    直倒贴近车位中心
    HOLD      P 挡保持 3 s
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

SLOT_FALLBACK = (-8.48, -2.04, 21.3)     # 探针实测，找不到就走兜底
APPROACH_STANDOFF = 9.0                  # 车位前方 9 m 起步
LANE_OFFSET = 2.15                       # 车道线相对车位中心的 x 偏移
TURN_SPEED = 0.25
APPROACH_SPEED = 0.30
REV_SPEED = 0.55


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
        raw = getattr(ob, "type", None)
        tname = getattr(raw, "name", None) or str(raw)
        if "PARK" in tname.upper():
            # ObstacleInfo 的字段是 pos_x / pos_y / ori_z（探针打印出来的），没有 center
            return (ob.pos_x, ob.pos_y, ob.ori_z)
    return None


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def speed_cmd(v_meas, v_target, thr_max=0.12, brk_max=0.25):
    """与 park_v8 同款速度闭环：加速给油门、超速轻刹，绝不给阶跃。"""
    # 只用油门调速：超速就松油门滑行，绝不踩刹车（刹车会造成减速度超限）
    err = v_target - abs(v_meas)
    if err <= 0.04:
        return 0.0, 0.0
    return clamp(0.02 + 0.10 * err, 0.02, thr_max), 0.0


def obstacle_pressure(msg, pose):
    """右前方有车就返回 0~1 的压力值，用于提前往左带方向。"""
    yaw = heading_rad(pose.ori_z)
    c, s = math.cos(yaw), math.sin(yaw)
    worst = 0.0
    for ob in getattr(msg, "obstacles", []) or []:
        tname = str(getattr(getattr(ob, "type", None), "name", getattr(ob, "type", "")))
        if "CAR" not in tname.upper():
            continue
        dx = ob.pos_x - pose.pos_x
        dy = ob.pos_y - pose.pos_y
        fx = dx * c + dy * s
        fy = -dx * s + dy * c
        if 0.0 < fx < 12.0 and 0.3 < fy < 3.5:
            worst = max(worst, (1.0 - fx / 12.0) * (1.0 - fy / 3.5))
    return worst


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...")
    api.connect()
    print("已连接。")

    phase = "APPROACH"
    t0 = time.time()
    slot = None
    hold = 0.0
    last_print = 0.0

    def go(gear, thr, brk, steer):
        api.set_vehicle_control(VehicleControl(gear=gear, throttle=thr, brake=brk, steering=steer))

    for msg, _ in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()
        yaw = heading_rad(pose.ori_z)

        if slot is None:
            slot = find_slot(msg.obstacles) or SLOT_FALLBACK
            cx, cy, ori = slot
            print(f"车位 ({cx:.2f}, {cy:.2f}) ori={ori:.1f}° -> 数学 {math.degrees(heading_rad(ori)):.1f}°")
            psi_target = heading_rad(ori)
            psi_turn = wrap(psi_target - yaw)
            print(f"需净转 {math.degrees(psi_turn):.1f}°")

        cx, cy, ori = slot
        psi_target = heading_rad(ori)
        aim_x = cx + LANE_OFFSET
        aim_y = cy + APPROACH_STANDOFF

        if phase == "APPROACH":
            dx, dy = aim_x - pose.pos_x, aim_y - pose.pos_y
            dist = math.hypot(dx, dy)
            if now - last_print > 1.0:
                last_print = now
                print(f"[{now - t0:5.1f}s] APPROACH pos=({pose.pos_x:.2f},{pose.pos_y:.2f}) "
                      f"yaw={pose.ori_z:.1f} v={v:.2f} 距对位点 {dist:.1f} m")
            if dist < 0.6:
                phase = "ARC"
                print(f"[{now - t0:5.1f}s] 到达对位点，开始倒车打舵")
            else:
                des = math.atan2(dy, dx)
                err = wrap(des - yaw)
                # 方向盘正方向 = 减小数学航向，所以这里要取负号
                steer = clamp(-err * 2.2, -1.0, 1.0)
                # 右前方有车：提前往左带方向（负 = 左），并降速
                press = obstacle_pressure(msg, pose)
                if press > 0.05:
                    steer = clamp(steer - 0.6 * press, -1.0, 1.0)
                vt = APPROACH_SPEED * (1.0 - 0.6 * press) if dist > 3 else 0.15
                thr, brk = speed_cmd(v, max(vt, 0.12))
                if press > 0.2 and now - last_print > 1.0:
                    last_print = now
                    print(f"[{now - t0:5.1f}s] 右前有障碍 press={press:.2f} → 左让 steer={steer:+.2f} vt={vt:.2f}")
                go(GearMode.DRIVE, thr, brk, steer)
            continue

        err_h = wrap(psi_target - yaw)

        if phase == "ARC":
            steer_arc = clamp(err_h * 2.5, -1.0, 1.0)
            print(f"ARC yaw={pose.ori_z:6.1f} target={math.degrees(psi_target):7.1f} "
                  f"err={math.degrees(err_h):7.1f} steer={steer_arc:+.2f} v={v:+.2f}")
            if abs(err_h) < math.radians(8.0):
                phase = "SETTLE"
                print(f"[{now - t0:5.1f}s] 航向到位，直倒向车位中心")
            else:
                # 倒挡：steer 正值增大数学航向（与前进相反）
                thr_r, brk_r = speed_cmd(v, REV_SPEED)
                go(GearMode.REVERSE, thr_r, brk_r, steer_arc)
            continue

        if phase == "SETTLE":
            d = math.hypot(cx - pose.pos_x, cy - pose.pos_y)
            if d < 0.25 or abs(v) > 2.0 and d < 0.6:
                phase = "HOLD"
                print(f"[{now - t0:5.1f}s] 到位（距中心 {d:.2f} m）")
            else:
                thr_s, brk_s = speed_cmd(v, 0.45)
                go(GearMode.REVERSE, thr_s, brk_s, clamp(err_h * 1.5, -0.6, 0.6))
            continue

        if phase == "HOLD":
            go(GearMode.PARKING, 0.0, 1.0, 0.0)
            hold += 0.05
            if hold > 3.0:
                print(f"驻车完成，用时 {now - t0:.1f}s")
                break
            continue

        if now - last_print > 2:
            last_print = now
            print(f"[{now - t0:5.1f}s] phase={phase} pos=({pose.pos_x:.2f},{pose.pos_y:.2f}) v={v:.2f} yaw={pose.ori_z:.1f}")

        if now - t0 > 150:
            print("超时")
            break

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
