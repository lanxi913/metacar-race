# -*- coding: utf-8 -*-
"""异形泊车 第 1 步：换到左车道 → 往前开 → 停到待泊位置（本轮只做接近，不倒车）。

按你的要求：
  1. 先换到左侧车道
  2. 沿车道往前开
  3. 检测到车位后，停在车位前方 6 m 的位置（车心对车位中心）
  4. 停稳即结束，倒车动作下一轮再加

控制约束：只用油门调速，刹车只在最后停车时轻踩并限斜率，避免减速度超限。

探针实测：车位 (-8.48, -2.04) ori 21.3°；自车起点 (-6.35, 30) yaw 90
yaw 90° → 数学 -90°（车头朝 -y），此时车的左侧是 +x。
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

LANE_SHIFT = 3.20        # 换一条车道（米）
STOP_AHEAD = 6.0         # 停在车位中心前方 6 m
CRUISE = 0.6             # m/s，慢点无所谓
THR_STEP = 0.02          # 每帧油门最大变化
BRK_STEP = 0.02          # 每帧刹车最大变化（限斜率，防减速度超限）


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    return math.radians(-yaw_deg)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def find_slot(obstacles):
    for ob in obstacles or []:
        t = getattr(ob, "type", None)
        name = getattr(t, "name", None) or str(t)
        if "PARK" in name.upper():
            return (ob.pos_x, ob.pos_y, ob.ori_z)
    return None


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...")
    api.connect()
    print("已连接。")

    lim = {"thr": 0.0, "brk": 0.0}

    def send(gear, thr, brk, steer):
        lim["thr"] += clamp(thr - lim["thr"], -THR_STEP, THR_STEP)
        lim["brk"] += clamp(brk - lim["brk"], -BRK_STEP, BRK_STEP)
        lim["thr"] = clamp(lim["thr"], 0.0, 1.0)
        lim["brk"] = clamp(lim["brk"], 0.0, 1.0)
        api.set_vehicle_control(VehicleControl(gear=gear, throttle=lim["thr"],
                                               brake=lim["brk"], steering=steer))

    slot = None
    cx = cy = 0.0
    x_lane_left = None
    phase = "LANE_CHANGE"
    t0 = time.time()
    last = 0.0

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        yaw = heading_rad(pose.ori_z)
        now = time.time()

        if slot is None:
            s = find_slot(msg.obstacles)
            if s is None:
                send(GearMode.PARKING, 0.0, 0.2, 0.0)
                continue
            cx, cy, ori = s
            slot = (cx, cy, ori)
            # 车在车位右侧车道，左侧车道 = 当前 x 再加一个车道宽（车头朝 -y 时左侧是 +x）
            x_lane_left = pose.pos_x + LANE_SHIFT
            print(f"车位 ({cx:.2f}, {cy:.2f}) ori={ori:.1f}°  自车 x={pose.pos_x:.2f}")
            print(f"目标：换到左车道 x≈{x_lane_left:.2f}，再前进到 y≈{cy + STOP_AHEAD:.2f}")

        if phase == "LANE_CHANGE":
            # 车头朝 -y，横向误差 = 目标x - 当前x，左侧为正
            ex = x_lane_left - pose.pos_x
            if abs(ex) < 0.25:
                phase = "FORWARD"
                print(f"[{now - t0:5.1f}s] 已换到左车道 x={pose.pos_x:.2f}，开始前进")
            else:
                # 前进 + 左打方向（正方向需要试，这里用 -sign 与前面验证过的纯跟踪一致）
                steer = clamp(-ex * 0.6, -0.5, 0.5)
                thr = 0.10 if v < CRUISE else 0.0
                send(GearMode.DRIVE, thr, 0.0, steer)
                if now - last > 1.0:
                    last = now
                    print(f"[{now - t0:5.1f}s] LANE_CHANGE x={pose.pos_x:.2f} err={ex:+.2f} v={v:.2f} steer={steer:+.2f}")
            continue

        if phase == "FORWARD":
            ey = (cy + STOP_AHEAD) - pose.pos_y
            # 车头朝 -y：y 越小越靠前，ey>0 表示还没到
            if ey < 0.3:
                phase = "STOP"
                print(f"[{now - t0:5.1f}s] 到达待泊位置 y={pose.pos_y:.2f}，开始缓刹")
            else:
                thr = 0.10 if v < CRUISE else 0.0
                send(GearMode.DRIVE, thr, 0.0, 0.0)
                if now - last > 1.0:
                    last = now
                    print(f"[{now - t0:5.1f}s] FORWARD y={pose.pos_y:.2f} 距目标 {ey:.1f} m v={v:.2f}")
            continue

        if phase == "STOP":
            if abs(v) < 0.05:
                send(GearMode.PARKING, 0.0, 0.35, 0.0)
                if now - last > 1.0:
                    last = now
                    print(f"[{now - t0:5.1f}s] 已停稳 pos=({pose.pos_x:.2f},{pose.pos_y:.2f}) "
                          f"yaw={pose.ori_z:.1f} 距车位中心 {math.hypot(cx-pose.pos_x, cy-pose.pos_y):.2f} m")
                if now - t0 > 2 and abs(v) < 0.05 and now - last < 0.1:
                    pass
                if time.time() - t0 > 60 and abs(v) < 0.05:
                    print("接近段完成（本轮不倒车）")
                    break
            else:
                # 只给很轻的刹车，且斜率受限
                send(GearMode.DRIVE, 0.0, 0.30, 0.0)
            continue

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
