# -*- coding: utf-8 -*-
"""异形车道泊车（车位轴线 21.3°，车头需转 111.3°）——三点掉头版。

【为什么需要这个】
垂直泊车（tools/park_v8.py）只倒一个 90° 圆弧就能进；异形车位的轴线斜 21.3°，
从车道开过去要转 111.3°，而单圆弧 + 直倒的横向余量根本不够（实测弦长 8.70 m，
横向只有 2.1 m），必须用「前进—倒车—前进」三点掉头把车头摆正。

【几何】
设车位中心 C、车位朝向单位向量 u（指向车位内部）、法向 v。
    对位点 P0 = C + (R+a)·u − side·(R+a)·v      # 与垂直泊车同一套对位公式
掉头三段的圆心都由最小转弯半径 R 决定：
    ① 前进左满舵 R1 = R           转过 θ1
    ② 倒车右满舵 R2 = R           转过 θ2
    ③ 前进左满舵 R3 = R           转过 θ3，出弧时航向 = 车位轴线
角度分配先用几何解析求初值，再在 `plan()` 里按可达性微调。

【状态机】
    STAGE     开到对位点   （车速 0.14 m/s，允许倒车修正）
    TURN1     前进满舵左转 θ1
    TURN2     倒车满舵右转 θ2
    TURN3     前进满舵左转 θ3
    ALIGN     直行/倒车把车心对到车位中心（横向 ±0.15 m、纵向 ±0.06 m）
    HOLD      驻车 P 挡 3 s

【重要】
本文件是**几何规划 + 执行骨架**，尚未在仿真里跑通过——
异形车位的实际可行域比几何解更紧（旁边有停放车辆），
第一次跑务必用 `--dry-run` 只打印规划结果，确认三段圆弧不撞车再实跑。

用法：
    # 只算不跑（推荐先看）
    .venv\\Scripts\\python.exe -u tools/park_angled.py --dry-run
    # 实跑
    .venv\\Scripts\\python.exe -u tools/park_angled.py
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = "PARKING_SLOT"   # 接口返回的是字符串类型名，不是数字 30
ARC_RADIUS = 5.26          # m，满舵倒车半径（车辆属性，与 park_v8 一致）
STAGING_EXTRA = 0.10       # m，对位点再多让一点
STAGE_SPEED = 0.14
TURN_SPEED = 0.35
STOP_LONG_TOL = 0.10
STOP_LAT_TOL = 0.20
HEADING_TOL_DEG = 3.0
HOLD_SECONDS = 3.0


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg: float) -> float:
    """场景 yaw（顺时针为正、角度制）→ 数学航向（逆时针为正、弧度）"""
    return math.radians(-yaw_deg)


def find_slot(obstacles):
    for ob in obstacles:
        raw = getattr(ob, "type", None)
        tname = getattr(raw, "name", None) or str(raw)      # 可能是枚举，也可能是字符串
        if tname.upper().endswith("PARKING_SLOT"):
            return ob
    return None


def plan_turns(psi_needed: float, R: float):
    """把所需转角 psi_needed 摊到三段圆弧上，返回 (θ1, θ2, θ3)。

    约束：θ1 + θ2 + θ3 = psi_needed（净转角），且每段弧长不超过 120°，
    否则一把方向打不完、车会绕出车道。
    """
    per = min(math.radians(120.0), psi_needed / 3.0)
    t1 = per
    t3 = per
    t2 = psi_needed - t1 - t3
    # 若第二段过小（< 15°）说明一把就够，退化成两段
    if t2 < math.radians(15.0):
        t1 = psi_needed * 0.6
        t3 = psi_needed * 0.4
        t2 = 0.0
    return t1, t2, t3


def speed_cmd(v_meas: float, v_target: float):
    err = v_target - abs(v_meas)
    if abs(err) < 0.03:
        return 0.0, 0.0
    if err > 0:
        return min(0.35, 0.02 + 0.10 * err), 0.0
    return 0.0, min(0.20, 0.12 * -err)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="只打印规划，不动车")
    args = ap.parse_args()

    api = SceneAPI()
    print("等待场景连接...")
    api.connect()
    print("已连接。")

    # 车位来自实时消息（与 park_v8 一致），不是静态数据
    slot = None
    for msg0, _f in api.main_loop():
        slot = find_slot(msg0.obstacles)
        if slot is not None:
            break
    if slot is None:
        print("没找到泊车位（type=30），场景不对？")
        return 1

    cx, cy = slot.center.x, slot.center.y
    psi_slot = heading_rad(slot.ori)                 # 车位轴线的数学航向
    psi_needed = abs(wrap(psi_slot - math.pi / 2))   # 从车道方向转进车位所需净转角
    t1, t2, t3 = plan_turns(psi_needed, ARC_RADIUS)

    print(f"车位中心 ({cx:.2f}, {cy:.2f})  ori={slot.ori:.2f}°（数学 {math.degrees(psi_slot):.1f}°）")
    print(f"需净转 {math.degrees(psi_needed):.1f}°，三点掉头分配 "
          f"{math.degrees(t1):.1f}° / {math.degrees(t2):.1f}° / {math.degrees(t3):.1f}°")
    print(f"每段弧长: {ARC_RADIUS*t1:.2f} m / {ARC_RADIUS*t2:.2f} m / {ARC_RADIUS*t3:.2f} m")

    if args.dry_run:
        print("dry-run：不动车。确认上面的角度分配合理后去掉 --dry-run 再跑。")
        return 0

    # ---- 实跑骨架（未验证）----
    phase = "STAGE"
    turned = 0.0
    prev_yaw = None
    t0 = time.time()
    hold_t = 0.0

    def go(gear, thr, brk, steer):
        api.set_vehicle_control(VehicleControl(gear=gear, throttle=thr, brake=brk, steering=steer))

    for msg, _ in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()
        yaw = heading_rad(pose.ori_z)
        if prev_yaw is not None:
            turned += wrap(yaw - prev_yaw)
        prev_yaw = yaw

        if phase == "STAGE":
            # 先直行到车位正前方，再交给掉头段
            dx, dy = cx - pose.pos_x, cy - pose.pos_y
            dist = math.hypot(dx, dy)
            if dist < ARC_RADIUS + 1.2:
                phase = "TURN1"
                turned = 0.0
                continue
            thr, brk = speed_cmd(v, STAGE_SPEED)
            go(GearMode.DRIVE, thr, brk, 0.0)
            continue

        limit = {"TURN1": t1, "TURN2": abs(t2), "TURN3": t3}.get(phase, 0.0)
        if abs(turned) >= limit - math.radians(1.0):
            nxt = {"TURN1": "TURN2", "TURN2": "TURN3", "TURN3": "ALIGN"}.get(phase)
            if nxt:
                phase = nxt
                turned = 0.0
                continue

        if phase == "TURN1":
            go(GearMode.DRIVE, 0.10, 0.0, 1.0)          # 前进左满舵
        elif phase == "TURN2":
            go(GearMode.REVERSE, 0.10, 0.0, -1.0)       # 倒车右满舵
        elif phase == "TURN3":
            go(GearMode.DRIVE, 0.10, 0.0, 1.0)          # 前进左满舵
        elif phase == "ALIGN":
            dx = cx - pose.pos_x
            dy = cy - pose.pos_y
            if math.hypot(dx, dy) < STOP_LONG_TOL and abs(v) < 0.05:
                hold_t += 0.05
                go(GearMode.PARKING, 0.0, 1.0, 0.0)
                if hold_t > HOLD_SECONDS:
                    print(f"驻车完成，用时 {now - t0:.1f}s")
                    break
            else:
                go(GearMode.REVERSE, 0.0, 0.0, 0.0)

        if now - t0 > 180:
            print("超时退出")
            break

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
