# -*- coding: utf-8 -*-
r"""「平行车道泊车」入口：现场规划 + 入位 + 停车保持 5 s。

用法（顺序不能反）：
    1) .venv\Scripts\python.exe -u parking\main.py
    2) 客户端点「启动场景」（车辆直行那一行的正下方：平行车道泊车）
    3) 终端会打印规划结果与每帧状态，跑完给出 SUMMARY

要点：
  * 车位与障碍物全部从场景读取，不写死坐标（AGENTS.md 第 4 条）
  * 路径用 Hybrid A* 现场搜索：直行接近 → 斜插进车位 → 倒车回正
  * 入位后刹停并保持 6 s（评分要求 5 s）
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from metacar import GearMode, ObstacleType, SceneAPI, VehicleControl

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from parking.controller import ParkingDriver          # noqa: E402
from parking.planner import Planner, side_line_from_slot, slot_goal  # noqa: E402

EGO_L, EGO_W = 4.814, 2.18


def main() -> int:
    ap = argparse.ArgumentParser(description="平行车道泊车")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--goal-tol", type=float, default=0.22)
    args = ap.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    st = api.get_scene_static_data()
    print(f"路线点: {len(st.route)}  子场景: {[s.name for s in st.sub_scenes]}", flush=True)

    driver = None
    planner = None
    slot = None
    t0 = time.time()
    frames = 0
    last = None

    for msg, cams in api.main_loop():
        frames += 1
        car = msg.main_vehicle
        pose = msg.pose_gnss
        status = msg.scene_status

        if driver is None:
            # ---------- 首帧：读车位与障碍物，现场规划 ----------
            slots = [o for o in msg.obstacles if o.type == ObstacleType.PARKING_SLOT]
            cars = [o for o in msg.obstacles if o.type in (ObstacleType.CAR, ObstacleType.TRUCK, ObstacleType.BUS)]
            if not slots:
                print("没找到 PARKING_SLOT，保持刹车。", flush=True)
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                continue

            slot = slots[0]
            print(f"车位: ({slot.pos_x:.2f},{slot.pos_y:.2f}) {slot.length:.1f}x{slot.width:.1f} ori={slot.ori_z:.1f}", flush=True)
            for c in cars:
                print(f"  已停车: ({c.pos_x:.2f},{c.pos_y:.2f}) {c.length:.1f}x{c.width:.1f} ori={c.ori_z:.1f}", flush=True)

            def rect_of(o):
                th = math.radians(-o.ori_z)
                ca, sa = math.cos(th), math.sin(th)
                hl, hw = o.length / 2 + 0.04, o.width / 2 + 0.04
                return [(o.pos_x + dx * ca - dy * sa, o.pos_y + dx * sa + dy * ca)
                        for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]

            planner = Planner(EGO_L, EGO_W, 2.6, 0.6,
                              obstacles=[rect_of(c) for c in cars],
                              side_line_x=side_line_from_slot(slot))

            lane_x = pose.pos_x
            handoff_y = slot.pos_y + 3.8
            start = (lane_x, handoff_y, -math.pi / 2)
            goal = slot_goal(slot)
            print(f"规划: start=({start[0]:.2f},{start[1]:.2f}) goal=({goal[0]:.2f},{goal[1]:.2f})", flush=True)
            tp = time.time()
            path = planner.plan(start, goal, goal_tol=args.goal_tol)
            print(f"规划耗时 {time.time() - tp:.1f}s  路径点 {0 if path is None else len(path)}", flush=True)

            if path is None:
                print("规划失败，保持刹车。", flush=True)
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                continue

            fwd = sum(1 for p in path if p.d > 0)
            print(f"  前进 {fwd} 段 / 倒车 {len(path) - fwd} 段", flush=True)
            print(f"  终点 ({path[-1].x:.2f},{path[-1].y:.2f}) yaw={math.degrees(-path[-1].th):.1f}", flush=True)

            driver = ParkingDriver(path, lane_x=lane_x, handoff_y=handoff_y,
                                   goal_xy=(goal[0], goal[1]), goal_theta=goal[2])
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            continue

        dt = 1.0 / 30.0
        if last is not None:
            dt = max(status.used_time - last, 1e-3)
        last = status.used_time

        vc, state = driver.step(pose.pos_x, pose.pos_y, pose.ori_z, car.speed, dt)
        api.set_vehicle_control(vc)

        if frames % 10 == 1:
            gx, gy = driver.goal_xy
            print(
                f"[{status.used_time:6.1f}s] {state:5s} "
                f"pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) yaw={pose.ori_z:6.1f} "
                f"v={car.speed:5.2f} thr={vc.throttle:4.2f} brk={vc.brake:4.2f} "
                f"str={vc.steering:+5.2f} gear={vc.gear.name:7s} "
                f"d=({gx - pose.pos_x:+5.2f},{gy - pose.pos_y:+5.2f})",
                flush=True,
            )

        if state == "done":
            break
        if time.time() - t0 > args.seconds:
            print("到达运行时长，退出。", flush=True)
            break

    summary = {
        "frames": frames,
        "state": None if driver is None else driver.state,
        "notes": [] if driver is None else driver.notes,
        "slot": None if slot is None else [round(slot.pos_x, 2), round(slot.pos_y, 2)],
        "elapsed": round(time.time() - t0, 1),
    }
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    (log_dir / f"parking_{time.strftime('%Y%m%d-%H%M%S')}.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
