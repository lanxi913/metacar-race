# -*- coding: utf-8 -*-
r"""车辆停止 · 基线模板（目标 110 分 = 任务 100 + 加速度 ≤5 m/s² 的 10）

题目要求：跑到终点后停稳（停稳后保持 5 s），不能超时。
实测结论（2026-10-23，三次 110 分）：
    油门 0.01 稳态约 4.78 m/s，0.5 s 窗口最大加速度 ≈4.19 m/s²（<5，保分）
    刹车 0.10 从 4.78 m/s 刹停约 4.22 m，等效减速度 2.70 m/s²
    油门 0.02 起步会出现 11 m/s² 尖峰；刹车 0.50 会出现 19 m/s² 尖峰 —— 都不能用

实现要点（都写在 metacar_race/control/stop.py 里）：
    1) 用「当前速度算出的刹车距离」决定何时开始一次连续制动，
       不再逐帧按抖动的 car.speed 在油门/刹车之间来回切（会抖出加速度尖峰）。
    2) 速度 <0.30 m/s 持续 0.80 s 且距终点 <0.40 m 才锁存驻车。

用法（顺序不能反）：
    1) .venv\Scripts\python.exe -u templates\stop\zyyxnjydhcg\run.py
    2) 客户端点「启动场景」（车辆停止）
    3) 停稳后自动保持制动，跑完退出

参数：
    --throttle   巡航油门，默认 0.01（改大就会丢加速度分）
    --brake      制动指令，默认 0.10
    --hold       停稳后保持秒数，默认 5.0
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.config import (  # noqa: E402
    STOP_BRAKE_COMMAND,
    STOP_HOLD_SECONDS,
    STOP_THROTTLE_COMMAND,
)
from metacar_race.control.stop import StopController, hold_complete  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="车辆停止基线模板")
    parser.add_argument("--throttle", type=float, default=STOP_THROTTLE_COMMAND)
    parser.add_argument("--brake", type=float, default=STOP_BRAKE_COMMAND)
    parser.add_argument("--hold", type=float, default=STOP_HOLD_SECONDS)
    parser.add_argument("--max-seconds", type=float, default=100.0)
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景，这里会等）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    route = list(static.route)
    if len(route) < 2:
        print("route 点不足，无法循迹。", flush=True)
        return 1
    controller = StopController(route)
    endpoint = static.sub_scenes[-1].end_point if static.sub_scenes else None
    print(
        f"子场景: {[s.name for s in static.sub_scenes]}  终点={endpoint}  "
        f"油门={args.throttle}  刹车={args.brake}",
        flush=True,
    )

    previous_time = None
    frames = 0
    try:
        for msg, _camera_frames in api.main_loop():
            frames += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            end_point = status.end_point or endpoint

            dt = 1.0 / 30.0 if previous_time is None else max(status.used_time - previous_time, 1e-3)
            previous_time = status.used_time

            vc, info = controller.control(
                float(pose.pos_x),
                float(pose.pos_y),
                float(pose.ori_z),
                float(car.speed),
                dt,
                end_point,
            )
            # 模板允许调参：油门/刹车在标定值附近微调
            vc.throttle = min(vc.throttle, args.throttle) if vc.throttle > 0 else vc.throttle
            vc.brake = args.brake if vc.brake > 0 else vc.brake
            api.set_vehicle_control(vc)

            if frames % 30 == 1:
                print(
                    f"[{status.used_time:6.1f}s] v={float(car.speed):5.2f} "
                    f"dist={info.distance_to_end:6.2f} along={info.along_track:6.2f} "
                    f"stopped={info.stopped} hold={info.held_seconds:4.1f}",
                    flush=True,
                )

            if info.stopped and info.held_seconds >= args.hold:
                print(f"停稳 {info.held_seconds:.1f}s，完成。", flush=True)
                break
            if status.used_time >= args.max_seconds:
                print("到达时限，退出。", flush=True)
                break
    except KeyboardInterrupt:
        print("手动中断。", flush=True)
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass

    print(f"结束，共 {frames} 帧。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
