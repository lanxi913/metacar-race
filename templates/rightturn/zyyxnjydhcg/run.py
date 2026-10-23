# -*- coding: utf-8 -*-
r"""直角转弯 · 基线模板（目标 110 分 = 任务 100 + 加速度 ≤5 m/s² 的 10）

实测结论（2026-10-23，三次 110 分，无压线）：
    前视距离必须收到 3.0 m。车辆停止题那套 6 m 前视放在 3 m 曲率的直角弯上
    会切弯、压到边线（−10/−20）。
    油门 0.01（约 4.78 m/s），加速度分仍然有效。
    终点前不要主动驻车：主动停会让平台迟迟不触发完成判定，继续保持前进。

用法（顺序不能反）：
    1) .venv\Scripts\python.exe -u templates\rightturn\zyyxnjydhcg\run.py
    2) 客户端点「启动场景」（直角转弯）
    3) 到终点后平台自动结束

参数：
    --throttle   定油门，默认 0.01
    --lookahead  前视距离（m），默认 3.0（直角弯的关键参数）
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.config import (  # noqa: E402
    TURN_LOOKAHEAD_M,
    TURN_THROTTLE_COMMAND,
)
from metacar_race.control.tracking import PurePursuit  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="直角转弯基线模板")
    parser.add_argument("--throttle", type=float, default=TURN_THROTTLE_COMMAND)
    parser.add_argument("--lookahead", type=float, default=TURN_LOOKAHEAD_M)
    parser.add_argument("--max-seconds", type=float, default=100.0)
    args = parser.parse_args()

    if args.lookahead > 4.0:
        print(f"提醒：前视 {args.lookahead} m 偏大，3 m 曲率的弯容易切内线。")

    print("连接仿真场景（先跑脚本，再点启动场景，这里会等）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    route = list(static.route)
    if len(route) < 2:
        print("route 点不足，无法循迹。", flush=True)
        return 1
    pursuit = PurePursuit(route, lookahead=args.lookahead)
    endpoint = static.sub_scenes[-1].end_point if static.sub_scenes else None
    print(
        f"子场景: {[s.name for s in static.sub_scenes]}  route={len(route)}  "
        f"lookahead={args.lookahead:.1f}  throttle={args.throttle:.3f}",
        flush=True,
    )

    frames = 0
    try:
        for msg, _camera_frames in api.main_loop():
            frames += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            end_point = status.end_point or endpoint

            steering = pursuit.steering(
                float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
            )
            # 距终点 1.5 m 内回正，避免冲线时还在打方向
            if end_point is not None:
                left = math.hypot(
                    float(end_point.x) - float(pose.pos_x),
                    float(end_point.y) - float(pose.pos_y),
                )
                if left <= 1.5:
                    steering = 0.0

            api.set_vehicle_control(
                VehicleControl(
                    gear=GearMode.DRIVE,
                    throttle=args.throttle,
                    brake=0.0,
                    steering=float(steering),
                )
            )

            if frames % 30 == 1:
                print(
                    f"[{status.used_time:6.1f}s] "
                    f"pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f}) "
                    f"yaw={float(pose.ori_z):7.2f} v={float(car.speed):5.2f} "
                    f"st={float(steering):+5.2f}",
                    flush=True,
                )

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
