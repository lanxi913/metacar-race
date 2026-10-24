# -*- coding: utf-8 -*-
r"""只读场景探查入口，用于泊车等需要先读取几何的任务。

运行：
    .venv\Scripts\python.exe -u tools\probe_scene.py --seconds 3
    → 客户端启动目标场景

脚本会持续发送驻车指令，不移动车辆；静态数据和首帧障碍物写入 logs/。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from metacar import GearMode, SceneAPI, VehicleControl

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def vector_data(point) -> dict[str, float]:
    return {
        "x": float(point.x),
        "y": float(point.y),
        "z": float(point.z),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="只读场景探查")
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    static_data = static.model_dump(mode="json", by_alias=True)
    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    static_path = log_dir / f"probe_static_{stamp}.json"
    first_frame_path = log_dir / f"probe_frame_{stamp}.json"
    static_path.write_text(
        json.dumps(static_data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print(
        f"子场景: {[s.name for s in static.sub_scenes]}  "
        f"route={len(static.route)} roads={len(static.roads)}  "
        f"VLA={static.vla_extension is not None}",
        flush=True,
    )
    for sub_scene in static.sub_scenes:
        print(
            f"子场景 {sub_scene.name}: start={sub_scene.start_point} "
            f"end={sub_scene.end_point}",
            flush=True,
        )
    print(f"静态数据: {static_path}", flush=True)

    start_time = None
    frames = 0
    parking_slots = []
    try:
        for frames, (msg, camera_frames) in enumerate(api.main_loop(), start=1):
            if start_time is None:
                start_time = msg.scene_status.used_time
                first_data = {
                    "scene_status": msg.scene_status.model_dump(
                        mode="json", by_alias=True
                    ),
                    "main_vehicle": msg.main_vehicle.model_dump(
                        mode="json", by_alias=True
                    ),
                    "pose_gnss": msg.pose_gnss.model_dump(
                        mode="json", by_alias=True
                    ),
                    "obstacles": [
                        obstacle.model_dump(mode="json", by_alias=True)
                        for obstacle in msg.obstacles
                    ],
                    "trajectory": [
                        vector_data(point)
                        for point in msg.trajectory
                    ],
                    "camera_ids": [camera.id for camera in camera_frames],
                }
                first_frame_path.write_text(
                    json.dumps(first_data, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                parking_slots = [
                    obstacle
                    for obstacle in msg.obstacles
                    if obstacle.type.value == 30
                ]
                print(
                    f"障碍物={len(msg.obstacles)} "
                    f"停车位={len(parking_slots)} "
                    f"轨迹点={len(msg.trajectory)} "
                    f"相机={len(camera_frames)}",
                    flush=True,
                )
                for slot in parking_slots:
                    print(
                        f"停车位 id={slot.id} pos=({slot.pos_x:.2f},"
                        f"{slot.pos_y:.2f}) ori_z={slot.ori_z:.2f} "
                        f"size={slot.length:.2f}x{slot.width:.2f}",
                        flush=True,
                    )
                for obstacle in msg.obstacles:
                    print(
                        f"障碍物 id={obstacle.id} type={obstacle.type.name}"
                        f"({obstacle.type.value}) "
                        f"pos=({obstacle.pos_x:.2f},{obstacle.pos_y:.2f}) "
                        f"size={obstacle.length:.2f}x{obstacle.width:.2f}",
                        flush=True,
                    )
                print(f"首帧数据: {first_frame_path}", flush=True)

            api.set_vehicle_control(
                VehicleControl(gear=GearMode.PARKING, brake=1.0)
            )
            if msg.scene_status.used_time - start_time >= args.seconds:
                break
    except KeyboardInterrupt:
        print("手动中断。", flush=True)
    finally:
        try:
            api.set_vehicle_control(
                VehicleControl(gear=GearMode.PARKING, brake=1.0)
            )
        except Exception:  # noqa: BLE001
            pass

    print(
        f"探查完成：frames={frames} static={static_path} "
        f"first_frame={first_frame_path}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
