# -*- coding: utf-8 -*-
r"""垂直车道泊车入口。

运行：
    .venv\Scripts\python.exe -u tools\park_vertical.py
    → 客户端启动“垂直车道泊车”
"""
from __future__ import annotations

import csv
import json
import math
import sys
import time
from pathlib import Path

from metacar import GearMode, ObstacleType, SceneAPI, Vector3, VehicleControl

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from metacar_race.config import (
    PARK_APPROACH_DISTANCE_M,
    PARK_ALIGN_EXTENSION_M,
    PARK_LOOKAHEAD_M,
    PARK_THROTTLE_COMMAND,
    STOP_EXIT_GRACE_SECONDS,
    STOP_HOLD_SECONDS,
)
from metacar_race.control.parking import build_vertical_parking_path
from metacar_race.control.stop import StopController, hold_complete


def main() -> int:
    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)
    static = api.get_scene_static_data()

    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"park_vertical_{stamp}.csv"
    json_path = log_dir / f"park_vertical_{stamp}.json"
    csv_file = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(
        [
            "frame",
            "sim_time",
            "x",
            "y",
            "yaw",
            "speed",
            "distance_to_slot",
            "steering",
            "throttle",
            "brake",
            "stopped",
        ]
    )

    controller = None
    slot = None
    endpoint = None
    path = None
    final_x = final_y = 0.0
    final_yaw = 0.0
    final_info = None
    frame = 0
    ending = "unknown"
    previous_time = None
    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            final_x = float(pose.pos_x)
            final_y = float(pose.pos_y)
            final_yaw = float(pose.ori_z)
            if previous_time is None:
                dt = 1.0 / 30.0
            else:
                dt = max(status.used_time - previous_time, 1e-3)
            previous_time = status.used_time

            if controller is None:
                slot_candidates = [
                    obstacle
                    for obstacle in msg.obstacles
                    if obstacle.type == ObstacleType.PARKING_SLOT
                ]
                if not slot_candidates:
                    api.set_vehicle_control(
                        VehicleControl(gear=GearMode.PARKING, brake=1.0)
                    )
                    continue
                slot = min(
                    slot_candidates,
                    key=lambda obstacle: math.hypot(
                        obstacle.pos_x - final_x,
                        obstacle.pos_y - final_y,
                    ),
                )
                endpoint = Vector3(
                    x=float(slot.pos_x),
                    y=float(slot.pos_y),
                    z=float(slot.pos_z),
                )
                path = build_vertical_parking_path(
                    list(static.route),
                    slot,
                    PARK_APPROACH_DISTANCE_M,
                    PARK_ALIGN_EXTENSION_M,
                )
                controller = StopController(
                    path,
                    lookahead=PARK_LOOKAHEAD_M,
                    throttle_command=PARK_THROTTLE_COMMAND,
                    steer_lock_distance=0.0,
                )
                print(
                    f"车位 id={slot.id} center=({slot.pos_x:.2f},{slot.pos_y:.2f}) "
                    f"size={slot.length:.2f}x{slot.width:.2f} "
                    f"yaw={slot.ori_z:.2f}; 泊车路径={len(path)} 点",
                    flush=True,
                )

            vc, info = controller.control(
                final_x,
                final_y,
                final_yaw,
                float(car.speed),
                dt,
                endpoint,
            )
            final_info = info
            api.set_vehicle_control(vc)
            writer.writerow(
                [
                    frame,
                    round(status.used_time, 4),
                    round(final_x, 4),
                    round(final_y, 4),
                    round(final_yaw, 3),
                    round(float(car.speed), 4),
                    round(info.distance_to_end, 4),
                    round(vc.steering, 4),
                    round(vc.throttle, 4),
                    round(vc.brake, 4),
                    info.stopped,
                ]
            )
            if frame % 10 == 1:
                print(
                    f"[{status.used_time:6.1f}s] "
                    f"pos=({final_x:7.2f},{final_y:7.2f}) "
                    f"yaw={final_yaw:7.2f} "
                    f"v={float(car.speed):5.2f} "
                    f"d={info.distance_to_end:5.2f} "
                    f"st={vc.steering:+5.2f} "
                    f"th={vc.throttle:5.3f} br={vc.brake:4.2f} "
                    f"hold={info.held_seconds:4.1f}",
                    flush=True,
                )
            if (
                hold_complete(info)
                and info.held_seconds >= STOP_HOLD_SECONDS + STOP_EXIT_GRACE_SECONDS
            ):
                ending = "hold_complete"
                break
    except KeyboardInterrupt:
        ending = "interrupt"
    finally:
        try:
            csv_file.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            api.set_vehicle_control(
                VehicleControl(gear=GearMode.PARKING, brake=1.0)
            )
        except Exception:  # noqa: BLE001
            pass

    slot_error = None
    slot_yaw_error = None
    if slot is not None:
        slot_error = math.hypot(final_x - slot.pos_x, final_y - slot.pos_y)
        yaw_delta = abs((final_yaw - slot.ori_z) % 180.0)
        slot_yaw_error = min(yaw_delta, 180.0 - yaw_delta)
    summary = {
        "scene": [s.name for s in static.sub_scenes],
        "ending": ending,
        "stopped": bool(final_info and final_info.stopped),
        "frames": frame,
        "slot_center": (
            None if slot is None else [round(slot.pos_x, 4), round(slot.pos_y, 4)]
        ),
        "slot_yaw": None if slot is None else round(slot.ori_z, 3),
        "final_x": round(final_x, 4),
        "final_y": round(final_y, 4),
        "final_yaw": round(final_yaw, 3),
        "slot_position_error_m": (
            None if slot_error is None else round(slot_error, 4)
        ),
        "slot_yaw_error_deg": (
            None if slot_yaw_error is None else round(slot_yaw_error, 3)
        ),
        "csv": str(csv_path),
    }
    json_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
