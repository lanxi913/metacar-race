# -*- coding: utf-8 -*-
r"""垂直车道泊车：入口对齐后直线驶入。

运行：
    .venv\Scripts\python.exe -u tools\park_vertical_straight.py
    → 客户端启动“垂直车道泊车”
"""
from __future__ import annotations

import csv
import json
import math
import sys
import time
from pathlib import Path

from metacar import GearMode, ObstacleType, SceneAPI, VehicleControl

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from metacar_race.config import (
    PARK_ADJUST_THROTTLE,
    PARK_ENTRY_ALIGN_DEG,
    PARK_ENTRY_LOOKAHEAD_M,
    PARK_ENTRY_SWITCH_RADIUS_M,
    PARK_THROTTLE_COMMAND,
    STOP_EXIT_GRACE_SECONDS,
    STOP_HOLD_SECONDS,
)
from metacar_race.control.parking import (
    build_entrance_parking_paths,
    nearest_parallel_yaw,
    yaw_error,
)
from metacar_race.control.stop import StopController
from metacar_race.control.tracking import PurePursuit


def main() -> int:
    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)
    static = api.get_scene_static_data()

    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"park_straight_{stamp}.csv"
    json_path = log_dir / f"park_straight_{stamp}.json"
    csv_file = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(
        [
            "frame",
            "sim_time",
            "phase",
            "x",
            "y",
            "yaw",
            "target_yaw",
            "yaw_error",
            "speed",
            "distance_to_center",
            "steering",
            "throttle",
            "brake",
        ]
    )

    slot = None
    endpoint = None
    entry = None
    target_yaw = None
    approach = None
    entry_controller = None
    phase = "approach"
    final_x = final_y = final_yaw = 0.0
    frame = 0
    ending = "unknown"
    phase_start = 0.0
    previous_time = None
    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            sim_time = float(status.used_time)
            final_x = float(pose.pos_x)
            final_y = float(pose.pos_y)
            final_yaw = float(pose.ori_z)
            speed = float(car.speed)
            if previous_time is None:
                dt = 1.0 / 30.0
                phase_start = sim_time
            else:
                dt = max(sim_time - previous_time, 1e-3)
            previous_time = sim_time

            if slot is None:
                slots = [
                    obstacle
                    for obstacle in msg.obstacles
                    if obstacle.type == ObstacleType.PARKING_SLOT
                ]
                if not slots:
                    api.set_vehicle_control(
                        VehicleControl(gear=GearMode.PARKING, brake=1.0)
                    )
                    continue
                slot = min(
                    slots,
                    key=lambda item: math.hypot(
                        item.pos_x - final_x,
                        item.pos_y - final_y,
                    ),
                )
                approach_path, entry_path, entry = build_entrance_parking_paths(
                    list(static.route),
                    slot,
                )
                approach = PurePursuit(
                    approach_path,
                    lookahead=PARK_ENTRY_LOOKAHEAD_M,
                )
                entry_controller = StopController(
                    entry_path,
                    lookahead=1.5,
                    throttle_command=PARK_ADJUST_THROTTLE,
                    steer_lock_distance=0.05,
                )
                endpoint = entry_path[-1]
                target_yaw = nearest_parallel_yaw(
                    final_yaw,
                    float(slot.ori_z),
                )
                print(
                    f"车位中心=({slot.pos_x:.2f},{slot.pos_y:.2f}) "
                    f"入口=({entry.x:.2f},{entry.y:.2f}) "
                    f"入口航向={target_yaw:.2f} "
                    f"对齐路径={len(approach_path)} "
                    f"直线段={len(entry_path)}",
                    flush=True,
                )

            yaw_err = yaw_error(final_yaw, target_yaw)
            distance_to_entry = math.hypot(
                final_x - float(entry.x),
                final_y - float(entry.y),
            )
            distance_to_center = math.hypot(
                final_x - float(slot.pos_x),
                final_y - float(slot.pos_y),
            )

            if phase == "approach":
                steering = approach.steering(final_x, final_y, final_yaw)
                vc = VehicleControl(
                    gear=GearMode.DRIVE,
                    throttle=PARK_THROTTLE_COMMAND,
                    brake=0.0,
                    steering=steering,
                )
                if (
                    distance_to_entry <= PARK_ENTRY_SWITCH_RADIUS_M
                    and abs(yaw_err) <= PARK_ENTRY_ALIGN_DEG
                ):
                    phase = "straight_entry"
                    phase_start = sim_time
                    print(
                        f"入口对齐：distance={distance_to_entry:.3f} "
                        f"yaw_error={yaw_err:+.3f}",
                        flush=True,
                    )
            elif phase == "straight_entry":
                vc, info = entry_controller.control(
                    final_x,
                    final_y,
                    final_yaw,
                    speed,
                    dt,
                    endpoint,
                )
                if info.stopped or sim_time - phase_start >= 15.0:
                    phase = "parked"
                    phase_start = sim_time
            elif phase == "parked":
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                if (
                    sim_time - phase_start
                    >= STOP_HOLD_SECONDS + STOP_EXIT_GRACE_SECONDS
                ):
                    ending = "hold_complete"
                    break
            else:
                raise RuntimeError(f"未知阶段: {phase}")

            api.set_vehicle_control(vc)
            writer.writerow(
                [
                    frame,
                    round(sim_time, 4),
                    phase,
                    round(final_x, 4),
                    round(final_y, 4),
                    round(final_yaw, 3),
                    round(target_yaw, 3),
                    round(yaw_err, 3),
                    round(speed, 4),
                    round(distance_to_center, 4),
                    round(vc.steering, 4),
                    round(vc.throttle, 4),
                    round(vc.brake, 4),
                ]
            )
            if frame % 10 == 1:
                print(
                    f"[{sim_time:6.1f}s] {phase:13s} "
                    f"pos=({final_x:7.2f},{final_y:7.2f}) "
                    f"yaw={final_yaw:7.2f} err={yaw_err:+6.2f} "
                    f"d_entry={distance_to_entry:5.2f} "
                    f"d_center={distance_to_center:5.2f} "
                    f"st={vc.steering:+5.2f} "
                    f"th={vc.throttle:5.3f} br={vc.brake:4.2f}",
                    flush=True,
                )
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

    position_error = None
    yaw_error_deg = None
    if slot is not None:
        position_error = math.hypot(
            final_x - float(slot.pos_x),
            final_y - float(slot.pos_y),
        )
        if target_yaw is not None:
            yaw_error_deg = abs(yaw_error(final_yaw, target_yaw))
    summary = {
        "scene": [s.name for s in static.sub_scenes],
        "ending": ending,
        "phase": phase,
        "frames": frame,
        "slot_center": (
            None if slot is None else [round(slot.pos_x, 4), round(slot.pos_y, 4)]
        ),
        "slot_yaw": None if slot is None else round(slot.ori_z, 3),
        "entry": (
            None if entry is None else [round(entry.x, 4), round(entry.y, 4)]
        ),
        "target_yaw": None if target_yaw is None else round(target_yaw, 3),
        "final_x": round(final_x, 4),
        "final_y": round(final_y, 4),
        "final_yaw": round(final_yaw, 3),
        "position_error_m": (
            None if position_error is None else round(position_error, 4)
        ),
        "yaw_error_deg": (
            None if yaw_error_deg is None else round(yaw_error_deg, 3)
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
