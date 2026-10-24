# -*- coding: utf-8 -*-
r"""垂直车道泊车：前进入位、倒车调姿、前进回正。

运行顺序：
    .venv\Scripts\python.exe -u tools\park_vertical_adjust.py
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
    PARK_ADJUST_THROTTLE,
    PARK_ALIGN_EXTENSION_M,
    PARK_ALIGN_TIMEOUT_SECONDS,
    PARK_ALIGN_TOLERANCE_DEG,
    PARK_APPROACH_OFFSET_M,
    PARK_APPROACH_DISTANCE_M,
    PARK_ENTRY_RADIUS_M,
    PARK_FINAL_ALIGN_DEG,
    PARK_LOOKAHEAD_M,
    PARK_MAX_ADJUST_CYCLES,
    PARK_PAUSE_SECONDS,
    PARK_POSITION_TOLERANCE_M,
    PARK_RECOVERY_CONTROL_M,
    PARK_RECOVERY_LOOKAHEAD_M,
    PARK_REVERSE_DISTANCE_M,
    PARK_REVERSE_THROTTLE,
    PARK_THROTTLE_COMMAND,
    STOP_EXIT_GRACE_SECONDS,
    STOP_HOLD_SECONDS,
)
from metacar_race.control.parking import (
    build_alignment_path,
    build_vertical_parking_path,
    nearest_parallel_yaw,
    slot_travel_direction,
    yaw_error,
)
from metacar_race.control.stop import StopController
from metacar_race.control.tracking import PurePursuit


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def main() -> int:
    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)
    static = api.get_scene_static_data()

    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"park_adjust_{stamp}.csv"
    json_path = log_dir / f"park_adjust_{stamp}.json"
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
            "distance_to_slot",
            "steering",
            "throttle",
            "brake",
        ]
    )

    controller = None
    slot = None
    endpoint = None
    approach_endpoint = None
    phase = "approach"
    phase_start = 0.0
    target_yaw = None
    reverse_reference = None
    recovery_pursuit = None
    final_x = final_y = 0.0
    final_yaw = 0.0
    frame = 0
    ending = "unknown"
    previous_time = None
    adjust_cycles = 0

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
                direction_x, direction_y = slot_travel_direction(
                    final_x,
                    final_y,
                    slot,
                )
                approach_endpoint = Vector3(
                    x=float(slot.pos_x) - direction_x * PARK_APPROACH_OFFSET_M,
                    y=float(slot.pos_y) - direction_y * PARK_APPROACH_OFFSET_M,
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

            position = (final_x, final_y)
            distance_to_slot = math.hypot(
                final_x - float(slot.pos_x),
                final_y - float(slot.pos_y),
            )
            current_yaw_error = (
                None
                if target_yaw is None
                else yaw_error(final_yaw, target_yaw)
            )

            if phase == "approach":
                vc, info = controller.control(
                    final_x,
                    final_y,
                    final_yaw,
                    speed,
                    dt,
                    approach_endpoint,
                )
                if (
                    info.stopped
                    or (
                        speed <= 0.20
                        and math.hypot(
                            final_x - float(approach_endpoint.x),
                            final_y - float(approach_endpoint.y),
                        )
                        <= PARK_ENTRY_RADIUS_M
                    )
                ):
                    target_yaw = nearest_parallel_yaw(
                        final_yaw, float(slot.ori_z)
                    )
                    phase = "reverse_settle"
                    phase_start = sim_time
                    print(
                        f"首次入位：pos={position} yaw={final_yaw:.2f} "
                        f"target_yaw={target_yaw:.2f}",
                        flush=True,
                    )

            elif phase == "reverse_settle":
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                if sim_time - phase_start >= PARK_PAUSE_SECONDS:
                    phase = "reverse_align"
                    phase_start = sim_time
                    reverse_reference = position

            elif phase == "reverse_align":
                error = yaw_error(final_yaw, target_yaw)
                # 倒车时方向盘作用方向与前进相反。
                steering = _clamp(-error / 25.0, -0.65, 0.65)
                vc = VehicleControl(
                    gear=GearMode.REVERSE,
                    throttle=PARK_REVERSE_THROTTLE,
                    brake=0.0,
                    steering=steering,
                )
                reversed_distance = _distance(position, reverse_reference)
                if (
                    abs(error) <= PARK_ALIGN_TOLERANCE_DEG
                    or reversed_distance >= PARK_REVERSE_DISTANCE_M
                    or sim_time - phase_start >= 5.0
                ):
                    phase = "forward_settle"
                    phase_start = sim_time

            elif phase == "forward_settle":
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                if speed <= 0.20 and sim_time - phase_start >= PARK_PAUSE_SECONDS:
                    recovery_path = build_alignment_path(
                        final_x,
                        final_y,
                        final_yaw,
                        slot,
                        PARK_RECOVERY_CONTROL_M,
                    )
                    recovery_pursuit = PurePursuit(
                        recovery_path,
                        lookahead=PARK_RECOVERY_LOOKAHEAD_M,
                    )
                    phase = "forward_align"
                    phase_start = sim_time

            elif phase == "forward_align":
                steering = recovery_pursuit.steering(
                    final_x, final_y, final_yaw
                )
                vc = VehicleControl(
                    gear=GearMode.DRIVE,
                    throttle=PARK_ADJUST_THROTTLE,
                    brake=0.0,
                    steering=steering,
                )
                angle_ok = (
                    current_yaw_error is not None
                    and abs(current_yaw_error) <= PARK_FINAL_ALIGN_DEG
                )
                if angle_ok and sim_time - phase_start >= 0.8:
                    if distance_to_slot <= PARK_POSITION_TOLERANCE_M:
                        phase = "final_stop"
                        phase_start = sim_time
                    else:
                        phase = "center_reverse"
                        phase_start = sim_time
                elif sim_time - phase_start >= PARK_ALIGN_TIMEOUT_SECONDS:
                    phase = "final_stop"
                    phase_start = sim_time

            elif phase == "center_reverse":
                heading = -math.radians(final_yaw)
                along = (
                    (float(slot.pos_x) - final_x) * math.cos(heading)
                    + (float(slot.pos_y) - final_y) * math.sin(heading)
                )
                if abs(along) <= PARK_POSITION_TOLERANCE_M and speed <= 0.15:
                    phase = "final_stop"
                    phase_start = sim_time
                elif abs(along) <= 0.08:
                    phase = "final_stop"
                    phase_start = sim_time
                else:
                    gear = GearMode.REVERSE if along < 0.0 else GearMode.DRIVE
                    vc = VehicleControl(
                        gear=gear,
                        throttle=PARK_ADJUST_THROTTLE,
                        brake=0.0,
                        steering=0.0,
                    )
                    if sim_time - phase_start >= PARK_ALIGN_TIMEOUT_SECONDS:
                        phase = "final_stop"
                        phase_start = sim_time

            elif phase == "final_stop":
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                if speed <= 0.15 and sim_time - phase_start >= PARK_PAUSE_SECONDS:
                    position_ok = distance_to_slot <= PARK_POSITION_TOLERANCE_M
                    angle_ok = (
                        current_yaw_error is not None
                        and abs(current_yaw_error) <= PARK_FINAL_ALIGN_DEG
                    )
                    print(
                        f"调整轮次 {adjust_cycles + 1}: "
                        f"distance={distance_to_slot:.3f} "
                        f"angle_error={current_yaw_error:+.3f} "
                        f"position_ok={position_ok} angle_ok={angle_ok}",
                        flush=True,
                    )
                    if (position_ok and angle_ok) or (
                        adjust_cycles >= PARK_MAX_ADJUST_CYCLES
                    ):
                        phase = "parked"
                        phase_start = sim_time
                    else:
                        adjust_cycles += 1
                        target_yaw = nearest_parallel_yaw(
                            final_yaw, float(slot.ori_z)
                        )
                        phase = "reverse_settle"
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
                raise RuntimeError(f"未知泊车阶段: {phase}")

            api.set_vehicle_control(vc)
            writer.writerow(
                [
                    frame,
                    round(sim_time, 4),
                    phase,
                    round(final_x, 4),
                    round(final_y, 4),
                    round(final_yaw, 3),
                    None if target_yaw is None else round(target_yaw, 3),
                    (
                        None
                        if current_yaw_error is None
                        else round(current_yaw_error, 3)
                    ),
                    round(speed, 4),
                    round(distance_to_slot, 4),
                    round(vc.steering, 4),
                    round(vc.throttle, 4),
                    round(vc.brake, 4),
                ]
            )
            if frame % 10 == 1:
                error_text = (
                    "n/a"
                    if current_yaw_error is None
                    else f"{current_yaw_error:+6.2f}"
                )
                print(
                    f"[{sim_time:6.1f}s] {phase:14s} "
                    f"pos=({final_x:7.2f},{final_y:7.2f}) "
                    f"yaw={final_yaw:7.2f} err={error_text} "
                    f"v={speed:5.2f} d={distance_to_slot:5.2f} "
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

    slot_error = None
    slot_yaw_error = None
    if slot is not None:
        slot_error = math.hypot(
            final_x - float(slot.pos_x),
            final_y - float(slot.pos_y),
        )
        slot_yaw_error = abs(yaw_error(final_yaw, float(slot.ori_z)))
        slot_yaw_error = min(slot_yaw_error, 180.0 - slot_yaw_error)
    summary = {
        "scene": [s.name for s in static.sub_scenes],
        "ending": ending,
        "phase": phase,
        "adjust_cycles": adjust_cycles,
        "frames": frame,
        "slot_center": (
            None if slot is None else [round(slot.pos_x, 4), round(slot.pos_y, 4)]
        ),
        "slot_yaw": None if slot is None else round(slot.ori_z, 3),
        "target_yaw": None if target_yaw is None else round(target_yaw, 3),
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
