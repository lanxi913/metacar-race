# -*- coding: utf-8 -*-
r"""垂直车道泊车 · chang-kou 基线。

策略（2026-10-23 实测调整）：
    1. 从车道直行，开过车位。
    2. 在车位中心前方约 5.6 m 停稳。
    3. 挂 R 挡，持续反向转向，让车尾扫进车位。
    4. 车身与车位轴线平行、中心误差达标后驻车保持。

运行顺序：
    .venv\Scripts\python.exe -u templates\parking\chang-kou\run.py
    → 客户端启动“垂直车道泊车”
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, ObstacleType, SceneAPI, VehicleControl  # noqa: E402


APPROACH_THROTTLE = 0.01
STAGE_BRAKE = 0.10
STAGE_DECEL_MS2 = 2.70
STAGE_LEAD_M = 0.10
STAGE_STILL_SPEED_MS = 0.30
STAGE_SETTLE_SECONDS = 0.80

REVERSE_THROTTLE = 0.16
REVERSE_BRAKE = 0.20
REVERSE_MAX_SPEED_MS = 1.20
REVERSE_STOP_START_M = 1.50
REVERSE_CREEP_THROTTLE = 0.04
REVERSE_STEER_SIGN = -1.0
CENTER_TOL_M = 0.35
HEADING_TOL_DEG = 8.0
HOLD_SECONDS = 5.0


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def find_slot(obstacles):
    slots = [
        obstacle
        for obstacle in obstacles
        if obstacle.type == ObstacleType.PARKING_SLOT
    ]
    if not slots:
        return None
    return slots[0]


def main() -> int:
    parser = argparse.ArgumentParser(description="垂直车道泊车倒车入库")
    parser.add_argument("--stage-offset", type=float, default=None,
                        help="默认取车道到车位的横向距离")
    parser.add_argument("--max-seconds", type=float, default=60.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)
    static = api.get_scene_static_data()

    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"parking_chang-kou_{stamp}.csv"
    json_path = log_dir / f"parking_chang-kou_{stamp}.json"
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
            "heading_error",
            "distance_to_center",
            "speed",
            "steering",
            "throttle",
            "brake",
        ]
    )

    slot = None
    stage_point = None
    stage_ready = False
    phase = "approach"
    stationary_seconds = 0.0
    hold_seconds = 0.0
    final_x = final_y = final_yaw = 0.0
    frame = 0
    ending = "unknown"
    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            final_x = float(pose.pos_x)
            final_y = float(pose.pos_y)
            final_yaw = float(pose.ori_z)
            speed = float(car.speed)
            dt = 1.0 / 30.0

            if slot is None:
                slot = find_slot(msg.obstacles)
                if slot is None:
                    api.set_vehicle_control(
                        VehicleControl(gear=GearMode.PARKING, brake=1.0)
                    )
                    continue
                lateral = abs(final_x - float(slot.pos_x))
                stage_offset = (
                    lateral if args.stage_offset is None else args.stage_offset
                )
                direction_y = -1.0 if final_y > float(slot.pos_y) else 1.0
                stage_point = (
                    final_x,
                    float(slot.pos_y) + direction_y * stage_offset,
                )
                print(
                    f"车位 center=({slot.pos_x:.2f},{slot.pos_y:.2f}) "
                    f"yaw={slot.ori_z:.2f} size={slot.length:.2f}x{slot.width:.2f}; "
                    f"倒车准备点={stage_point}",
                    flush=True,
                )

            center_distance = distance(
                (final_x, final_y),
                (float(slot.pos_x), float(slot.pos_y)),
            )
            heading_error = wrap_deg(float(slot.ori_z) - final_yaw)

            if phase == "approach":
                stop_distance = max(
                    0.0,
                    speed * speed / (2.0 * STAGE_DECEL_MS2) + STAGE_LEAD_M,
                )
                if (
                    abs(final_x - stage_point[0]) <= 1.0
                    and abs(final_y - stage_point[1]) <= stop_distance
                ):
                    if speed <= STAGE_STILL_SPEED_MS:
                        vc = VehicleControl(
                            gear=GearMode.PARKING, brake=1.0
                        )
                        stationary_seconds += dt
                    else:
                        vc = VehicleControl(
                            gear=GearMode.DRIVE,
                            throttle=0.0,
                            brake=STAGE_BRAKE,
                            steering=0.0,
                        )
                        stationary_seconds = 0.0
                    if stationary_seconds >= STAGE_SETTLE_SECONDS:
                        stage_ready = True
                        phase = "reverse"
                        stationary_seconds = 0.0
                        print(
                            f"到达倒车准备点：pos=({final_x:.2f},{final_y:.2f}) "
                            f"yaw={final_yaw:.2f}",
                            flush=True,
                        )
                else:
                    vc = VehicleControl(
                        gear=GearMode.DRIVE,
                        throttle=APPROACH_THROTTLE,
                        brake=0.0,
                        steering=0.0,
                    )

            elif phase == "reverse":
                if (
                    center_distance <= CENTER_TOL_M
                    and abs(heading_error) <= HEADING_TOL_DEG
                    and abs(speed) <= STAGE_STILL_SPEED_MS
                ):
                    phase = "hold"
                    hold_seconds = 0.0
                    vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                    print(
                        f"入库条件满足：d={center_distance:.3f} "
                        f"yaw_error={heading_error:+.2f}",
                        flush=True,
                    )
                else:
                    steering = max(
                        -1.0,
                        min(
                            1.0,
                            REVERSE_STEER_SIGN * heading_error / 30.0,
                        ),
                    )
                    if center_distance <= REVERSE_STOP_START_M:
                        throttle = 0.0
                        brake = REVERSE_BRAKE
                        if abs(speed) <= 0.10:
                            throttle = REVERSE_CREEP_THROTTLE
                            brake = 0.0
                    else:
                        throttle = REVERSE_THROTTLE
                        brake = 0.0
                        if abs(speed) > REVERSE_MAX_SPEED_MS:
                            throttle = 0.0
                            brake = REVERSE_BRAKE
                    vc = VehicleControl(
                        gear=GearMode.REVERSE,
                        throttle=throttle,
                        brake=brake,
                        steering=steering,
                    )
            elif phase == "hold":
                hold_seconds += dt
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                if hold_seconds >= HOLD_SECONDS:
                    ending = "hold_complete"
                    break
            else:
                raise RuntimeError(f"未知阶段: {phase}")

            if not args.dry_run:
                api.set_vehicle_control(vc)
            writer.writerow(
                [
                    frame,
                    round(float(status.used_time), 4),
                    phase,
                    round(final_x, 4),
                    round(final_y, 4),
                    round(final_yaw, 3),
                    round(float(slot.ori_z), 3),
                    round(heading_error, 3),
                    round(center_distance, 4),
                    round(speed, 4),
                    round(vc.steering, 4),
                    round(vc.throttle, 4),
                    round(vc.brake, 4),
                ]
            )
            if frame % 10 == 1:
                print(
                    f"[{status.used_time:6.1f}s] {phase:8s} "
                    f"pos=({final_x:7.2f},{final_y:7.2f}) "
                    f"yaw={final_yaw:7.2f} err={heading_error:+6.2f} "
                    f"d={center_distance:5.2f} v={speed:5.2f} "
                    f"st={vc.steering:+5.2f} th={vc.throttle:5.3f} "
                    f"br={vc.brake:4.2f}",
                    flush=True,
                )
            if status.used_time >= args.max_seconds:
                ending = "timeout"
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

    summary = {
        "scene": [s.name for s in static.sub_scenes],
        "ending": ending,
        "phase": phase,
        "frames": frame,
        "stage_ready": stage_ready,
        "slot_center": (
            None if slot is None else [round(slot.pos_x, 4), round(slot.pos_y, 4)]
        ),
        "final_x": round(final_x, 4),
        "final_y": round(final_y, 4),
        "final_yaw": round(final_yaw, 3),
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
