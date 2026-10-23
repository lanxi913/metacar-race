# -*- coding: utf-8 -*-
r"""车辆紧急停止：在路线终点坐标系内精确停车。

运行顺序：
    .venv\Scripts\python.exe -u tools\emergency_stop_run.py
    → 客户端启动“车辆紧急停止”
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.config import (  # noqa: E402
    EMERGENCY_STOP_HOLD_SECONDS,
    EMERGENCY_STOP_RESPONSE_SECONDS,
    LIMIT_SCENE_ELEMENT_KMH,
    LOOKAHEAD_M,
)
from metacar_race.control.emergency_stop import (  # noqa: E402
    endpoint_target_speed,
    make_stop_frame,
    minimum_endpoint_clearance,
    safe_stop_margin,
    stop_coordinates,
)
from metacar_race.control.follow import (  # noqa: E402
    LongitudinalController,
    adaptive_params,
    estimate_nearest_lane_width,
    select_leader,
)
from metacar_race.control.tracking import PurePursuit  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="车辆紧急停止")
    parser.add_argument("--max-seconds", type=float, default=100.0)
    parser.add_argument("--lookahead", type=float, default=LOOKAHEAD_M)
    args = parser.parse_args()

    print("等待场景连接...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)
    static = api.get_scene_static_data()
    route = list(static.route)
    if len(route) < 2:
        print("route 点不足，无法建立停车坐标系。", flush=True)
        return 1
    pursuit = PurePursuit(route, lookahead=args.lookahead)
    fallback_end = static.sub_scenes[-1].end_point if static.sub_scenes else None
    speed_limit_ms = LIMIT_SCENE_ELEMENT_KMH / 3.6

    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"emergency_stop_{stamp}.csv"
    json_path = log_dir / f"emergency_stop_{stamp}.json"
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
            "speed",
            "target_speed",
            "along_to_end",
            "lateral_to_end",
            "leader_gap",
            "leader_speed",
            "leader_clearance_at_end",
            "stop_margin",
            "lane_width",
            "min_gap",
            "max_speed",
            "throttle",
            "brake",
            "steering",
            "held_seconds",
        ]
    )

    controller = LongitudinalController()
    phase = "approach"
    frame = 0
    ending = "unknown"
    held_seconds = 0.0
    final_x = final_y = final_yaw = speed = 0.0
    along_to_end = lateral_to_end = control_along = 0.0
    leader = None
    leader_clearance_at_end = None
    stop_margin = 0.0
    target_speed = 0.0
    lane_width = 0.0
    params = None
    stop_frame = None
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

            endpoint = status.end_point or fallback_end
            if stop_frame is None and endpoint is not None:
                stop_frame = make_stop_frame(endpoint, route)
                print(
                    f"停车坐标系：origin=({stop_frame.x:.2f},"
                    f"{stop_frame.y:.2f}) "
                    f"forward=({stop_frame.forward_x:.3f},"
                    f"{stop_frame.forward_y:.3f})",
                    flush=True,
                )

            if stop_frame is None:
                api.set_vehicle_control(
                    VehicleControl(gear=GearMode.PARKING, brake=1.0)
                )
                continue

            coords = stop_coordinates(pose, stop_frame)
            along_to_end = coords.along_m
            lateral_to_end = coords.lateral_m
            lane_width = estimate_nearest_lane_width(
                static.roads,
                final_x,
                final_y,
                fallback=float(car.width) * 1.8,
            )
            params = adaptive_params(
                float(car.length),
                float(car.width),
                lane_width,
                speed_limit_ms,
            )
            leader = select_leader(
                msg.obstacles,
                pose,
                float(car.length),
                params,
            )
            min_clearance = minimum_endpoint_clearance(float(car.length))
            if leader is not None:
                leader_clearance_at_end = leader.gap_m - along_to_end
            stop_margin = safe_stop_margin(
                leader_clearance_at_end,
                min_clearance,
            )
            control_along = along_to_end - stop_margin

            target_speed = endpoint_target_speed(
                control_along,
                speed,
                params.max_speed_ms,
                EMERGENCY_STOP_RESPONSE_SECONDS,
                min_clearance,
            )
            if (
                leader_clearance_at_end is not None
                and leader_clearance_at_end < min_clearance
            ):
                target_speed = min(
                    target_speed,
                    math.sqrt(
                        2.0
                        * 0.5
                        * max(0.0, leader.gap_m - min_clearance)
                    ),
                )
                if leader.gap_m <= min_clearance:
                    target_speed = 0.0

            if phase == "approach":
                if control_along <= 0.05 and abs(speed) <= 0.10:
                    phase = "hold"
                    held_seconds = 0.0
                    print(
                        f"停车坐标系到位：along={along_to_end:+.3f} "
                        f"lateral={lateral_to_end:+.3f} "
                        f"v={speed:.2f}",
                        flush=True,
                    )
                else:
                    throttle, brake = controller.update(
                        speed,
                        target_speed,
                        None,
                        dt,
                        params,
                    )
                    steering = pursuit.steering(
                        final_x,
                        final_y,
                        final_yaw,
                    )
                    if along_to_end < 0.6 * float(car.length):
                        steering = 0.0
                    vc = VehicleControl(
                        gear=GearMode.DRIVE,
                        throttle=throttle,
                        brake=brake,
                        steering=steering,
                    )
            else:
                held_seconds += dt
                throttle = brake = steering = 0.0
                vc = VehicleControl(
                    gear=GearMode.PARKING,
                    brake=min(1.0, 0.22 + 0.55 * held_seconds),
                )
                if held_seconds >= EMERGENCY_STOP_HOLD_SECONDS:
                    ending = "hold_complete"
                    break

            api.set_vehicle_control(vc)
            writer.writerow(
                [
                    frame,
                    round(float(status.used_time), 4),
                    phase,
                    round(final_x, 4),
                    round(final_y, 4),
                    round(final_yaw, 3),
                    round(speed, 4),
                    round(target_speed, 4),
                    round(along_to_end, 4),
                    round(lateral_to_end, 4),
                    None if leader is None else round(leader.gap_m, 4),
                    None if leader is None else round(leader.speed_ms, 4),
                    (
                        None
                        if leader_clearance_at_end is None
                        else round(leader_clearance_at_end, 4)
                    ),
                    round(stop_margin, 4),
                    round(lane_width, 4),
                    round(params.min_gap_m, 4),
                    round(params.max_speed_ms, 4),
                    round(throttle, 4),
                    round(brake, 4),
                    round(steering, 4),
                    round(held_seconds, 3),
                ]
            )
            if frame % 10 == 1:
                print(
                    f"[{status.used_time:6.1f}s] {phase:8s} "
                    f"pos=({final_x:7.2f},{final_y:7.2f}) "
                    f"v={speed:5.2f} vt={target_speed:5.2f} "
                    f"along={along_to_end:+6.2f} "
                    f"lat={lateral_to_end:+5.2f} "
                    f"lead_clear={leader_clearance_at_end} "
                    f"stop_margin={stop_margin:.2f} "
                    f"th={throttle:4.2f} br={brake:4.2f}",
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
        "ending": ending,
        "phase": phase,
        "frames": frame,
        "final_x": round(final_x, 4),
        "final_y": round(final_y, 4),
        "final_yaw": round(final_yaw, 3),
        "speed": round(speed, 4),
        "along_to_end": round(along_to_end, 4),
        "lateral_to_end": round(lateral_to_end, 4),
        "control_along": round(control_along, 4),
        "stop_margin": round(stop_margin, 4),
        "leader": None if leader is None else leader.__dict__,
        "leader_clearance_at_end": leader_clearance_at_end,
        "adaptive_params": None if params is None else params.__dict__,
        "stop_frame": None if stop_frame is None else stop_frame.__dict__,
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
