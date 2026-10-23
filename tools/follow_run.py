# -*- coding: utf-8 -*-
r"""车辆跟随首版：单车道、单前车、直线路线。

运行顺序：
    .venv\Scripts\python.exe -u tools\follow_run.py
    → 客户端启动“车辆跟随”
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
    FOLLOW_END_TOL_M,
    FOLLOW_HOLD_SECONDS,
    FOLLOW_LOOKAHEAD_M,
)
from metacar_race.control.follow import (  # noqa: E402
    LongitudinalController,
    compute_target_speed,
    select_leader,
)
from metacar_race.control.tracking import PurePursuit  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="车辆跟随首版")
    parser.add_argument("--max-seconds", type=float, default=60.0)
    parser.add_argument("--lookahead", type=float, default=FOLLOW_LOOKAHEAD_M)
    args = parser.parse_args()

    print("等待场景连接...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)
    static = api.get_scene_static_data()
    route = list(static.route)
    if len(route) < 2:
        print("route 点不足，无法循迹。", flush=True)
        return 1
    pursuit = PurePursuit(route, lookahead=args.lookahead)
    fallback_end = static.sub_scenes[-1].end_point if static.sub_scenes else None

    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"follow_{stamp}.csv"
    json_path = log_dir / f"follow_{stamp}.json"
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
            "leader_id",
            "leader_type",
            "leader_forward",
            "leader_lateral",
            "leader_gap",
            "leader_speed",
            "desired_gap",
            "safe_speed",
            "remaining_to_end",
            "throttle",
            "brake",
            "steering",
            "held_seconds",
        ]
    )

    controller = LongitudinalController()
    phase = "follow"
    held_seconds = 0.0
    frame = 0
    ending = "unknown"
    final_x = final_y = final_yaw = speed = 0.0
    leader = None
    target_speed = desired_gap = safe_speed = 0.0
    remaining_to_end = None
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
            if endpoint is not None:
                remaining_to_end = math.hypot(
                    float(endpoint.x) - final_x,
                    float(endpoint.y) - final_y,
                )

            leader = select_leader(
                msg.obstacles,
                pose,
                float(car.length),
            )
            target_speed, desired_gap, safe_speed = compute_target_speed(
                leader,
                remaining_to_end,
            )

            if phase == "follow":
                if (
                    remaining_to_end is not None
                    and remaining_to_end <= FOLLOW_END_TOL_M
                    and abs(speed) <= 0.12
                ):
                    phase = "hold"
                    held_seconds = 0.0
                    print(
                        f"到达终点：remaining={remaining_to_end:.3f} "
                        f"v={speed:.2f}",
                        flush=True,
                    )
                else:
                    throttle, brake = controller.update(
                        speed,
                        target_speed,
                        None if leader is None else leader.gap_m,
                        dt,
                    )
                    steering = pursuit.steering(
                        final_x,
                        final_y,
                        final_yaw,
                    )
                    if (
                        remaining_to_end is not None
                        and remaining_to_end < 2.0
                    ):
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
                if held_seconds >= FOLLOW_HOLD_SECONDS:
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
                    None if leader is None else leader.obstacle_id,
                    None if leader is None else leader.obstacle_type,
                    None if leader is None else round(leader.forward_m, 4),
                    None if leader is None else round(leader.lateral_m, 4),
                    None if leader is None else round(leader.gap_m, 4),
                    None if leader is None else round(leader.speed_ms, 4),
                    round(desired_gap, 4),
                    round(safe_speed, 4),
                    (
                        None
                        if remaining_to_end is None
                        else round(remaining_to_end, 4)
                    ),
                    round(throttle, 4),
                    round(brake, 4),
                    round(steering, 4),
                    round(held_seconds, 3),
                ]
            )
            if frame % 10 == 1:
                leader_text = "none"
                if leader is not None:
                    leader_text = (
                        f"id={leader.obstacle_id} gap={leader.gap_m:5.2f} "
                        f"lead_v={leader.speed_ms:5.2f}"
                    )
                print(
                    f"[{status.used_time:6.1f}s] {phase:5s} "
                    f"pos=({final_x:7.2f},{final_y:7.2f}) "
                    f"v={speed:5.2f} vt={target_speed:5.2f} "
                    f"{leader_text} th={throttle:4.2f} "
                    f"br={brake:4.2f} st={steering:+5.2f}",
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
        "remaining_to_end": (
            None if remaining_to_end is None else round(remaining_to_end, 4)
        ),
        "leader": None if leader is None else leader.__dict__,
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
