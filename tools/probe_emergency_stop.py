# -*- coding: utf-8 -*-
r"""车辆紧急停止场景探针。

低速沿路线行驶，记录所有障碍物、交通灯、最近前方目标和关键帧截图。
所有动态阈值由车辆尺寸、车道宽度和当前速度推导。

运行顺序：
    .venv\Scripts\python.exe -u tools\probe_emergency_stop.py
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

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, ObstacleType, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.config import LIMIT_SCENE_ELEMENT_KMH  # noqa: E402
from metacar_race.control.follow import (  # noqa: E402
    LongitudinalController,
    adaptive_params,
    estimate_nearest_lane_width,
)
from metacar_race.control.tracking import PurePursuit  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
IGNORED_TYPES = {
    ObstacleType.ROAD_MARK,
    ObstacleType.TRAFFIC_SIGN,
    ObstacleType.SPEED_LIMIT_SIGN,
    ObstacleType.TRAFFIC_LIGHT,
    ObstacleType.PARKING_SLOT,
}


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def enum_name(value) -> str:
    return str(getattr(value, "name", value))


def vector_dict(value):
    if value is None:
        return None
    return {
        "x": float(value.x),
        "y": float(value.y),
        "z": float(value.z),
    }


def nearest_ahead(obstacles, pose, max_lateral: float):
    heading = -math.radians(float(pose.ori_z))
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    best = None
    for obstacle in obstacles:
        if obstacle.type in IGNORED_TYPES:
            continue
        dx = float(obstacle.pos_x) - float(pose.pos_x)
        dy = float(obstacle.pos_y) - float(pose.pos_y)
        forward = dx * cos_h + dy * sin_h
        lateral = -dx * sin_h + dy * cos_h
        if forward <= 0.0 or abs(lateral) > max_lateral:
            continue
        item = (forward, lateral, obstacle)
        if best is None or item[0] < best[0]:
            best = item
    return best


def main() -> int:
    parser = argparse.ArgumentParser(description="车辆紧急停止场景探针")
    parser.add_argument("--max-seconds", type=float, default=75.0)
    parser.add_argument("--lookahead", type=float, default=6.0)
    parser.add_argument("--snapshot-every", type=float, default=2.0)
    args = parser.parse_args()

    print("等待场景连接...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。低速探针开始记录。", flush=True)

    static = api.get_scene_static_data()
    route = list(static.route)
    if len(route) < 2:
        print("route 点不足，无法循迹。", flush=True)
        return 1
    pursuit = PurePursuit(route, lookahead=args.lookahead)
    fallback_end = static.sub_scenes[-1].end_point if static.sub_scenes else None
    speed_limit_ms = LIMIT_SCENE_ELEMENT_KMH / 3.6

    log_dir = ROOT / "logs"
    snapshot_dir = log_dir / "emergency_stop_snapshots"
    log_dir.mkdir(exist_ok=True)
    snapshot_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    ego_csv_path = log_dir / f"emergency_stop_ego_{stamp}.csv"
    obstacle_csv_path = log_dir / f"emergency_stop_obstacles_{stamp}.csv"
    summary_path = log_dir / f"emergency_stop_probe_{stamp}.json"

    ego_file = ego_csv_path.open("w", newline="", encoding="utf-8")
    obstacle_file = obstacle_csv_path.open("w", newline="", encoding="utf-8")
    ego_writer = csv.writer(ego_file)
    obstacle_writer = csv.writer(obstacle_file)
    ego_writer.writerow(
        [
            "frame",
            "sim_time",
            "x",
            "y",
            "yaw",
            "speed",
            "gear",
            "target_speed",
            "nearest_forward",
            "nearest_lateral",
            "nearest_type",
            "nearest_id",
            "remaining_to_end",
            "throttle",
            "brake",
            "steering",
            "lane_width",
            "min_gap",
            "traffic_light_groups",
        ]
    )
    obstacle_writer.writerow(
        [
            "frame",
            "sim_time",
            "obstacle_id",
            "type",
            "x",
            "y",
            "yaw",
            "vel_x",
            "vel_y",
            "speed",
            "length",
            "width",
            "forward_m",
            "lateral_m",
            "heading_diff_deg",
            "extra_info",
        ]
    )

    controller = LongitudinalController()
    seen = {}
    frame = 0
    ending = "unknown"
    final_pose = None
    final_speed = 0.0
    final_remaining = None
    next_snapshot = 0.0
    snapshot_files = []
    try:
        for frame, (msg, frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            final_pose = pose
            final_speed = float(car.speed)
            dt = 1.0 / 30.0

            endpoint = status.end_point or fallback_end
            if endpoint is not None:
                final_remaining = math.hypot(
                    float(endpoint.x) - float(pose.pos_x),
                    float(endpoint.y) - float(pose.pos_y),
                )

            lane_width = estimate_nearest_lane_width(
                static.roads,
                float(pose.pos_x),
                float(pose.pos_y),
                fallback=float(car.width) * 1.8,
            )
            params = adaptive_params(
                float(car.length),
                float(car.width),
                lane_width,
                speed_limit_ms,
            )
            nearest = nearest_ahead(
                msg.obstacles,
                pose,
                params.max_lateral_m,
            )
            if nearest is None:
                target_speed = min(3.0, params.max_speed_ms)
                nearest_forward = nearest_lateral = None
                nearest_type = nearest_id = None
                command_gap = None
            else:
                nearest_forward, nearest_lateral, obstacle = nearest
                nearest_type = enum_name(obstacle.type)
                nearest_id = obstacle.id
                command_gap = max(
                    0.0,
                    nearest_forward - 0.5 * float(car.length),
                )
                if command_gap < params.min_gap_m:
                    target_speed = 0.0
                elif command_gap < 2.0 * params.min_gap_m:
                    target_speed = min(1.0, params.max_speed_ms)
                else:
                    target_speed = min(3.0, params.max_speed_ms)

            if final_remaining is not None:
                target_speed = min(
                    target_speed,
                    math.sqrt(max(0.0, 2.0 * final_remaining)),
                )

            throttle, brake = controller.update(
                final_speed,
                target_speed,
                command_gap,
                dt,
                params,
            )
            steering = pursuit.steering(
                float(pose.pos_x),
                float(pose.pos_y),
                float(pose.ori_z),
            )
            vc = VehicleControl(
                gear=GearMode.DRIVE,
                throttle=throttle,
                brake=brake,
                steering=steering,
            )
            api.set_vehicle_control(vc)

            ego_writer.writerow(
                [
                    frame,
                    round(float(status.used_time), 4),
                    round(float(pose.pos_x), 4),
                    round(float(pose.pos_y), 4),
                    round(float(pose.ori_z), 3),
                    round(final_speed, 4),
                    enum_name(car.gear),
                    round(target_speed, 4),
                    None if nearest_forward is None else round(nearest_forward, 4),
                    None if nearest_lateral is None else round(nearest_lateral, 4),
                    nearest_type,
                    nearest_id,
                    None if final_remaining is None else round(final_remaining, 4),
                    round(throttle, 4),
                    round(brake, 4),
                    round(steering, 4),
                    round(lane_width, 4),
                    round(params.min_gap_m, 4),
                    len(msg.traffic_light_groups),
                ]
            )

            heading = -math.radians(float(pose.ori_z))
            for obstacle in msg.obstacles:
                dx = float(obstacle.pos_x) - float(pose.pos_x)
                dy = float(obstacle.pos_y) - float(pose.pos_y)
                forward = dx * math.cos(heading) + dy * math.sin(heading)
                lateral = -dx * math.sin(heading) + dy * math.cos(heading)
                obstacle_writer.writerow(
                    [
                        frame,
                        round(float(status.used_time), 4),
                        obstacle.id,
                        enum_name(obstacle.type),
                        round(float(obstacle.pos_x), 4),
                        round(float(obstacle.pos_y), 4),
                        round(float(obstacle.ori_z), 3),
                        round(float(obstacle.vel_x), 4),
                        round(float(obstacle.vel_y), 4),
                        round(
                            math.hypot(
                                float(obstacle.vel_x),
                                float(obstacle.vel_y),
                            ),
                            4,
                        ),
                        round(float(obstacle.length), 3),
                        round(float(obstacle.width), 3),
                        round(forward, 4),
                        round(lateral, 4),
                        round(
                            wrap_deg(
                                float(obstacle.ori_z) - float(pose.ori_z)
                            ),
                            3,
                        ),
                        obstacle.extra_info,
                    ]
                )
                if (obstacle.id, enum_name(obstacle.type)) not in seen:
                    seen[(obstacle.id, enum_name(obstacle.type))] = {
                        "id": obstacle.id,
                        "type": enum_name(obstacle.type),
                        "first_seen_s": round(float(status.used_time), 3),
                        "first_forward": round(forward, 4),
                        "first_lateral": round(lateral, 4),
                    }

            if float(status.used_time) >= next_snapshot and frames:
                snapshot_name = (
                    f"emergency_stop_{stamp}_"
                    f"{float(status.used_time):06.2f}.jpg"
                )
                snapshot_path = snapshot_dir / snapshot_name
                if cv2.imwrite(
                    str(snapshot_path),
                    frames[0].frame,
                ):
                    snapshot_files.append(str(snapshot_path))
                next_snapshot += max(0.5, args.snapshot_every)

            if frame % 15 == 1:
                nearest_text = "none"
                if nearest is not None:
                    nearest_text = (
                        f"id={nearest_id} type={nearest_type} "
                        f"forward={nearest_forward:.2f} "
                        f"lateral={nearest_lateral:.2f}"
                    )
                print(
                    f"[{status.used_time:6.1f}s] "
                    f"pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                    f"v={final_speed:5.2f} target={target_speed:5.2f} "
                    f"{nearest_text} th={throttle:4.2f} br={brake:4.2f}",
                    flush=True,
                )

            if status.used_time >= args.max_seconds:
                ending = "probe_complete"
                break
    except KeyboardInterrupt:
        ending = "interrupt"
    finally:
        try:
            ego_file.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            obstacle_file.close()
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
        "frames": frame,
        "final_pose": None if final_pose is None else final_pose.model_dump(mode="json"),
        "final_speed": round(final_speed, 4),
        "final_remaining": (
            None if final_remaining is None else round(final_remaining, 4)
        ),
        "sub_scenes": [
            {
                "name": scene.name,
                "start_point": vector_dict(scene.start_point),
                "end_point": vector_dict(scene.end_point),
            }
            for scene in static.sub_scenes
        ],
        "route_points": [vector_dict(point) for point in static.route],
        "roads": [road.model_dump(mode="json") for road in static.roads],
        "obstacles_seen": list(seen.values()),
        "snapshots": snapshot_files,
        "ego_csv": str(ego_csv_path),
        "obstacles_csv": str(obstacle_csv_path),
    }
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(
        "PROBE_SUMMARY "
        + json.dumps(
            {
                "ending": ending,
                "frames": frame,
                "final_speed": round(final_speed, 3),
                "final_remaining": (
                    None
                    if final_remaining is None
                    else round(final_remaining, 3)
                ),
                "obstacles_seen": len(seen),
                "snapshots": len(snapshot_files),
                "summary": str(summary_path),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
