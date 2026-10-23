# -*- coding: utf-8 -*-
r"""车辆跟随场景探针。

主车全程保持驻车制动，只记录场景静态数据和动态障碍物。用于在实现
跟车控制器前确认前车类型、相对位置、速度和场景评分条件。

运行顺序：
    .venv\Scripts\python.exe -u tools\probe_follow.py
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

from metacar import GearMode, ObstacleType, SceneAPI, VehicleControl  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
VEHICLE_TYPES = {
    ObstacleType.CAR,
    ObstacleType.TRUCK,
    ObstacleType.BUS,
    ObstacleType.SPECIAL_VEHICLE,
    ObstacleType.DYNAMIC,
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


def relative_xy(pose, obstacle) -> tuple[float, float]:
    """返回障碍物在车辆坐标系中的前向和左向距离。"""
    heading = -math.radians(float(pose.ori_z))
    dx = float(obstacle.pos_x) - float(pose.pos_x)
    dy = float(obstacle.pos_y) - float(pose.pos_y)
    forward = dx * math.cos(heading) + dy * math.sin(heading)
    lateral = -dx * math.sin(heading) + dy * math.cos(heading)
    return forward, lateral


def main() -> int:
    parser = argparse.ArgumentParser(description="车辆跟随场景只读探针")
    parser.add_argument("--max-seconds", type=float, default=45.0)
    parser.add_argument("--print-every", type=int, default=30)
    args = parser.parse_args()

    print("等待场景连接...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。主车保持驻车制动，开始记录。", flush=True)

    static = api.get_scene_static_data()
    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    ego_path = log_dir / f"follow_probe_ego_{stamp}.csv"
    obstacle_path = log_dir / f"follow_probe_obstacles_{stamp}.csv"
    summary_path = log_dir / f"follow_probe_{stamp}.json"

    ego_file = ego_path.open("w", newline="", encoding="utf-8")
    obstacle_file = obstacle_path.open("w", newline="", encoding="utf-8")
    ego_writer = csv.writer(ego_file)
    obstacle_writer = csv.writer(obstacle_file)
    ego_writer.writerow(
        [
            "frame",
            "wall_time",
            "sim_time",
            "scene_name",
            "sub_scene_name",
            "x",
            "y",
            "z",
            "yaw",
            "vel_x",
            "vel_y",
            "speed",
            "gear",
            "throttle",
            "brake",
            "steering",
            "time_limit",
            "end_x",
            "end_y",
            "trajectory_points",
            "road_count",
            "obstacle_count",
        ]
    )
    obstacle_writer.writerow(
        [
            "frame",
            "wall_time",
            "sim_time",
            "obstacle_id",
            "type",
            "x",
            "y",
            "z",
            "vel_x",
            "vel_y",
            "vel_z",
            "speed",
            "yaw",
            "length",
            "width",
            "height",
            "forward_m",
            "lateral_m",
            "center_distance_m",
            "heading_diff_deg",
            "vehicle_candidate",
            "extra_info",
        ]
    )

    start_wall = time.time()
    frame = 0
    sim_time = 0.0
    ending = "unknown"
    seen: dict[int, dict] = {}
    final_pose = None
    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            sim_time = float(status.used_time)
            wall_time = time.time() - start_wall
            final_pose = pose

            api.set_vehicle_control(
                VehicleControl(gear=GearMode.PARKING, brake=1.0)
            )

            end_point = status.end_point
            ego_writer.writerow(
                [
                    frame,
                    round(wall_time, 4),
                    round(sim_time, 4),
                    status.sub_scene_name,
                    status.sub_scene_name,
                    round(float(pose.pos_x), 4),
                    round(float(pose.pos_y), 4),
                    round(float(pose.pos_z), 4),
                    round(float(pose.ori_z), 3),
                    round(float(pose.vel_x), 4),
                    round(float(pose.vel_y), 4),
                    round(float(car.speed), 4),
                    enum_name(car.gear),
                    round(float(car.throttle), 4),
                    round(float(car.brake), 4),
                    round(float(car.steering), 4),
                    round(float(status.time_limit), 3),
                    None if end_point is None else round(float(end_point.x), 4),
                    None if end_point is None else round(float(end_point.y), 4),
                    len(msg.trajectory),
                    len(static.roads),
                    len(msg.obstacles),
                ]
            )

            for obstacle in msg.obstacles:
                forward, lateral = relative_xy(pose, obstacle)
                obs_type = obstacle.type
                is_vehicle = obs_type in VEHICLE_TYPES
                speed = math.hypot(float(obstacle.vel_x), float(obstacle.vel_y))
                heading_diff = wrap_deg(
                    float(obstacle.ori_z) - float(pose.ori_z)
                )
                obstacle_writer.writerow(
                    [
                        frame,
                        round(wall_time, 4),
                        round(sim_time, 4),
                        obstacle.id,
                        enum_name(obs_type),
                        round(float(obstacle.pos_x), 4),
                        round(float(obstacle.pos_y), 4),
                        round(float(obstacle.pos_z), 4),
                        round(float(obstacle.vel_x), 4),
                        round(float(obstacle.vel_y), 4),
                        round(float(obstacle.vel_z), 4),
                        round(speed, 4),
                        round(float(obstacle.ori_z), 3),
                        round(float(obstacle.length), 3),
                        round(float(obstacle.width), 3),
                        round(float(obstacle.height), 3),
                        round(forward, 4),
                        round(lateral, 4),
                        round(math.hypot(forward, lateral), 4),
                        round(heading_diff, 3),
                        is_vehicle,
                        obstacle.extra_info,
                    ]
                )
                if obstacle.id not in seen:
                    seen[obstacle.id] = {
                        "id": obstacle.id,
                        "type": enum_name(obs_type),
                        "first_seen_s": round(sim_time, 3),
                        "first_x": round(float(obstacle.pos_x), 4),
                        "first_y": round(float(obstacle.pos_y), 4),
                        "length": round(float(obstacle.length), 3),
                        "width": round(float(obstacle.width), 3),
                    }
                    print(
                        f"新障碍物 id={obstacle.id} "
                        f"type={enum_name(obs_type)} "
                        f"forward={forward:.2f} lateral={lateral:.2f}",
                        flush=True,
                    )

            front_vehicles = []
            for obstacle in msg.obstacles:
                if obstacle.type not in VEHICLE_TYPES:
                    continue
                forward, lateral = relative_xy(pose, obstacle)
                if forward > 0.0:
                    front_vehicles.append((forward, lateral, obstacle))
            front_vehicles.sort(key=lambda item: item[0])
            front = front_vehicles[0] if front_vehicles else None

            if frame % max(1, args.print_every) == 1:
                front_text = "none"
                if front is not None:
                    forward, lateral, obstacle = front
                    front_text = (
                        f"id={obstacle.id} type={enum_name(obstacle.type)} "
                        f"forward={forward:.2f} lateral={lateral:.2f} "
                        f"v={math.hypot(obstacle.vel_x, obstacle.vel_y):.2f}"
                    )
                print(
                    f"[{sim_time:6.1f}s] sub_scene={status.sub_scene_name!r} "
                    f"pos=({pose.pos_x:.2f},{pose.pos_y:.2f}) "
                    f"yaw={pose.ori_z:.1f} v={car.speed:5.2f} "
                    f"obstacles={len(msg.obstacles)} front={front_text}",
                    flush=True,
                )

            if sim_time >= args.max_seconds:
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
        "sim_time": round(sim_time, 4),
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
        "final_pose": (
            None if final_pose is None else final_pose.model_dump(mode="json")
        ),
        "ego_csv": str(ego_path),
        "obstacles_csv": str(obstacle_path),
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
                "sim_time": round(sim_time, 3),
                "obstacles_seen": len(seen),
                "summary": str(summary_path),
                "ego_csv": str(ego_path),
                "obstacles_csv": str(obstacle_path),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
