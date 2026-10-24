# -*- coding: utf-8 -*-
r"""环形转弯只读探针：采集路线、车道、实时轨迹和车辆状态。

运行顺序：
    .venv\Scripts\python.exe -u tools\probe_ring_turn.py
    → 终端出现“等待场景连接...”后，在客户端启动“环形转弯”场景

探针全程挂 P 挡并施加制动，不执行循迹控制。
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


ROOT = Path(__file__).resolve().parents[1]


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _nearest_on_polyline(points, x: float, y: float):
    """返回折线最近点的距离、最近线段和局部切向角。"""
    if not points:
        return None

    best = None
    for i in range(max(0, len(points) - 1)):
        ax, ay = float(points[i].x), float(points[i].y)
        bx, by = float(points[i + 1].x), float(points[i + 1].y)
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-9:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
        px, py = ax + t * dx, ay + t * dy
        distance = math.hypot(px - x, py - y)
        if best is None or distance < best["distance_m"]:
            best = {
                "distance_m": distance,
                "index": i,
                "projection_x": px,
                "projection_y": py,
                "heading_rad": math.atan2(dy, dx),
            }

    if best is None:
        p = points[0]
        return {
            "distance_m": math.hypot(float(p.x) - x, float(p.y) - y),
            "index": 0,
            "projection_x": float(p.x),
            "projection_y": float(p.y),
            "heading_rad": None,
        }
    return best


def _nearest_lane(roads, x: float, y: float, pose_heading: float):
    """从运行时道路数据中选择几何上最近的车道，不依赖车道 ID。"""
    best = None
    for road in roads:
        for lane in road.lanes:
            match = _nearest_on_polyline(lane.path_points, x, y)
            if match is None:
                continue
            heading_error = None
            if match["heading_rad"] is not None:
                heading_error = abs(
                    _wrap(match["heading_rad"] - pose_heading)
                )
            score = match["distance_m"]
            if heading_error is not None:
                score += 0.5 * heading_error
            candidate = {
                "road_id": road.id,
                "lane_id": lane.id,
                "lane_width_m": float(lane.width),
                "center_distance_m": match["distance_m"],
                "path_index": match["index"],
                "heading_error_deg": (
                    None
                    if heading_error is None
                    else math.degrees(heading_error)
                ),
                "score": score,
                "lane": lane,
            }
            if best is None or candidate["score"] < best["score"]:
                best = candidate
    return best


def _lane_border_margin(lane, x: float, y: float, car_width: float):
    left = _nearest_on_polyline(lane.left_border.path_points, x, y)
    right = _nearest_on_polyline(lane.right_border.path_points, x, y)
    distances = [
        item["distance_m"]
        for item in (left, right)
        if item is not None
    ]
    if not distances:
        return None, None, None
    left_m = None if left is None else left["distance_m"]
    right_m = None if right is None else right["distance_m"]
    geometric_margin = min(distances) - 0.5 * car_width
    return left_m, right_m, geometric_margin


def _trajectory_key(points):
    if len(points) < 2:
        return (len(points),)
    first, last = points[0], points[-1]
    return (
        len(points),
        round(float(first.x), 2),
        round(float(first.y), 2),
        round(float(last.x), 2),
        round(float(last.y), 2),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="环形转弯只读探针")
    parser.add_argument(
        "--sample-seconds",
        type=float,
        default=5.0,
        help="连接后保持制动采样的场景时间，单位 s",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=600,
        help="安全帧数上限",
    )
    args = parser.parse_args()
    if args.sample_seconds <= 0.0:
        parser.error("--sample-seconds 必须大于 0")
    if args.max_frames <= 0:
        parser.error("--max-frames 必须大于 0")

    print("等待场景连接...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    route = list(static.route)
    roads = list(static.roads)
    sub_scenes = list(static.sub_scenes)
    print(
        f"路线点={len(route)} 道路={len(roads)} "
        f"子场景={[s.name for s in sub_scenes]}",
        flush=True,
    )
    print(
        "静态起点="
        f"{None if not sub_scenes else sub_scenes[0].start_point} "
        "静态终点="
        f"{None if not sub_scenes else sub_scenes[-1].end_point}",
        flush=True,
    )

    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"ring_turn_probe_{stamp}.csv"
    json_path = log_dir / f"ring_turn_probe_{stamp}.json"
    csv_file = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(
        [
            "frame",
            "sim_time",
            "time_limit",
            "sub_scene",
            "x",
            "y",
            "yaw",
            "vx",
            "vy",
            "speed",
            "steering",
            "route_distance",
            "route_heading_error_deg",
            "trajectory_points",
            "trajectory_distance",
            "trajectory_heading_error_deg",
            "road_id",
            "lane_id",
            "lane_width",
            "lane_center_distance",
            "left_border_distance",
            "right_border_distance",
            "lane_margin",
            "obstacle_count",
        ]
    )

    frame = 0
    ending = "unknown"
    first_pose = None
    last_pose = None
    trajectory_samples = []
    seen_trajectory_keys = set()
    try:
        for frame, (msg, _camera_frames) in enumerate(
            api.main_loop(),
            start=1,
        ):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            x = float(pose.pos_x)
            y = float(pose.pos_y)
            yaw_deg = float(pose.ori_z)
            pose_heading = math.radians(-yaw_deg)
            if first_pose is None:
                first_pose = {
                    "x": x,
                    "y": y,
                    "yaw_deg": yaw_deg,
                    "sim_time": float(status.used_time),
                }
            last_pose = {
                "x": x,
                "y": y,
                "yaw_deg": yaw_deg,
                "sim_time": float(status.used_time),
            }

            route_match = _nearest_on_polyline(route, x, y)
            route_distance = (
                None if route_match is None else route_match["distance_m"]
            )
            route_heading_error = None
            if route_match is not None and route_match["heading_rad"] is not None:
                route_heading_error = math.degrees(
                    abs(_wrap(route_match["heading_rad"] - pose_heading))
                )

            trajectory = list(getattr(msg, "trajectory", []) or [])
            trajectory_key = _trajectory_key(trajectory)
            if trajectory and trajectory_key not in seen_trajectory_keys:
                seen_trajectory_keys.add(trajectory_key)
                trajectory_samples.append(
                    {
                        "sim_time": float(status.used_time),
                        "points": [
                            [float(p.x), float(p.y), float(p.z)]
                            for p in trajectory
                        ],
                    }
                )
            trajectory_match = _nearest_on_polyline(trajectory, x, y)
            trajectory_distance = (
                None
                if trajectory_match is None
                else trajectory_match["distance_m"]
            )
            trajectory_heading_error = None
            if (
                trajectory_match is not None
                and trajectory_match["heading_rad"] is not None
            ):
                trajectory_heading_error = math.degrees(
                    abs(
                        _wrap(
                            trajectory_match["heading_rad"] - pose_heading
                        )
                    )
                )

            lane_match = _nearest_lane(roads, x, y, pose_heading)
            left_border_distance = None
            right_border_distance = None
            lane_margin = None
            if lane_match is not None:
                (
                    left_border_distance,
                    right_border_distance,
                    lane_margin,
                ) = _lane_border_margin(
                    lane_match["lane"],
                    x,
                    y,
                    float(car.width),
                )

            writer.writerow(
                [
                    frame,
                    round(float(status.used_time), 4),
                    round(float(status.time_limit), 4),
                    status.sub_scene_name,
                    round(x, 4),
                    round(y, 4),
                    round(yaw_deg, 3),
                    round(float(pose.vel_x), 4),
                    round(float(pose.vel_y), 4),
                    round(float(car.speed), 4),
                    round(float(car.steering), 4),
                    None if route_distance is None else round(route_distance, 4),
                    (
                        None
                        if route_heading_error is None
                        else round(route_heading_error, 3)
                    ),
                    len(trajectory),
                    (
                        None
                        if trajectory_distance is None
                        else round(trajectory_distance, 4)
                    ),
                    (
                        None
                        if trajectory_heading_error is None
                        else round(trajectory_heading_error, 3)
                    ),
                    None if lane_match is None else lane_match["road_id"],
                    None if lane_match is None else lane_match["lane_id"],
                    (
                        None
                        if lane_match is None
                        else round(lane_match["lane_width_m"], 4)
                    ),
                    (
                        None
                        if lane_match is None
                        else round(lane_match["center_distance_m"], 4)
                    ),
                    (
                        None
                        if left_border_distance is None
                        else round(left_border_distance, 4)
                    ),
                    (
                        None
                        if right_border_distance is None
                        else round(right_border_distance, 4)
                    ),
                    None if lane_margin is None else round(lane_margin, 4),
                    len(list(getattr(msg, "obstacles", []) or [])),
                ]
            )

            api.set_vehicle_control(
                VehicleControl(gear=GearMode.PARKING, brake=1.0)
            )

            if frame % 10 == 1:
                print(
                    f"[{status.used_time:6.1f}s] "
                    f"pos=({x:7.2f},{y:7.2f}) yaw={yaw_deg:7.2f} "
                    f"v={float(car.speed):5.2f} "
                    f"route_d={route_distance} "
                    f"traj={len(trajectory)} "
                    f"lane={None if lane_match is None else lane_match['lane_id']} "
                    f"margin={lane_margin}",
                    flush=True,
                )

            if (
                float(status.used_time) >= args.sample_seconds
                or frame >= args.max_frames
            ):
                ending = "sample_complete"
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

    static_payload = static.model_dump(mode="json")
    summary = {
        "scene": [s.name for s in sub_scenes],
        "ending": ending,
        "frames": frame,
        "sample_seconds": args.sample_seconds,
        "first_pose": first_pose,
        "last_pose": last_pose,
        "static": static_payload,
        "trajectory_samples": trajectory_samples,
        "csv": str(csv_path),
    }
    json_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(
        "SUMMARY "
        + json.dumps(
            {
                "scene": summary["scene"],
                "ending": ending,
                "frames": frame,
                "route_points": len(route),
                "roads": len(roads),
                "trajectory_samples": len(trajectory_samples),
                "json": str(json_path),
                "csv": str(csv_path),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
