# -*- coding: utf-8 -*-
r"""环形转弯入口：实时轨迹循迹、曲率预判限速和自适应纵向控制。

运行顺序：
    .venv\Scripts\python.exe -u tools\ring_turn_run.py
    → 终端出现“等待场景连接...”后，在客户端启动“环形转弯”场景
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

from metacar_race.control.ring_turn import (  # noqa: E402
    RingTurnController,
    estimate_lane_width,
)


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="环形转弯")
    parser.add_argument(
        "--finish-hold-seconds",
        type=float,
        default=1.5,
        help="接近终点后继续保持前进的时长，单位 s",
    )
    args = parser.parse_args()

    print("等待场景连接...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    route = list(static.route)
    roads = list(static.roads)
    if len(route) < 3:
        print("静态路线点不足，无法循迹。", flush=True)
        return 1

    endpoints = [
        scene.end_point
        for scene in static.sub_scenes
        if scene.end_point is not None
    ]
    fallback_endpoint = endpoints[-1] if endpoints else route[-1]
    print(
        f"子场景={[s.name for s in static.sub_scenes]} "
        f"路线点={len(route)} 道路={len(roads)} "
        f"终点={fallback_endpoint}",
        flush=True,
    )

    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"ring_turn_{stamp}.csv"
    json_path = log_dir / f"ring_turn_{stamp}.json"
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
            "target_speed",
            "target_speed_raw",
            "required_speed",
            "time_budget_left",
            "route_index",
            "remaining",
            "cross_track",
            "curvature",
            "lookahead",
            "path_source",
            "finish_mode",
            "shift_allowed",
            "shift_active",
            "startup_active",
            "throttle",
            "brake",
            "steering",
            "trajectory_points",
        ]
    )

    controller = None
    lane_width = 0.0
    frame = 0
    ending = "unknown"
    finish_started = None
    first_pose = None
    last_pose = None
    last_result = None
    error_text = None
    previous_speed = None
    previous_sim_time = None
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
            yaw = float(pose.ori_z)
            speed = float(car.speed)
            sim_time = float(status.used_time)
            time_limit = float(status.time_limit)
            trajectory = list(getattr(msg, "trajectory", []) or [])
            if previous_speed is None or previous_sim_time is None:
                a_now = 0.0
            else:
                dt_sample = max(sim_time - previous_sim_time, 1e-3)
                a_now = (speed - previous_speed) / dt_sample
            previous_speed = speed
            previous_sim_time = sim_time

            if controller is None:
                lane_width = estimate_lane_width(
                    roads,
                    x,
                    y,
                    fallback=float(car.width) * 1.8,
                )
                controller = RingTurnController(
                    route,
                    float(car.length),
                    float(car.width),
                    lane_width,
                )
                print(
                    f"车辆尺寸=({float(car.length):.2f},"
                    f"{float(car.width):.2f}) "
                    f"车道宽度={lane_width:.2f}",
                    flush=True,
                )

            result = controller.compute(
                x,
                y,
                yaw,
                speed,
                sim_time,
                time_limit,
                dynamic_path=trajectory,
                a_now=a_now,
            )
            last_result = result
            if first_pose is None:
                first_pose = {
                    "x": round(x, 4),
                    "y": round(y, 4),
                    "yaw": round(yaw, 3),
                    "sim_time": round(sim_time, 4),
                }
            last_pose = {
                "x": round(x, 4),
                "y": round(y, 4),
                "yaw": round(yaw, 3),
                "sim_time": round(sim_time, 4),
            }

            throttle = result.throttle
            brake = result.brake
            steering = result.steering
            if result.finish_mode:
                if finish_started is None:
                    finish_started = sim_time
                if sim_time - finish_started >= args.finish_hold_seconds:
                    ending = "finish_hold_complete"
                    break
                # 不主动驻车，低速继续向前，让平台完成到达判定。
                brake = 0.0

            api.set_vehicle_control(
                VehicleControl(
                    gear=GearMode.DRIVE,
                    throttle=throttle,
                    brake=brake,
                    steering=steering,
                )
            )
            writer.writerow(
                [
                    frame,
                    round(sim_time, 4),
                    round(x, 4),
                    round(y, 4),
                    round(yaw, 3),
                    round(speed, 4),
                    round(result.target_speed_ms, 4),
                    round(result.target_speed_filtered_ms, 4),
                    round(result.required_speed_ms, 4),
                    round(result.time_budget_left_s, 4),
                    result.route_index,
                    round(result.remaining_m, 4),
                    round(result.cross_track_m, 4),
                    round(result.curvature_per_m, 6),
                    round(result.lookahead_m, 4),
                    result.path_source,
                    int(result.finish_mode),
                    int(result.shift_assist_allowed),
                    int(result.shift_assist_active),
                    int(result.startup_active),
                    round(throttle, 5),
                    round(brake, 5),
                    round(steering, 5),
                    len(trajectory),
                ]
            )

            if frame % 15 == 1:
                print(
                    f"[{sim_time:6.1f}s] "
                    f"pos=({x:7.2f},{y:7.2f}) yaw={yaw:7.2f} "
                    f"v={speed:5.2f} vt={result.target_speed_ms:5.2f} "
                    f"idx={result.route_index:3d} "
                    f"left={result.remaining_m:6.1f} "
                    f"cross={result.cross_track_m:5.2f} "
                    f"ld={result.lookahead_m:4.2f} "
                    f"src={result.path_source} "
                    f"shift={int(result.shift_assist_active)} "
                    f"th={throttle:5.4f} br={brake:5.3f} "
                    f"st={steering:+5.2f}",
                    flush=True,
                )

            if sim_time >= time_limit - 0.5:
                ending = "time_limit"
                break
    except KeyboardInterrupt:
        ending = "interrupt"
    except Exception as exc:  # noqa: BLE001
        ending = "error"
        error_text = f"{type(exc).__name__}: {exc}"
        print(f"运行异常：{error_text}", flush=True)
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

    endpoint_distance = None
    if last_pose is not None and fallback_endpoint is not None:
        endpoint_distance = math.hypot(
            float(fallback_endpoint.x) - last_pose["x"],
            float(fallback_endpoint.y) - last_pose["y"],
        )
    summary = {
        "scene": [s.name for s in static.sub_scenes],
        "ending": ending,
        "error": error_text,
        "frames": frame,
        "lane_width_m": round(lane_width, 4),
        "first_pose": first_pose,
        "last_pose": last_pose,
        "endpoint_distance_m": (
            None if endpoint_distance is None else round(endpoint_distance, 4)
        ),
        "last_control": (
            None if last_result is None else last_result.__dict__
        ),
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
                "error": error_text,
                "frames": frame,
                "endpoint_distance_m": summary["endpoint_distance_m"],
                "json": str(json_path),
                "csv": str(csv_path),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 1 if ending == "error" else 0


if __name__ == "__main__":
    raise SystemExit(main())
