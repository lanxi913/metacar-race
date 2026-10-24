# -*- coding: utf-8 -*-
r"""油门与刹车开环标定。

不要在停车控制器里盲调参数。本工具每次只测试一组固定纵向量：

油门响应：
    .venv\Scripts\python.exe -u tools\calibrate_longitudinal.py ^
        --mode throttle --throttle 0.05 --hold-seconds 2 --brake 0.5

制动距离：
    .venv\Scripts\python.exe -u tools\calibrate_longitudinal.py ^
        --mode brake --start-throttle 0.10 --target-speed 3 --brake 0.30

脚本固定 steering=0，用于直线场景隔离纵向控制。测试会提前刹停，
所以平台分数不是本次目的；请以日志中的响应和距离为准。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

from metacar import GearMode, SceneAPI, VehicleControl

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def motion_distance(samples: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(samples, samples[1:])
    )


def wait_for_stop(
    speed: float, stopped_since: float | None, sim_time: float
) -> tuple[bool, float | None]:
    # 平台静止时 car.speed 仍会在 0 附近小幅正负跳变。
    if abs(speed) <= 0.30:
        if stopped_since is None:
            stopped_since = sim_time
        if sim_time - stopped_since >= 0.8:
            return True, stopped_since
    else:
        stopped_since = None
    return False, stopped_since


def main() -> int:
    parser = argparse.ArgumentParser(description="油门与刹车开环标定")
    parser.add_argument("--mode", choices=("throttle", "brake"), required=True)
    parser.add_argument("--throttle", type=float, default=0.05)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--start-throttle", type=float, default=0.10)
    parser.add_argument("--target-speed", type=float, default=3.0)
    parser.add_argument("--brake", type=float, default=0.30)
    parser.add_argument("--max-seconds", type=float, default=25.0)
    parser.add_argument("--safety-distance", type=float, default=5.0)
    args = parser.parse_args()

    args.throttle = clamp(args.throttle, 0.0, 1.0)
    args.start_throttle = clamp(args.start_throttle, 0.0, 1.0)
    args.brake = clamp(args.brake, 0.0, 1.0)
    args.target_speed = max(0.1, args.target_speed)

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    fallback_end = static.sub_scenes[-1].end_point if static.sub_scenes else None
    endpoint = fallback_end
    print(
        f"子场景: {[s.name for s in static.sub_scenes]}  "
        f"route={len(static.route)}  终点={fallback_end}",
        flush=True,
    )
    print(
        f"标定模式={args.mode} throttle={args.throttle} "
        f"start_throttle={args.start_throttle} "
        f"target_speed={args.target_speed} brake={args.brake}",
        flush=True,
    )

    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"longitudinal_{args.mode}_{stamp}.csv"
    json_path = log_dir / f"longitudinal_{args.mode}_{stamp}.json"
    csv_file = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(
        [
            "frame",
            "sim_time",
            "elapsed",
            "x",
            "y",
            "yaw",
            "speed",
            "throttle",
            "brake",
            "phase",
            "along_track",
            "a_frame",
            "a_0_5s",
            "a_1s",
        ]
    )

    phase = "accelerate"
    phase_start = 0.0
    brake_start = None
    brake_start_xy = None
    brake_start_speed = None
    initial_xy = None
    previous_xy = None
    previous_speed = 0.0
    previous_sim_time = None
    stopped_since = None
    motion_samples: list[tuple[float, float]] = []
    speed_history: list[tuple[float, float]] = []
    first_motion_time = None
    first_brake_command_time = None
    ending = "unknown"
    final_speed = 0.0
    frame = 0
    test_start_time = None

    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            endpoint = status.end_point or endpoint or fallback_end
            sim_time = float(status.used_time)
            x = float(pose.pos_x)
            y = float(pose.pos_y)
            speed = float(car.speed)
            final_speed = speed

            if previous_sim_time is None:
                dt = 1.0 / 30.0
                phase_start = sim_time
                test_start_time = sim_time
            else:
                dt = max(sim_time - previous_sim_time, 1e-3)
            elapsed = sim_time - phase_start

            if initial_xy is None:
                initial_xy = (x, y)
            if previous_xy is not None:
                motion_samples.append(previous_xy)
            previous_xy = (x, y)
            speed_history.append((sim_time, speed))
            if first_motion_time is None and abs(speed) > 0.10:
                first_motion_time = sim_time

            along_track = None
            if endpoint is not None:
                dx = float(endpoint.x) - x
                dy = float(endpoint.y) - y
                yaw = -math.radians(float(pose.ori_z))
                along_track = dx * math.cos(yaw) + dy * math.sin(yaw)
                if (
                    phase == "accelerate"
                    and along_track <= args.safety_distance
                ):
                    phase = "brake"
                    phase_start = sim_time
                    brake_start = sim_time
                    brake_start_xy = (x, y)
                    brake_start_speed = speed

            if phase == "accelerate":
                if args.mode == "throttle":
                    if elapsed >= args.hold_seconds:
                        phase = "brake"
                        phase_start = sim_time
                        brake_start = sim_time
                        brake_start_xy = (x, y)
                        brake_start_speed = speed
                    throttle = args.throttle
                    brake = 0.0
                else:
                    if speed >= args.target_speed:
                        phase = "brake"
                        phase_start = sim_time
                        brake_start = sim_time
                        brake_start_xy = (x, y)
                        brake_start_speed = speed
                    throttle = args.start_throttle
                    brake = 0.0

            elif phase == "brake":
                if first_brake_command_time is None:
                    first_brake_command_time = sim_time
                throttle = 0.0
                brake = args.brake
                stopped, stopped_since = wait_for_stop(
                    speed, stopped_since, sim_time
                )
                if stopped:
                    phase = "park"
            elif phase == "park":
                throttle = 0.0
                brake = 1.0
            else:
                throttle = 0.0
                brake = 0.0

            vc = VehicleControl(
                gear=(
                    GearMode.PARKING
                    if phase == "park"
                    else GearMode.DRIVE
                ),
                throttle=float(throttle),
                brake=float(brake),
                steering=0.0,
            )
            api.set_vehicle_control(vc)

            a_frame = 0.0
            if previous_sim_time is not None and dt >= 0.015:
                a_frame = (speed - previous_speed) / dt

            def window_accel(seconds: float) -> float:
                candidates = [
                    (t, v)
                    for t, v in speed_history
                    if sim_time - t >= seconds
                ]
                if not candidates:
                    return 0.0
                t0, v0 = candidates[-1]
                return (speed - v0) / max(sim_time - t0, 1e-3)

            a_05 = window_accel(0.5)
            a_1 = window_accel(1.0)
            writer.writerow(
                [
                    frame,
                    round(sim_time, 4),
                    round(elapsed, 4),
                    round(x, 4),
                    round(y, 4),
                    round(float(pose.ori_z), 3),
                    round(speed, 4),
                    round(throttle, 4),
                    round(brake, 4),
                    phase,
                    None if along_track is None else round(along_track, 4),
                    round(a_frame, 4),
                    round(a_05, 4),
                    round(a_1, 4),
                ]
            )

            if frame % 10 == 1:
                along_text = (
                    "n/a" if along_track is None else f"{along_track:5.2f}"
                )
                print(
                    f"[{sim_time:6.1f}s] {phase:10s} "
                    f"v={speed:5.2f} "
                    f"th={throttle:4.2f} br={brake:4.2f} "
                    f"a05={a_05:+5.2f} a1={a_1:+5.2f} "
                    f"along={along_text}",
                    flush=True,
                )

            previous_speed = speed
            previous_sim_time = sim_time

            if phase == "park":
                ending = "stopped"
                break
            if sim_time - phase_start > args.max_seconds:
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

    if previous_xy is not None:
        motion_samples.append(previous_xy)
    stop_distance = None
    if brake_start_xy is not None and previous_xy is not None:
        stop_distance = math.hypot(
            previous_xy[0] - brake_start_xy[0],
            previous_xy[1] - brake_start_xy[1],
        )
    def max_window_accel(window_s: float) -> float:
        values = []
        for start, (t0, v0) in enumerate(speed_history):
            for t1, v1 in speed_history[start + 1:]:
                delta_t = t1 - t0
                if delta_t >= window_s:
                    values.append(abs((v1 - v0) / delta_t))
                    break
        return max(values, default=0.0)

    summary = {
        "mode": args.mode,
        "ending": ending,
        "scene": [s.name for s in static.sub_scenes],
        "route_points": len(static.route),
        "throttle": args.throttle if args.mode == "throttle" else args.start_throttle,
        "brake": args.brake,
        "target_speed": args.target_speed,
        "test_start_time": test_start_time,
        "first_motion_time": first_motion_time,
        "first_brake_command_time": first_brake_command_time,
        "brake_start_speed": (
            None if brake_start_speed is None else round(brake_start_speed, 4)
        ),
        "final_speed": round(final_speed, 4),
        "brake_distance_m": (
            None if stop_distance is None else round(stop_distance, 4)
        ),
        "total_distance_m": round(motion_distance(motion_samples), 4),
        "max_abs_accel_0_5s": round(max_window_accel(0.5), 4),
        "max_abs_accel_1s": round(max_window_accel(1.0), 4),
        "frames": frame,
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
