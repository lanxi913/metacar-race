# -*- coding: utf-8 -*-
r"""U 型弯低速循迹入口。

在线示例里横向控制使用每帧的 ``sim_car_msg.trajectory`` 选目标点。U 型弯比
直角转弯更依赖连续弯道中的实时路径点，因此这里优先跟随 msg.trajectory；
如果场景没有下发实时轨迹，则回退到 static_data.route。

运行顺序：

    .venv\Scripts\python.exe -u tools\run_uturn.py
    → 客户端点「U型弯」/「U 型弯」场景的「启动场景」
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

from metacar import GearMode, SceneAPI, VehicleControl, Vector3

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from metacar_race.config import (
    UTURN_BOTTOM_BAND_M,
    UTURN_CURVE_SPEED_MS,
    UTURN_INNER_OFFSET_M,
    UTURN_LOOKAHEAD_M,
    UTURN_STRAIGHT_SPEED_MS,
    UTURN_THROTTLE_COMMAND,
    UTURN_THROTTLE_RAMP_SECONDS,
)
from metacar_race.control.tracking import PurePursuit


def _endpoint_distance(endpoint, x: float, y: float) -> float | None:
    if endpoint is None:
        return None
    return math.hypot(float(endpoint.x) - x, float(endpoint.y) - y)


def _route_bounds(route):
    xs = [float(p.x) for p in route]
    ys = [float(p.y) for p in route]
    return min(xs), max(xs), min(ys), max(ys)


def _bottom_inner_direction(route):
    """估算 U 型弯底部向内侧的方向。

    当前训练场景是起终点在上方、弯底在下方；正式不写死坐标，而是用 route
    首尾中点和 route 包围盒中心判断：从包围盒外侧极值指向首尾中点的一侧就是内侧。
    """
    _, _, min_y, max_y = _route_bounds(route)
    endpoint_mid_y = (float(route[0].y) + float(route[-1].y)) * 0.5
    center_y = (min_y + max_y) * 0.5
    return 1.0 if endpoint_mid_y >= center_y else -1.0


def _apply_bottom_inner_offset(points, bottom_y, inner_dir, offset, band):
    if offset == 0.0 or band <= 0.0:
        return points

    shifted = []
    for p in points:
        y = float(p.y)
        # 离弯底越近，内收越多；离开弯底 band 米后不偏移。
        depth = abs(y - bottom_y)
        weight = max(0.0, min(1.0, 1.0 - depth / band))
        shifted.append(
            Vector3(
                float(p.x),
                y + inner_dir * offset * weight,
                float(p.z),
            )
        )
    return shifted


def main() -> int:
    parser = argparse.ArgumentParser(description="U 型弯低速实时轨迹循迹")
    parser.add_argument("--throttle", type=float, default=UTURN_THROTTLE_COMMAND)
    parser.add_argument("--lookahead", type=float, default=UTURN_LOOKAHEAD_M)
    parser.add_argument("--ramp-seconds", type=float, default=UTURN_THROTTLE_RAMP_SECONDS)
    parser.add_argument("--straight-speed", type=float, default=UTURN_STRAIGHT_SPEED_MS)
    parser.add_argument("--curve-speed", type=float, default=UTURN_CURVE_SPEED_MS)
    parser.add_argument("--inner-offset", type=float, default=UTURN_INNER_OFFSET_M)
    parser.add_argument("--bottom-band", type=float, default=UTURN_BOTTOM_BAND_M)
    parser.add_argument("--max-seconds", type=float, default=100.0)
    args = parser.parse_args()
    args.throttle = max(0.0, min(1.0, args.throttle))
    if args.lookahead <= 0.0:
        parser.error("--lookahead 必须大于 0")

    print("连接 U 型弯场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    static_route = list(static.route)
    if len(static_route) < 2:
        print("static route 点不足，无法循迹。", flush=True)
        return 1
    _, _, route_min_y, route_max_y = _route_bounds(static_route)
    inner_dir = _bottom_inner_direction(static_route)
    bottom_y = route_min_y if inner_dir > 0.0 else route_max_y
    offset_static_route = _apply_bottom_inner_offset(
        static_route,
        bottom_y,
        inner_dir,
        args.inner_offset,
        args.bottom_band,
    )
    static_pursuit = PurePursuit(offset_static_route, lookahead=args.lookahead)
    endpoint = static.sub_scenes[-1].end_point if static.sub_scenes else None
    print(
        f"子场景: {[s.name for s in static.sub_scenes]}  "
        f"static_route={len(static_route)}  lookahead={args.lookahead:.2f}  "
        f"throttle={args.throttle:.3f}  ramp={args.ramp_seconds:.1f}s  "
        f"v_straight={args.straight_speed:.1f}  v_curve={args.curve_speed:.1f}  "
        f"inner_offset={args.inner_offset:.1f}  "
        f"终点={endpoint}",
        flush=True,
    )

    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"uturn_{stamp}.csv"
    json_path = log_dir / f"uturn_{stamp}.json"
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
            "steering",
            "left",
            "trajectory_points",
            "path_source",
        ]
    )

    frames = 0
    previous_speed = 0.0
    previous_time = None
    max_abs_accel = 0.0
    ending = "unknown"
    try:
        for frames, (msg, _camera_frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            endpoint = status.end_point or endpoint

            x = float(pose.pos_x)
            y = float(pose.pos_y)
            yaw = float(pose.ori_z)
            speed = float(car.speed)

            if previous_time is None:
                dt = 1.0 / 30.0
            else:
                dt = max(status.used_time - previous_time, 1e-3)
            if frames > 2 and dt >= 0.015:
                max_abs_accel = max(
                    max_abs_accel,
                    abs(speed - previous_speed) / dt,
                )
            previous_time = status.used_time
            previous_speed = speed

            trajectory = list(getattr(msg, "trajectory", []) or [])
            if len(trajectory) >= 2:
                trajectory = _apply_bottom_inner_offset(
                    trajectory,
                    bottom_y,
                    inner_dir,
                    args.inner_offset,
                    args.bottom_band,
                )
                pursuit = PurePursuit(trajectory, lookahead=args.lookahead)
                path_source = "trajectory"
            else:
                pursuit = static_pursuit
                path_source = "static_route"
            steering = pursuit.steering(x, y, yaw)

            left = _endpoint_distance(endpoint, x, y)
            if left is not None and left <= 1.5:
                steering = 0.0
            throttle = args.throttle
            if args.ramp_seconds > 0.0:
                throttle *= min(1.0, status.used_time / args.ramp_seconds)
            curve_factor = min(1.0, abs(steering) / 0.35)
            target_speed = (
                args.straight_speed
                - curve_factor * (args.straight_speed - args.curve_speed)
            )
            brake = 0.0
            if speed > target_speed:
                throttle = 0.0
                brake = min(0.12, 0.05 * (speed - target_speed))

            vc = VehicleControl(
                gear=GearMode.DRIVE,
                throttle=throttle,
                brake=brake,
                steering=steering,
            )
            api.set_vehicle_control(vc)
            writer.writerow(
                [
                    frames,
                    round(status.used_time, 4),
                    round(x, 4),
                    round(y, 4),
                    round(yaw, 3),
                    round(speed, 4),
                    round(steering, 4),
                    None if left is None else round(left, 3),
                    len(trajectory),
                    path_source,
                ]
            )

            if frames % 10 == 1:
                left_text = "n/a" if left is None else f"{left:6.2f}"
                print(
                    f"[{status.used_time:6.1f}s] "
                    f"pos=({x:7.2f},{y:7.2f}) "
                    f"yaw={yaw:7.2f} v={speed:5.2f} "
                    f"st={steering:+5.2f} left={left_text} "
                    f"traj={len(trajectory):3d} src={path_source}",
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
        "frames": frames,
        "throttle": args.throttle,
        "lookahead": args.lookahead,
        "max_abs_accel_from_signal": round(max_abs_accel, 4),
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
