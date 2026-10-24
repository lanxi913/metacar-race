# -*- coding: utf-8 -*-
r"""「车辆停止」标定入口。

运行顺序：
    .venv\Scripts\python.exe -u tools\calibrate_stop.py
    → 客户端点「启动场景」

本脚本不修改根目录 ``main.py``。结束后把平台成绩和 ``logs/stop_*.json``
一起记录到 Issue #3。
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

# 直接运行 tools\calibrate_stop.py 时，将仓库根目录加入模块搜索路径。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from metacar_race.config import (
    LOOKAHEAD_M,
    STOP_EXIT_GRACE_SECONDS,
    STOP_HOLD_SECONDS,
)
from metacar_race.control.stop import StopController, hold_complete


def main() -> int:
    parser = argparse.ArgumentParser(description="车辆停止标定")
    parser.add_argument("--seconds", type=float, default=100.0, help="运行时限")
    parser.add_argument(
        "--lookahead",
        type=float,
        default=LOOKAHEAD_M,
        help="纯跟踪前视距离",
    )
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    route = list(static.route)
    if len(route) < 2:
        print("route 点不足，无法循迹。", flush=True)
        return 1

    controller = StopController(route, lookahead=args.lookahead)
    fallback_end = static.sub_scenes[-1].end_point if static.sub_scenes else None
    endpoint = fallback_end
    print(
        f"子场景: {[s.name for s in static.sub_scenes]}  "
        f"路线点: {len(route)}  前视: {args.lookahead:.2f}  "
        f"初始终点: {fallback_end}",
        flush=True,
    )

    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = log_dir / f"stop_{stamp}.csv"
    json_path = log_dir / f"stop_{stamp}.json"
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
            "distance_to_end",
            "along_track",
            "target_speed",
            "throttle",
            "brake",
            "steering",
            "stopped",
            "held_seconds",
        ]
    )

    start_wall = time.time()
    previous_sim_time = None
    final_x = final_y = 0.0
    max_abs_accel = 0.0
    previous_speed = 0.0
    final_info = None
    ending = "unknown"
    frame = 0

    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            endpoint = status.end_point or endpoint or fallback_end

            if previous_sim_time is None:
                dt = 1.0 / 30.0
            else:
                dt = max(status.used_time - previous_sim_time, 1e-3)
            previous_sim_time = status.used_time

            speed = float(car.speed)
            if frame > 2 and dt >= 0.015:
                max_abs_accel = max(max_abs_accel, abs(speed - previous_speed) / dt)
            previous_speed = speed
            final_x, final_y = float(pose.pos_x), float(pose.pos_y)

            vc, info = controller.control(
                final_x,
                final_y,
                float(pose.ori_z),
                speed,
                dt,
                endpoint,
            )
            final_info = info
            api.set_vehicle_control(vc)
            writer.writerow(
                [
                    frame,
                    round(status.used_time, 4),
                    round(final_x, 4),
                    round(final_y, 4),
                    round(float(pose.ori_z), 3),
                    round(speed, 4),
                    round(info.distance_to_end, 4),
                    round(info.along_track, 4),
                    round(info.target_speed, 4),
                    round(vc.throttle, 4),
                    round(vc.brake, 4),
                    round(vc.steering, 4),
                    info.stopped,
                    round(info.held_seconds, 3),
                ]
            )

            if frame % 10 == 1:
                print(
                    f"[{status.used_time:6.1f}s] "
                    f"pos=({final_x:7.2f},{final_y:7.2f}) "
                    f"v={speed:5.2f} "
                    f"d={info.distance_to_end:5.2f} "
                    f"along={info.along_track:5.2f} "
                    f"vt={info.target_speed:5.2f} "
                    f"th={vc.throttle:4.2f} br={vc.brake:4.2f} "
                    f"st={vc.steering:+5.2f} "
                    f"stop={info.stopped} hold={info.held_seconds:4.1f}s",
                    flush=True,
                )

            if (
                hold_complete(info)
                and info.held_seconds >= STOP_HOLD_SECONDS + STOP_EXIT_GRACE_SECONDS
            ):
                ending = "hold_complete"
                break
            if time.time() - start_wall > args.seconds:
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
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass

    final_error = None
    if endpoint is not None:
        final_error = math.hypot(
            float(endpoint.x) - final_x,
            float(endpoint.y) - final_y,
        )

    summary = {
        "scene": [s.name for s in static.sub_scenes],
        "ending": ending,
        "stopped": bool(final_info and final_info.stopped),
        "held_seconds": round(final_info.held_seconds, 3) if final_info else 0.0,
        "target_x": round(float(endpoint.x), 4) if endpoint else None,
        "target_y": round(float(endpoint.y), 4) if endpoint else None,
        "final_x": round(final_x, 4),
        "final_y": round(final_y, 4),
        "position_error_m": round(final_error, 4) if final_error is not None else None,
        "max_abs_accel_from_signal": round(max_abs_accel, 4),
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
