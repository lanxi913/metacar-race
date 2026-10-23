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

from metacar import GearMode, SceneAPI, VehicleControl

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from metacar_race.config import UTURN_LOOKAHEAD_M, UTURN_THROTTLE_COMMAND
from metacar_race.control.tracking import PurePursuit


def _endpoint_distance(endpoint, x: float, y: float) -> float | None:
    if endpoint is None:
        return None
    return math.hypot(float(endpoint.x) - x, float(endpoint.y) - y)


def main() -> int:
    parser = argparse.ArgumentParser(description="U 型弯低速实时轨迹循迹")
    parser.add_argument("--throttle", type=float, default=UTURN_THROTTLE_COMMAND)
    parser.add_argument("--lookahead", type=float, default=UTURN_LOOKAHEAD_M)
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
    static_pursuit = PurePursuit(static_route, lookahead=args.lookahead)
    endpoint = static.sub_scenes[-1].end_point if static.sub_scenes else None
    print(
        f"子场景: {[s.name for s in static.sub_scenes]}  "
        f"static_route={len(static_route)}  lookahead={args.lookahead:.2f}  "
        f"throttle={args.throttle:.3f}  终点={endpoint}",
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
                pursuit = PurePursuit(trajectory, lookahead=args.lookahead)
                path_source = "trajectory"
            else:
                pursuit = static_pursuit
                path_source = "static_route"
            steering = pursuit.steering(x, y, yaw)

            left = _endpoint_distance(endpoint, x, y)
            if left is not None and left <= 1.5:
                steering = 0.0

            vc = VehicleControl(
                gear=GearMode.DRIVE,
                throttle=args.throttle,
                brake=0.0,
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
