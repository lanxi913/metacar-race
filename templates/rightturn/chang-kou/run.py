# -*- coding: utf-8 -*-
"""直角转弯 · chang-kou 基线（目标 110）。"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.tracking import PurePursuit  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="直角转弯")
    parser.add_argument("--throttle", type=float, default=0.01)
    parser.add_argument("--lookahead", type=float, default=3.0)
    args = parser.parse_args()
    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    pursuit = PurePursuit(static.route, lookahead=args.lookahead)
    print(f"子场景={[s.name for s in static.sub_scenes]} route={len(static.route)}", flush=True)
    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            steering = pursuit.steering(pose.pos_x, pose.pos_y, pose.ori_z)
            endpoint = msg.scene_status.end_point
            if endpoint is not None:
                left = ((endpoint.x - pose.pos_x) ** 2 + (endpoint.y - pose.pos_y) ** 2) ** 0.5
                if left <= 1.5:
                    steering = 0.0
            vc = VehicleControl(
                gear=GearMode.DRIVE,
                throttle=max(0.0, min(1.0, args.throttle)),
                brake=0.0,
                steering=steering,
            )
            api.set_vehicle_control(vc)
            if frame % 15 == 1:
                print(
                    f"[{msg.scene_status.used_time:6.1f}s] "
                    f"pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                    f"yaw={pose.ori_z:6.1f} v={msg.main_vehicle.speed:5.2f}",
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
