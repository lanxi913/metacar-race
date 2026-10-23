# -*- coding: utf-8 -*-
"""车辆停止 · chang-kou 基线（目标 110）。"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.config import STOP_HOLD_SECONDS  # noqa: E402
from metacar_race.control.stop import StopController, hold_complete  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="车辆停止")
    parser.add_argument("--lookahead", type=float, default=6.0)
    args = parser.parse_args()
    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    controller = StopController(static.route, lookahead=args.lookahead)
    fallback_end = static.sub_scenes[-1].end_point if static.sub_scenes else None
    endpoint = fallback_end
    final = None
    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            endpoint = msg.scene_status.end_point or endpoint or fallback_end
            vc, final = controller.control(
                pose.pos_x,
                pose.pos_y,
                pose.ori_z,
                msg.main_vehicle.speed,
                1.0 / 30.0,
                endpoint,
            )
            api.set_vehicle_control(vc)
            if frame % 15 == 1:
                print(
                    f"[{msg.scene_status.used_time:6.1f}s] "
                    f"pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                    f"v={msg.main_vehicle.speed:5.2f} "
                    f"d={final.distance_to_end:5.2f} hold={final.held_seconds:4.1f}",
                    flush=True,
                )
            if hold_complete(final) and final.held_seconds >= STOP_HOLD_SECONDS + 1.0:
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
    print(
        "SUMMARY "
        + json.dumps(
            {
                "phase": "stopped" if final and final.stopped else "ended",
                "distance_to_end": (
                    None if final is None else round(final.distance_to_end, 4)
                ),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
