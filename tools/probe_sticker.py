# -*- coding: utf-8 -*-
"""探针：把「停车区违规检测」场景能拿到的东西全打一遍。

CS4 的经验是"远处不下发、贴近才有"，所以这里分三个阶段 dump：
开跑前 static、行驶中每 2 s 一次的 obstacles、以及 VLA extension 全字段。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402


def dump_obj(o, tag=""):
    fields = {}
    for name in ("id", "type", "pos_x", "pos_y", "ori_z", "length", "width", "height", "extra"):
        try:
            fields[name] = getattr(o, name)
        except Exception:
            pass
    print(f"  [{tag}] {fields}", flush=True)


def main() -> int:
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print("== static ==", flush=True)
    ext = static.vla_extension
    if ext is not None:
        for name in dir(ext):
            if name.startswith("_"):
                continue
            try:
                val = getattr(ext, name)
            except Exception:
                continue
            if callable(val):
                continue
            if isinstance(val, (list, tuple)) and len(val) > 8:
                print(f"  vla_extension.{name}: <{len(val)} items> first={val[0]}", flush=True)
            else:
                print(f"  vla_extension.{name}: {val}", flush=True)
    print("  road_points:", len(getattr(static, "road_points", []) or []), flush=True)
    print("  sub_scenes:", [getattr(s, "name", "?") for s in (getattr(static, "sub_scenes", []) or [])], flush=True)

    t0 = None
    last_dump = -99.0
    frames = 0
    try:
        for msg, cams in api.main_loop():
            frames += 1
            st = msg.scene_status
            pose = msg.pose_gnss
            x, y = float(pose.pos_x), float(pose.pos_y)
            if t0 is None:
                t0 = st.used_time
            el = st.used_time - t0
            if el - last_dump > 5.0:
                last_dump = el
                print(f"\n== t={el:5.1f}s ego=({x:.1f},{y:.1f}) obstacles={len(msg.obstacles)} ==", flush=True)
                for o in msg.obstacles:
                    dump_obj(o)
                if cams:
                    print(f"  cams={len(cams)} shape={getattr(cams[0].frame, 'shape', None)}", flush=True)
            # 慢慢往前开一点，看看远处会不会出现新对象
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.35, brake=0.0, steering=0.0)
            api.set_vehicle_control(vc)
            if el > 55.0:
                break
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:
            pass
    print("frames", frames, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
