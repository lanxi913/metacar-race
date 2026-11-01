# -*- coding: utf-8 -*-
"""VLA 场景静态数据全字段探针：buildings / regions / obstacles 都打出来。"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402


def main() -> int:
    api = SceneAPI()
    api.connect()
    st = api.get_scene_static_data()
    ext = st.vla_extension
    print("== sub_scenes ==", flush=True)
    for s in (getattr(st, "sub_scenes", []) or []):
        print("  ", getattr(s, "name", "?"), "start=", getattr(s, "start_point", None),
              "end=", getattr(s, "end_point", None), flush=True)
    print("== buildings ==", flush=True)
    for b in (getattr(ext, "buildings", []) or []):
        d = b.model_dump() if hasattr(b, "model_dump") else dict(b)
        print("  ", d, flush=True)
    print("== regions ==", flush=True)
    for r in (getattr(ext, "regions", []) or []):
        d = r.model_dump() if hasattr(r, "model_dump") else dict(r)
        print("  ", d, flush=True)

    n = 0
    for msg, cams in api.main_loop():
        n += 1
        if n in (5, 60):
            print(f"== frame {n}: obstacles ==", flush=True)
            for o in msg.obstacles:
                extra = getattr(o, "extra_info", None)
                print("  ", o.type, (round(float(o.pos_x), 2), round(float(o.pos_y), 2)),
                      "ori=", round(float(o.ori_z), 1),
                      "len=", round(float(o.length), 2), "wid=", round(float(o.width), 2),
                      "extra=", extra, flush=True)
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        if n > 70:
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
