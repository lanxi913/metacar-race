# -*- coding: utf-8 -*-
"""港口迷雾场景探针：static 全字段 + 每帧消息全字段 + 障碍物明细。"""
from __future__ import annotations

import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402


def dump_model(tag, obj):
    fields = getattr(obj, "model_fields", None) or getattr(obj, "__fields__", {})
    print(f"== {tag}: {type(obj).__name__} fields={list(fields.keys())}", flush=True)
    for k in fields:
        try:
            v = getattr(obj, k)
        except Exception:
            continue
        s = repr(v)
        print(f"   {k}: {s[:200]}", flush=True)


def main() -> int:
    print("connecting ...", flush=True)
    api = SceneAPI()
    api.connect()
    print("connected", flush=True)
    st = api.get_scene_static_data()
    print("static ok", flush=True)
    dump_model("static", st)
    for i, r in enumerate(getattr(st, "roads", []) or []):
        print(f"   road[{i}]: {r}", flush=True)
    for i, s in enumerate(getattr(st, "sub_scenes", []) or []):
        print(f"   sub[{i}]: name={getattr(s,'name',None)} start={getattr(s,'start_point',None)} "
              f"end={getattr(s,'end_point',None)}", flush=True)
    if getattr(st, "vla_extension", None) is not None:
        print("   vla_extension:", st.vla_extension, flush=True)

    n = 0
    t0 = time.time()
    try:
        for msg, cams in api.main_loop():
            n += 1
            if n in (1, 2, 60, 180):
                print(f"\n--- frame {n} (t={time.time()-t0:.1f}s) ---", flush=True)
                dump_model("msg", msg)
                if msg.scene_status:
                    dump_model("status", msg.scene_status)
                if msg.main_vehicle:
                    dump_model("main_vehicle", msg.main_vehicle)
                print(f"   obstacles: {len(msg.obstacles)}", flush=True)
                for o in msg.obstacles[:12]:
                    print(f"     {o.type} pos=({float(o.pos_x):.1f},{float(o.pos_y):.1f}) "
                          f"ori={float(o.ori_z):.1f} len={float(o.length):.2f} "
                          f"wid={float(o.width):.2f} extra={getattr(o,'extra_info',None)}", flush=True)
                print(f"   cams: {len(cams) if cams else 0}", flush=True)
                if cams:
                    for ci, c in enumerate(cams[:4]):
                        print(f"     cam{ci} shape={getattr(c.frame,'shape',None)} "
                              f"info={getattr(c,'info',None)}", flush=True)
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if msg.scene_status and msg.scene_status.used_time > 20:
                break
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise SystemExit(1)
