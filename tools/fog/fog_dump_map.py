# -*- coding: utf-8 -*-
"""把场景静态数据整份导出来（路网 + 静态构件 + 车辆 + 任务对象），供离线分析地图。

输出：<out>/map_dump.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from fog_map import connect_retry, to_plain                         # noqa: E402


def xy(p):
    if isinstance(p, dict):
        return [round(float(p.get("x") or 0.0), 2), round(float(p.get("y") or 0.0), 2)]
    if isinstance(p, (list, tuple)) and len(p) >= 2:
        return [round(float(p[0]), 2), round(float(p[1]), 2)]
    return [0.0, 0.0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    api = connect_retry()
    static = to_plain(api.get_scene_static_data())
    out = {"roads": [], "static_objects": [], "vehicles": [], "task": {}}

    for r in static.get("roads") or []:
        out["roads"].append({
            "id": r.get("id"),
            "begin": xy(r.get("begin_pos")),
            "end": xy(r.get("end_pos")),
            "width": r.get("width") or r.get("width_m"),
            "type": r.get("driving_type"),
            "succ": list(r.get("successor_ids") or []),
            "pred": list(r.get("predecessor_ids") or []),
        })
    for o in static.get("static_objects") or []:
        pos = o.get("position") or o.get("center") or {}
        out["static_objects"].append({
            "id": o.get("object_id") or o.get("id"),
            "type": o.get("object_type") or o.get("type"),
            "pos": xy(pos),
            "size": [round(float(o.get(k) or 0.0), 2) for k in ("length_m", "width_m", "height_m")],
            "yaw": round(float(o.get("yaw") or 0.0), 2),
        })
    for v in static.get("vehicles") or []:
        out["vehicles"].append({"id": v.get("vehicle_id"), "len": v.get("length_m"),
                                "wid": v.get("width_m"), "vmax": v.get("maximum_speed_mps")})

    try:
        ts = to_plain(api.get_task_state())
        out["task"] = {"phase": ts.get("phase"), "owner": ts.get("intersection_owner"),
                       "roadblock_active": ts.get("roadblock_active")}
        objs = ts.get("shared_task_objects") or []
        out["task"]["objects"] = [{"id": o.get("object_id"), "type": o.get("object_type"),
                                   "pos": xy(o.get("position"))} for o in objs]
    except Exception as exc:                                   # noqa: BLE001
        out["task"] = {"error": str(exc)}

    # 车辆起点
    try:
        for vid in api.get_vehicle_ids():
            st = to_plain(api.get_vehicle_state(vid))
            pose = st.get("pose") or {}
            out.setdefault("spawn", {})[vid] = xy(pose if isinstance(pose, dict) else {})
    except Exception:                                          # noqa: BLE001
        pass

    path = os.path.join(args.out, "map_dump.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(out, fh, ensure_ascii=False, indent=1)
    print("written", path)
    print("roads=%d static_objects=%d vehicles=%d" % (len(out["roads"]),
                                                     len(out["static_objects"]),
                                                     len(out["vehicles"])))


if __name__ == "__main__":
    main()
