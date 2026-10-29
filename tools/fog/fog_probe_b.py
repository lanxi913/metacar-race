# -*- coding: utf-8 -*-
"""Berth_B observation probe: why does the 2nd observer never get a token?

Drives Car_B to the berth, then walks a second car through candidate
observation stands, dumping the full visible_objects payload each time.
"""
import argparse
import json
import math
import os
import time

from fog_map import RoadGraph, connect_retry, to_plain
from fog_run3 import pose_of, WHEELBASE_RATIO
from fog_mission import (Ctx, drive_polyline, hold_still, level_of, nav_path,
                         _report)

BERTH = (834.47, -65.72)
CANDIDATES = [
    ("c1_corridor_north", (838.5, -57.0)),
    ("c2_rect_north", (833.5, -58.0)),
    ("c3_east_5m", (839.8, -65.0)),
    ("c4_northwest", (830.0, -58.5)),
    ("c5_west_8m", (826.5, -65.7)),
    ("c6_south", (838.5, -70.0)),
]


def dump_obs(ctx, vid, tag):
    ob = to_plain(ctx.api.get_observation(vid))
    vis = ob.get("visible_objects") or []
    rows = []
    for it in vis:
        rows.append({
            "id": it.get("object_id"),
            "type": it.get("object_type"),
            "state": it.get("state"),
            "d": it.get("distance_m"),
            "tok": (it.get("observation_token") or "")[:12],
        })
    ctx.note("[probe] %s %s visible=%s" % (vid, tag, json.dumps(rows, ensure_ascii=False)))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=560.0)
    ap.add_argument("--observer", default="Car_A")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    api = connect_retry()
    log = open(os.path.join(args.out, "run.log"), "w", encoding="utf-8")
    t0 = time.time()

    def note(msg):
        line = "[%6.1f] %s" % (time.time() - t0, msg)
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

    static = to_plain(api.get_scene_static_data())
    graph = RoadGraph(static)
    specs = {v["vehicle_id"]: v for v in static.get("vehicles", [])}
    vids = list(api.get_vehicle_ids())
    wb = WHEELBASE_RATIO * float(specs["Car_C"].get("length_m") or 4.81)
    ctx = Ctx(api, vids, graph, wb, args.out, t0, note)
    note("[probe] up, observer=%s" % args.observer)

    # 1) Car_B -> berth slot
    pos_b = pose_of(api, "Car_B")[0]
    route_b = [(728.8, 6.0), (743.3, 4.7), (790.0, 4.7), (790.0, -30.0),
               (812.0, -40.0), (830.0, -56.0), (833.2, -62.6)]
    try:
        path_b = nav_path(ctx, pos_b, (731.0, 8.0), inflate=2.4) + route_b
        drive_polyline(ctx, "Car_B", path_b, args.seconds, v_cap=4.4)
    except Exception as exc:
        note("[probe] Car_B drive failed %s" % exc)
    hold_still(ctx, "Car_B", seconds=2.0)
    dump_obs(ctx, "Car_B", "at_berth")
    note("[probe] level_B = %s" % level_of(ctx, "Berth_B"))

    # 2) candidate stands for the second observer
    for name, pt in CANDIDATES:
        try:
            cur = pose_of(api, args.observer)[0]
            path = nav_path(ctx, cur, pt, inflate=1.8)
            if len(path) < 2:
                path = [cur, pt]
            drive_polyline(ctx, args.observer, path, args.seconds, v_cap=3.0)
        except Exception as exc:
            note("[probe] %s -> %s failed %s" % (args.observer, name, exc))
        hold_still(ctx, args.observer, seconds=1.2)
        p, yaw, v = pose_of(api, args.observer)
        brg = math.degrees(math.atan2(BERTH[1] - p[1], BERTH[0] - p[0]))
        err = (brg - yaw + 540.0) % 360.0 - 180.0
        dist = math.hypot(BERTH[0] - p[0], BERTH[1] - p[1])
        note("[probe] %s pos=(%.1f,%.1f) d=%.2f yaw=%.1f brg=%.1f err=%.1f"
             % (name, p[0], p[1], dist, yaw, brg, err))
        dump_obs(ctx, args.observer, name)
        ok = _report(ctx, args.observer, "Berth_B", verbose=True)
        note("[probe] %s report=%s level=%s" % (name, ok, level_of(ctx, "Berth_B")))
        if ok:
            dump_obs(ctx, "Car_B", name)

    dump_obs(ctx, "Car_B", "final")
    note("[probe] DONE level=%s" % level_of(ctx, "Berth_B"))


if __name__ == "__main__":
    main()
