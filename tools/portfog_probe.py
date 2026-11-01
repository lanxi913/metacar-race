# -*- coding: utf-8 -*-
"""港口迷雾交互探测：对已知物体发 interact，看平台怎么回答（距离/阶段/权限）。"""
from __future__ import annotations

import json
import time
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

from portfog_run import Link  # noqa: E402


def main() -> int:
    link = Link()
    print("waiting 9100 ...", flush=True)
    if not link.wait_ready(180):
        print("!! no server", flush=True)
        return 2

    st = link.state()
    objs = [o.get("object_id") for o in (st.get("shared_task_objects") or [])]
    print("phase:", st.get("phase"), "objects:", objs, flush=True)

    for vid in ("Car_C", "Car_A"):
        o = link.obs(vid)
        p = o.get("position") or {}
        print(f"{vid}: ({p.get('x'):.1f},{p.get('y'):.1f}) role={o.get('role')} "
              f"phase={o.get('phase')} visible={len(o.get('visible_objects') or [])}", flush=True)

    print("\n== interact 试探 ==", flush=True)
    pairs = [
        ("Car_C", "submit_key", "Signal_A"),
        ("Car_C", "report_observation", "Signal_A"),
        ("Car_C", "scan", "Obstacle_1"),
        ("Car_A", "request_entry", "Intersection"),
        ("Car_A", "deliver", "Berth_A"),
        ("Car_C", "release", "Intersection"),
    ]
    for vid, act, oid in pairs:
        r = link.interact(act, oid, vid)
        print(f"{vid:6s} {act:20s} {oid:12s} -> ok={r.get('ok')} {str(r.get('message'))[:90]}", flush=True)
        time.sleep(0.2)

    print("\n== 心跳 5 秒后再看状态 ==", flush=True)
    for _ in range(10):
        link.ctrl("Car_C", throttle=0.0, brake=1.0, steering=0.0, gear=1)
        time.sleep(0.5)
    sc = link.score()
    print("elapsed:", sc.get("elapsed_seconds"), "total:", sc.get("total_score"),
          "signals:", sc.get("signals_verified"), "/", sc.get("signals_total"), flush=True)
    print("events:", json.dumps(link.ask({"command": "get_events"}).get("events")[-4:],
                                ensure_ascii=False)[:600], flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
