# -*- coding: utf-8 -*-
"""拉取港口迷雾的完整观测/静态数据/规划问题，存盘并打印要点。"""
import json
import socket

OUT = r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\docs\portfog_obs.json"


def ask(payload: dict, wait: float = 5.0):
    s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
    s.settimeout(wait)
    try:
        s.sendall((json.dumps(payload) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        return json.loads(buf.split(b"\n")[0].decode("utf-8", "replace"))
    finally:
        s.close()


r = ask({"command": "get_observation", "vehicle_id": "Car_C"})
json.dump(r, open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)

obs = r.get("observation") or {}
print("vehicle_id:", obs.get("vehicle_id"), "role:", obs.get("role"), "phase:", obs.get("phase"))
print("position:", obs.get("position"))
print("signals verified/total:", obs.get("verified_signal_count"), obs.get("total_signal_count"))
print("destinations unlocked/total:", obs.get("unlocked_destination_count"),
      obs.get("total_destination_count"))
print("roadblock_active:", obs.get("roadblock_active"), "intersection_owner:", obs.get("intersection_owner"))
print("\nvisible_objects:", len(obs.get("visible_objects") or []))
for o in (obs.get("visible_objects") or [])[:10]:
    print("  ", json.dumps(o, ensure_ascii=False)[:220])

ssd = r.get("scene_static_data") or {}
print("\nscene_static_data.vehicles:", len(ssd.get("vehicles") or []))
for v in (ssd.get("vehicles") or []):
    print("  ", json.dumps(v, ensure_ascii=False)[:200])
print("scene_static_data.static_objects:", len(ssd.get("static_objects") or []))
for o in (ssd.get("static_objects") or [])[:12]:
    print("  ", json.dumps(o, ensure_ascii=False)[:220])

pp = r.get("planning_problem") or {}
print("\nplanning_problem keys:", list(pp.keys()))
print("map_min/max:", pp.get("map_min"), pp.get("map_max"))
for k in ("road_nodes", "road_edges", "road_boundaries"):
    print(f"{k}: {len(pp.get(k) or [])}")
print("\nsaved ->", OUT)
