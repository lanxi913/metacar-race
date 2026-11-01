# -*- coding: utf-8 -*-
"""把港口迷雾的 get_task / get_vehicles 完整存盘，并打印关键字段。"""
import json
import socket

OUT = r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\docs\portfog_task.json"


def ask(payload: dict, port: int = 9100, wait: float = 5.0):
    s = socket.create_connection(("127.0.0.1", port), timeout=3)
    s.settimeout(wait)
    try:
        s.sendall((json.dumps(payload) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        line = buf.split(b"\n")[0].decode("utf-8", "replace")
        return json.loads(line)
    finally:
        s.close()


task = ask({"command": "get_task"})
veh = ask({"command": "get_vehicles"})
json.dump({"task": task, "vehicles": veh}, open(OUT, "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)

t = task.get("task", {})
print("scenario:", t.get("scenario_id"), "|", t.get("scenario_name"), "|", t.get("api_version"))
print("control_mode:", t.get("control_mode"))
print("observation_model:", t.get("observation_model"))
print("\nallowed_actions:")
for a in t.get("allowed_actions", []):
    print("  -", a)
print("\nobjectives:")
for o in t.get("objectives", []):
    print(f"  {o.get('objective_id'):24s} {o.get('objective_type'):28s} "
          f"role={o.get('required_role')} state={o.get('state')}")
    if o.get("description"):
        print(f"      {o.get('description')}")
print("\nshared_task_objects:", len(t.get("shared_task_objects") or []))
for so in (t.get("shared_task_objects") or [])[:10]:
    print("  ", json.dumps(so, ensure_ascii=False)[:200])
pp = t.get("planning_problem") or {}
print("\nplanning keys:", list(pp.keys()))
print("map:", pp.get("map_min"), pp.get("map_max"))
print("vehicles:", [(v.get("vehicle_id"), v.get("start_node"), v.get("start_position"))
                    for v in (pp.get("vehicles") or [])][:6])
print("road_nodes:", len(pp.get("road_nodes") or []), "edges:", len(pp.get("road_edges") or []),
      "boundaries:", len(pp.get("road_boundaries") or []))
print("rules:", json.dumps(pp.get("rules"), ensure_ascii=False, indent=1)[:1200])
print("\nsaved ->", OUT)
