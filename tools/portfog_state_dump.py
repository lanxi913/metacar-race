# -*- coding: utf-8 -*-
"""打印 task_state 里所有共享物体（含不确定区域），并测试持续心跳能否让车动。"""
import json
import socket
import time

OUT = r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\docs\portfog_taskstate.json"


def ask(payload: dict, wait: float = 4.0):
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


r = ask({"command": "get_task_state"})
ts = r.get("task_state") or {}
json.dump(ts, open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
print("phase:", ts.get("phase"))
for o in ts.get("shared_task_objects") or []:
    reg = o.get("uncertainty_region") or {}
    mn, mx = reg.get("minimum"), reg.get("maximum")
    center = None
    if mn and mx:
        center = ((mn.get("x", 0) + mx.get("x", 0)) / 2, (mn.get("y", 0) + mx.get("y", 0)) / 2)
    print(f"  {o.get('object_id'):12s} {o.get('object_type'):22s} lvl={o.get('information_level'):10s} "
          f"state={o.get('state')!r} region={reg.get('shape')} center={center}")

print("\n== 心跳驱动测试（每 0.4 s 发一次）==")
p0 = (ask({"command": "get_observation", "vehicle_id": "Car_C"}).get("observation") or {}).get("position")
print(" start:", p0.get("x"), p0.get("y"))
for i in range(8):
    ask({"command": "set_vehicle_control", "vehicle_id": "Car_C", "gear": 1,
         "throttle": 0.35, "brake": 0.0, "steering": 0.0})
    time.sleep(0.4)
p1 = (ask({"command": "get_observation", "vehicle_id": "Car_C"}).get("observation") or {}).get("position")
print(" after:", p1.get("x"), p1.get("y"))
d = ((p1.get("x", 0) - p0.get("x", 0)) ** 2 + (p1.get("y", 0) - p0.get("y", 0)) ** 2) ** 0.5
print(f" moved {d:.3f} m")
