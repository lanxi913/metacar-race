# -*- coding: utf-8 -*-
"""验证 set_vehicle_control 能驱动车辆：发油门 -> 读位置变化。"""
import json
import socket
import time


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


def pos(vid="Car_C"):
    r = ask({"command": "get_observation", "vehicle_id": vid})
    p = (r.get("observation") or {}).get("position") or {}
    return p.get("x"), p.get("y")


print("start pos:", pos())
for gear in (1, 2):
    r = ask({"command": "set_vehicle_control", "vehicle_id": "Car_C", "gear": gear,
             "throttle": 0.30, "brake": 0.0, "steering": 0.0})
    print(f"gear={gear} ->", r.get("ok"), r.get("message"),
          "state:", (r.get("vehicle_state") or {}).get("speed_mps"))
    time.sleep(1.5)
    print("   pos after 1.5s:", pos())
