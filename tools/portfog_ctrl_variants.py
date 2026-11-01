# -*- coding: utf-8 -*-
"""看 set_vehicle_control 的完整回包，并试几种字段写法（speed / target_speed / nested）。"""
from __future__ import annotations

import json
import socket
import time

s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
s.settimeout(6)
buf = b""


def call(p):
    global buf
    s.sendall((json.dumps(p) + "\n").encode())
    while b"\n" not in buf:
        c = s.recv(65536)
        if not c:
            raise ConnectionError
        buf += c
    line, rest = buf.split(b"\n", 1)
    buf = rest
    return json.loads(line.decode("utf-8", "replace"))


r = call({"command": "set_vehicle_control", "vehicle_id": "Car_D", "gear": 1,
          "throttle": 0.5, "brake": 0.0, "steering": 0.0})
print("== full response ==")
print(json.dumps(r, ensure_ascii=False, indent=1)[:1500], flush=True)

print("\n== 其他字段写法 ==", flush=True)
variants = [
    ("speed", {"command": "set_vehicle_control", "vehicle_id": "Car_D", "speed": 3.0}),
    ("target_speed", {"command": "set_vehicle_control", "vehicle_id": "Car_D", "target_speed": 3.0}),
    ("nested", {"command": "set_vehicle_control", "vehicle_id": "Car_D",
                "vehicle_control": {"gear": 1, "throttle": 0.5, "brake": 0.0, "steering": 0.0}}),
    ("gear0", {"command": "set_vehicle_control", "vehicle_id": "Car_D", "gear": 0,
               "throttle": 0.5, "brake": 0.0, "steering": 0.0}),
    ("gear3", {"command": "set_vehicle_control", "vehicle_id": "Car_D", "gear": 3,
               "throttle": 0.5, "brake": 0.0, "steering": 0.0}),
]
for name, payload in variants:
    r = call(payload)
    print(f"  {name:14s} ok={r.get('ok')} msg={str(r.get('message'))[:60]} "
          f"gear={r.get('gear')} thr={r.get('throttle')}", flush=True)
    time.sleep(0.3)
s.close()
