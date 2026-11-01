# -*- coding: utf-8 -*-
"""找能被回存的字段形状：发不同写法，看回包里的 throttle/gear 是否变成我们发的值。"""
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


SHAPES = {
    "flat": {"command": "set_vehicle_control", "vehicle_id": "Car_D", "gear": 1,
             "throttle": 0.5, "brake": 0.0, "steering": 0.0},
    "nested control": {"command": "set_vehicle_control", "vehicle_id": "Car_D",
                       "control": {"gear": 1, "throttle": 0.5, "brake": 0.0, "steering": 0.0}},
    "nested vehicle_control": {"command": "set_vehicle_control", "vehicle_id": "Car_D",
                               "vehicle_control": {"gear": 1, "throttle": 0.5,
                                                   "brake": 0.0, "steering": 0.0}},
    "batch nested": {"command": "set_vehicle_controls",
                     "vehicle_controls": [{"vehicle_id": "Car_D", "gear": 1, "throttle": 0.5,
                                           "brake": 0.0, "steering": 0.0}]},
}
for name, payload in SHAPES.items():
    r = call(payload)
    print(f"{name:24s} ok={r.get('ok')} state={r.get('state')} "
          f"thr={r.get('throttle')} gear={r.get('gear')} vc_thr={(r.get('vehicle_control') or {}).get('throttle')}",
          flush=True)
    time.sleep(0.4)

print("\n== 读一次观测，看 state 与位置 ==", flush=True)
r = call({"command": "get_observation", "vehicle_id": "Car_D"})
o = r.get("observation") or {}
print("  obs.state:", o.get("state"), "pos:", o.get("position"), "phase:", o.get("phase"), flush=True)
s.close()
