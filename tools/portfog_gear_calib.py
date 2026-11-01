# -*- coding: utf-8 -*-
"""挡位枚举标定：gear 0..4 × throttle 0.6，各跑 2 秒量速度。"""
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


def ctrl(vid, throttle=0.0, brake=0.0, steering=0.0, gear=1):
    return call({"command": "set_vehicle_control", "vehicle_id": vid,
                 "vehicle_control": {"gear": gear, "throttle": throttle,
                                     "brake": brake, "steering": steering}})


def pos(vid):
    o = call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
    p = o.get("position") or {}
    return p.get("x", 0.0), p.get("y", 0.0)


for gear in (0, 1, 2, 3, 4):
    for _ in range(10):
        ctrl("Car_D", brake=1.0, gear=gear)
        time.sleep(0.05)
    a = pos("Car_D")
    t0 = time.time()
    st = None
    while time.time() - t0 < 2.0:
        r = ctrl("Car_D", throttle=0.6, gear=gear)
        st = r.get("state")
        time.sleep(0.1)
    b = pos("Car_D")
    d = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
    print(f"gear={gear} moved {d:6.2f} m/2s = {d/2.0:5.2f} m/s  state={st}", flush=True)
for _ in range(15):
    ctrl("Car_D", brake=1.0, gear=3)
    time.sleep(0.1)
s.close()
