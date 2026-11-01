# -*- coding: utf-8 -*-
"""最小验证：给油门看车能不能动，并测直线跟踪（只在车道上小幅修正）。"""
from __future__ import annotations

import json
import math
import socket
import time

s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
s.settimeout(10)
buf = b""


def call(p):
    global buf
    s.sendall((json.dumps(p) + "\n").encode())
    while b"\n" not in buf:
        c = s.recv(1 << 20)
        if not c:
            raise ConnectionError
        buf += c
    line, rest = buf.split(b"\n", 1)
    buf = rest
    return json.loads(line.decode("utf-8", "replace"))


def ctrl(vid, throttle=0.0, brake=0.0, steering=0.0, gear=2):
    return call({"command": "set_vehicle_control", "vehicle_id": vid,
                 "vehicle_control": {"gear": gear, "throttle": throttle,
                                     "brake": brake, "steering": steering}})


def pos(vid):
    o = call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
    p = o.get("position") or {}
    return float(p.get("x", 0.0)), float(p.get("y", 0.0))


for vid in ("Car_C", "Car_D", "Car_A", "Car_B"):
    a = pos(vid)
    t0 = time.time()
    st = None
    while time.time() - t0 < 1.5:
        r = ctrl(vid, throttle=0.45, steering=0.0, gear=2)
        st = r.get("state")
        time.sleep(0.08)
    ctrl(vid, brake=1.0, gear=3)
    b = pos(vid)
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    print(f"{vid}: ({a[0]:.1f},{a[1]:.1f}) -> ({b[0]:.1f},{b[1]:.1f}) 动了 {d:5.2f} m  state={st}",
          flush=True)
