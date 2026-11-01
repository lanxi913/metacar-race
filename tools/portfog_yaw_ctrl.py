# -*- coding: utf-8 -*-
"""验证用 echo rotation_y 做航向的直线控制（不用位移估计）。"""
from __future__ import annotations

import json
import math
import socket
import time


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


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


vid = "Car_C"
a = pos(vid)
print(f"start ({a[0]:.1f},{a[1]:.1f})", flush=True)

# 目标航向：正南（-y） = -90°
want = -90.0
t0 = time.time()
last = -1
while time.time() - t0 < 6.0:
    r = ctrl(vid, throttle=0.35, gear=2)
    yaw = r.get("rotation_y")
    err = wrap180(want - yaw)
    steer = max(-1.0, min(1.0, err / 45.0))
    ctrl(vid, throttle=0.35, steering=steer, gear=2)
    time.sleep(0.1)
    if time.time() - t0 - last > 1.0:
        last = time.time() - t0
        x, y = pos(vid)
        print(f"  t={last:4.1f} ({x:7.1f},{y:7.1f}) yaw={yaw:6.1f} err={err:+6.1f} steer={steer:+.2f}",
              flush=True)
for _ in range(12):
    ctrl(vid, brake=1.0, gear=3)
    time.sleep(0.05)
b = pos(vid)
print(f"end ({b[0]:.1f},{b[1]:.1f}) 位移 {math.hypot(b[0]-a[0], b[1]-a[1]):.1f} m", flush=True)
s.close()
