# -*- coding: utf-8 -*-
"""标定回包 rotation_y 与位移方向的关系，并测倒车脱困。"""
from __future__ import annotations

import json
import math
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
r = ctrl(vid, throttle=0.0, brake=1.0, gear=3)
print(f"start ({a[0]:.1f},{a[1]:.1f}) echo yaw={r.get('rotation_y')}", flush=True)

# 倒车脱困
for _ in range(15):
    r = ctrl(vid, throttle=0.4, steering=-1.0, gear=4)
    time.sleep(0.08)
b = pos(vid)
print(f"after reverse ({b[0]:.1f},{b[1]:.1f}) moved {math.hypot(b[0]-a[0], b[1]-a[1]):.2f} m",
      flush=True)

# 前进一段，比较位移角与 yaw
a = pos(vid)
t0 = time.time()
while time.time() - t0 < 1.5:
    r = ctrl(vid, throttle=0.35, steering=0.0, gear=2)
    time.sleep(0.08)
b = pos(vid)
yaw = r.get("rotation_y")
displ = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
print(f"drive: ({a[0]:.1f},{a[1]:.1f}) -> ({b[0]:.1f},{b[1]:.1f}) "
      f"位移角={displ:7.1f}°  echo yaw={yaw}", flush=True)
for _ in range(12):
    ctrl(vid, brake=1.0, gear=3)
    time.sleep(0.05)
s.close()
