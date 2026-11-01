# -*- coding: utf-8 -*-
"""脱困 + 直行验证：先倒车脱困，再沿当前空旷方向直行 20 m，记录轨迹。"""
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
    call({"command": "set_vehicle_control", "vehicle_id": vid,
          "vehicle_control": {"gear": gear, "throttle": throttle,
                              "brake": brake, "steering": steering}})


def pos(vid):
    o = call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
    p = o.get("position") or {}
    return float(p.get("x", 0.0)), float(p.get("y", 0.0))


vid = "Car_C"
p0 = pos(vid)
print(f"stuck at ({p0[0]:.1f},{p0[1]:.1f})", flush=True)

# 倒车 2 s
t0 = time.time()
while time.time() - t0 < 2.0:
    ctrl(vid, throttle=0.5, steering=-0.8, gear=4)
    time.sleep(0.08)
ctrl(vid, brake=1.0, gear=3)
time.sleep(0.3)
p1 = pos(vid)
print(f"after reverse ({p1[0]:.1f},{p1[1]:.1f}) 退 {math.hypot(p1[0]-p0[0], p1[1]-p0[1]):.1f} m",
      flush=True)

# 前进 3 s 看能不能走起来
t0 = time.time()
pts = [p1]
while time.time() - t0 < 3.0:
    ctrl(vid, throttle=0.35, steering=0.0, gear=2)
    time.sleep(0.1)
    if time.time() - t0 > 1.0 and len(pts) < 40:
        pts.append(pos(vid))
ctrl(vid, brake=1.0, gear=3)
p2 = pos(vid)
print(f"after forward ({p2[0]:.1f},{p2[1]:.1f}) 走 {math.hypot(p2[0]-p1[0], p2[1]-p1[1]):.1f} m",
      flush=True)
if len(pts) >= 2:
    h = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
    print(f"实际航向 {h:.1f}°", flush=True)
s.close()
