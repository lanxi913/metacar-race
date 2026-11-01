# -*- coding: utf-8 -*-
"""直行跟踪测试：位移航向(2m 基线) + 增益 /60 + 20 Hz，看横向漂移。"""
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
pts = [pos(vid)]
start = pts[0]
head = None
t0 = time.time()
last_log = -1.0
while time.time() - t0 < 10.0:
    x, y = pos(vid)
    if math.hypot(x - pts[-1][0], y - pts[-1][1]) > 1.5:
        pts.append((x, y))
        if len(pts) >= 2:
            h = math.degrees(math.atan2(pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0]))
            head = h if head is None else head + 0.7 * wrap180(h - head)
    want = -90.0                       # 正南
    err = wrap180(want - head) if head is not None else 0.0
    steer = max(-0.6, min(0.6, err / 60.0))
    thr = 0.30 if abs(err) < 40 else 0.18
    ctrl(vid, throttle=thr, steering=steer, gear=2)
    time.sleep(0.05)
    if time.time() - t0 - last_log > 1.5:
        last_log = time.time() - t0
        print(f"  t={last_log:4.1f} ({x:7.1f},{y:7.1f}) head={head if head is None else round(head,1)} "
              f"err={err:+6.1f} steer={steer:+.2f}", flush=True)
for _ in range(12):
    ctrl(vid, brake=1.0, gear=3)
    time.sleep(0.05)
end = pos(vid)
dy = end[1] - start[1]
dx = end[0] - start[0]
print(f"位移 dx={dx:+.1f} dy={dy:+.1f} 直线度={abs(dx)/max(1e-6,abs(dy)):.3f}", flush=True)
s.close()
