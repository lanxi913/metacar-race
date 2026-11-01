# -*- coding: utf-8 -*-
"""决定性实验：直线行驶（steering=0）与打舵行驶时，rotation_y 分别是什么。

结论判据：如果 rotation_y 在直线时≈常数、打舵时跟随 steering 变化，那它就是方向盘角；
如果它跟随实际位移方向，那它就是车身航向。
"""
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


def ctrl(vid, thr=0.0, brk=0.0, st=0.0, gear=2):
    return call({"command": "set_vehicle_control", "vehicle_id": vid,
                 "vehicle_control": {"gear": gear, "throttle": thr,
                                     "brake": brk, "steering": st}})


def pos(vid):
    o = call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
    p = o.get("position") or {}
    return float(p.get("x", 0.0)), float(p.get("y", 0.0))


vid = "Car_A"
x0, y0 = pos(vid)
print(f"start ({x0:.1f},{y0:.1f})", flush=True)

print("--- 直线段（steering=0, 3 s）---", flush=True)
t0 = time.time()
while time.time() - t0 < 3.0:
    r = ctrl(vid, thr=0.35, st=0.0, gear=2)
    x, y = pos(vid)
    print(f"  t={time.time()-t0:4.1f} ({x:7.2f},{y:7.2f}) rotation_y={r.get('rotation_y')} "
          f"steering={r.get('steering')}", flush=True)
    time.sleep(0.4)
ctrl(vid, brk=1.0, gear=3)
x1, y1 = pos(vid)
print(f"位移角={math.degrees(math.atan2(y1-y0, x1-x0)):.1f}°", flush=True)

print("--- 打舵段（steering=+0.8, 2 s）---", flush=True)
t0 = time.time()
while time.time() - t0 < 2.0:
    r = ctrl(vid, thr=0.30, st=0.8, gear=2)
    print(f"  t={time.time()-t0:4.1f} rotation_y={r.get('rotation_y')} steering={r.get('steering')}",
          flush=True)
    time.sleep(0.4)
ctrl(vid, brk=1.0, gear=3)
x2, y2 = pos(vid)
print(f"打舵后位移角={math.degrees(math.atan2(y2-y1, x2-x1)):.1f}°", flush=True)
s.close()
