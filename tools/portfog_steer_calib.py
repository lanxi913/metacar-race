# -*- coding: utf-8 -*-
"""转向标定：直行 -> 左打/右打，看 yaw 与位移方向的关系；同时确认 gear=4 是否倒车。"""
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


def pose(vid):
    r = call({"command": "get_observation", "vehicle_id": vid})
    o = r.get("observation") or {}
    p = o.get("position") or {}
    return p.get("x", 0.0), p.get("y", 0.0), r.get("rotation_y")


def run(vid, secs, thr, steer, gear=2):
    t0 = time.time()
    while time.time() - t0 < secs:
        ctrl(vid, throttle=thr, steering=steer, gear=gear)
        time.sleep(0.05)


def stop(vid):
    for _ in range(12):
        ctrl(vid, brake=1.0, gear=3)
        time.sleep(0.05)


print("== 直行 2s (gear2, steer=0) ==", flush=True)
a = pose("Car_D")
run("Car_D", 2.0, 0.4, 0.0)
b = pose("Car_D")
print(f"  {a} -> {b}  d={math.hypot(b[0]-a[0], b[1]-a[1]):.2f} "
      f"dx={b[0]-a[0]:+.2f} dy={b[1]-a[1]:+.2f}", flush=True)
stop("Car_D")

print("== 直行 + 右打 (steer=+1) 2s ==", flush=True)
a = pose("Car_D")
run("Car_D", 2.0, 0.4, 1.0)
b = pose("Car_D")
dyaw = None
if a[2] is not None and b[2] is not None:
    dyaw = b[2] - a[2]
print(f"  yaw {a[2]} -> {b[2]} (d={dyaw})", flush=True)
stop("Car_D")

print("== 直行 + 左打 (steer=-1) 2s ==", flush=True)
a = pose("Car_D")
run("Car_D", 2.0, 0.4, -1.0)
b = pose("Car_D")
dyaw = None
if a[2] is not None and b[2] is not None:
    dyaw = b[2] - a[2]
print(f"  yaw {a[2]} -> {b[2]} (d={dyaw})", flush=True)
stop("Car_D")

print("== gear=4 方向 ==", flush=True)
a = pose("Car_D")
run("Car_D", 1.5, 0.4, 0.0, gear=4)
b = pose("Car_D")
print(f"  {a} -> {b} dx={b[0]-a[0]:+.2f} dy={b[1]-a[1]:+.2f}", flush=True)
stop("Car_D")
s.close()
