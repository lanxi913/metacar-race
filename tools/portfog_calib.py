# -*- coding: utf-8 -*-
"""标定：嵌套控制驱动 Car_D，记录位置/航向随时间变化，确定 yaw 与运动方向的关系。"""
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


def obs(vid):
    o = call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
    p = o.get("position") or {}
    return p.get("x", 0.0), p.get("y", 0.0), o


r = call({"command": "get_observation", "vehicle_id": "Car_D"})
x, y = (r.get("observation") or {}).get("position", {}).get("x"), (r.get("observation") or {}).get("position", {}).get("y")
print(f"start ({x:.2f},{y:.2f})", flush=True)

print("-- 直行 4 秒（steering=0）--", flush=True)
t0 = time.time()
last = -1
while time.time() - t0 < 4.0:
    resp = ctrl("Car_D", throttle=0.4)
    if time.time() - t0 - last > 1.0:
        last = time.time() - t0
        ox, oy, o = obs("Car_D")
        print(f"   t={last:4.1f} ({ox:8.2f},{oy:8.2f}) yaw={o.get('yaw') or resp.get('rotation_y')} "
              f"spd={(resp.get('vehicle_state') or {}).get('speed_mps')}", flush=True)
    time.sleep(0.1)

print("-- 右打方向 3 秒 --", flush=True)
t0 = time.time()
while time.time() - t0 < 3.0:
    ctrl("Car_D", throttle=0.3, steering=1.0)
    time.sleep(0.1)
ox, oy, o = obs("Car_D")
print(f"   after steer ({ox:.2f},{oy:.2f})", flush=True)

print("-- 刹车停住 --", flush=True)
for _ in range(20):
    ctrl("Car_D", brake=1.0)
    time.sleep(0.1)
ox, oy, o = obs("Car_D")
print(f"   stopped ({ox:.2f},{oy:.2f})", flush=True)
s.close()
