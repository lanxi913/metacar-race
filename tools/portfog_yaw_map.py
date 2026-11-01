# -*- coding: utf-8 -*-
"""定标：yaw(rotation_y) 与实际运动方向的关系。直线行驶 2 秒，比较两者。"""
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


vid = "Car_D"
for trial in range(3):
    a = pos(vid)
    r = ctrl(vid, throttle=0.45, steering=0.0, gear=2)
    yaw0 = r.get("rotation_y")
    t0 = time.time()
    while time.time() - t0 < 2.0:
        r = ctrl(vid, throttle=0.45, steering=0.0, gear=2)
        time.sleep(0.08)
    ctrl(vid, brake=1.0, gear=3)
    b = pos(vid)
    displ = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    print(f"trial{trial}: yaw={yaw0:7.1f}  位移=({b[0]-a[0]:+6.2f},{b[1]-a[1]:+6.2f}) "
          f"位移角={displ:7.1f}  差={((displ - yaw0 + 180) % 360) - 180:+7.1f}", flush=True)
    time.sleep(0.3)
s.close()
