# -*- coding: utf-8 -*-
"""油门映射标定：不同 throttle 各跑 2 秒，量实际位移速度。"""
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


for thr in (0.3, 0.6, 1.0):
    a = pos("Car_D")
    t0 = time.time()
    vmax = 0.0
    while time.time() - t0 < 2.0:
        r = ctrl("Car_D", throttle=thr)
        st = (r.get("vehicle_state") or {}).get("speed_mps") or 0.0
        vmax = max(vmax, st)
        time.sleep(0.1)
    b = pos("Car_D")
    d = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
    print(f"thr={thr:4.1f} moved {d:6.2f} m in 2.0 s  => {d/2.0:5.2f} m/s (state spd max {vmax:.2f})",
          flush=True)
# 停住
for _ in range(15):
    ctrl("Car_D", brake=1.0)
    time.sleep(0.1)
s.close()
