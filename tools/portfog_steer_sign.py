# -*- coding: utf-8 -*-
"""用位移算航向，标定 steering 符号（+1 是左还是右）。"""
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
    call({"command": "set_vehicle_control", "vehicle_id": vid,
          "vehicle_control": {"gear": gear, "throttle": throttle,
                              "brake": brake, "steering": steering}})


def pos(vid):
    o = call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
    p = o.get("position") or {}
    return p.get("x", 0.0), p.get("y", 0.0)


def heading(a, b):
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def stop(vid):
    for _ in range(12):
        ctrl(vid, brake=1.0, gear=3)
        time.sleep(0.05)


for steer in (1.0, -1.0):
    stop("Car_D")
    a = pos("Car_D")
    t0 = time.time()
    while time.time() - t0 < 1.5:      # 先直行起步
        ctrl("Car_D", throttle=0.35, steering=0.0)
        time.sleep(0.05)
    b = pos("Car_D")
    t0 = time.time()
    while time.time() - t0 < 2.0:      # 再打方向
        ctrl("Car_D", throttle=0.35, steering=steer)
        time.sleep(0.05)
    c = pos("Car_D")
    h1, h2 = heading(a, b), heading(b, c)
    dh = (h2 - h1 + 180) % 360 - 180
    print(f"steer={steer:+.1f}: 直行航向 {h1:7.1f}° -> 打方向后 {h2:7.1f}°  Δ={dh:+6.1f}°", flush=True)
stop("Car_D")
s.close()
