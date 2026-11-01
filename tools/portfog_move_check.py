# -*- coding: utf-8 -*-
"""验证车是否能被驱动：连续油门 6 秒，看位置与 maximum_speed_mps。"""
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


def snap(v):
    o = call({"command": "get_observation", "vehicle_id": v}).get("observation") or {}
    p = o.get("position") or {}
    sc = call({"command": "get_task"}).get("score") or {}
    return (round(p.get("x", 0), 2), round(p.get("y", 0), 2),
            round(p.get("z", 0), 3), sc.get("maximum_speed_mps"), sc.get("elapsed_seconds"))


for vid in ("Car_D", "Car_C", "Car_A"):
    before = snap(vid)
    t0 = time.time()
    while time.time() - t0 < 6.0:
        call({"command": "set_vehicle_control", "vehicle_id": vid, "gear": 1,
              "throttle": 0.5, "brake": 0.0, "steering": 0.0})
        time.sleep(0.1)
    after = snap(vid)
    print(f"{vid}: {before} -> {after}", flush=True)
