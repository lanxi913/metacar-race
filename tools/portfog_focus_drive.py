# -*- coding: utf-8 -*-
"""验证：把场景窗口保持前台时，Car_D 是否能被持续驱动。"""
from __future__ import annotations

import json
import socket
import time


def ask(p, timeout=4):
    s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
    s.settimeout(timeout)
    try:
        s.sendall((json.dumps(p) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            c = s.recv(65536)
            if not c:
                break
            buf += c
        return json.loads(buf.split(b"\n")[0].decode("utf-8", "replace"))
    finally:
        s.close()


def pos(v):
    o = ask({"command": "get_observation", "vehicle_id": v}).get("observation") or {}
    p = o.get("position") or {}
    return round(p.get("x", 0), 2), round(p.get("y", 0), 2)


for vid in ("Car_D", "Car_C"):
    a = pos(vid)
    t0 = time.time()
    while time.time() - t0 < 4.0:
        ask({"command": "set_vehicle_control", "vehicle_id": vid, "gear": 1,
             "throttle": 0.5, "brake": 0.0, "steering": 0.0})
        time.sleep(0.1)
    b = pos(vid)
    d = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
    print(f"{vid}: {a} -> {b}  moved {d:.2f} m")
