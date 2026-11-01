# -*- coding: utf-8 -*-
"""10 秒不间断心跳驱动（中途不切窗口），每 2 秒读一次位置。"""
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


vid = "Car_A"
print("start", vid, pos(vid), flush=True)
t0 = time.time()
last = 0.0
while time.time() - t0 < 10.0:
    ask({"command": "set_vehicle_control", "vehicle_id": vid, "gear": 1,
         "throttle": 0.45, "brake": 0.0, "steering": 0.0})
    time.sleep(0.1)
    if time.time() - t0 - last > 2.0:
        last = time.time() - t0
        print(f"  t={last:4.1f}s {vid} {pos(vid)}", flush=True)
print("end", pos(vid), flush=True)
