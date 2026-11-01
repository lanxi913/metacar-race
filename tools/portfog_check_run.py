# -*- coding: utf-8 -*-
"""连续心跳 12 秒 + 一次 interact，看事件是否开始增长（判断任务是否在跑）。"""
from __future__ import annotations

import json
import socket
import time

VIDS = ["Car_A", "Car_B", "Car_C", "Car_D"]

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


def ev_info():
    ev = call({"command": "get_events"}).get("events") or []
    return len(ev), (round(ev[-1].get("elapsed_seconds", 0), 1) if ev else 0)


t0 = time.time()
last = 0.0
while time.time() - t0 < 12:
    call({"command": "set_vehicle_controls",
          "vehicle_controls": [{"vehicle_id": v, "gear": 1, "throttle": 0.0,
                                "brake": 1.0, "steering": 0.0} for v in VIDS]})
    time.sleep(0.1)
    if time.time() - t0 - last > 2:
        last = time.time() - t0
        n, t = ev_info()
        print(f"t={last:4.1f}s events={n} last_t={t}", flush=True)

print("--- interact scan ---", flush=True)
r = call({"command": "interact", "action": "scan", "object_id": "Obstacle_1", "vehicle_id": "Car_C"})
print("scan:", r.get("ok"), str(r.get("message"))[:70], flush=True)
time.sleep(1.0)
n, t = ev_info()
print("events after:", n, "last_t:", t, flush=True)
s.close()
