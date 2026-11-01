# -*- coding: utf-8 -*-
"""验证连接纪律：单条长连接、10 Hz、只发最少命令，看场景能否稳定活 60 秒。"""
from __future__ import annotations

import json
import socket
import time

VIDS = ["Car_A", "Car_B", "Car_C", "Car_D"]


def connect():
    for _ in range(60):
        try:
            s = socket.create_connection(("127.0.0.1", 9100), timeout=5)
            s.settimeout(10)
            return s
        except Exception:
            time.sleep(1.0)
    raise SystemExit("9100 not up")


s = connect()
buf = b""


def call(p):
    global buf
    s.sendall((json.dumps(p) + "\n").encode())
    while b"\n" not in buf:
        c = s.recv(65536)
        if not c:
            raise ConnectionError("closed")
        buf += c
    line, buf = buf.split(b"\n", 1)
    return json.loads(line.decode("utf-8", "replace"))


print("connected (single persistent socket)", flush=True)
t0 = time.time()
n = 0
while time.time() - t0 < 60:
    n += 1
    call({"command": "set_vehicle_controls",
          "vehicle_controls": [{"vehicle_id": v, "gear": 1, "throttle": 0.0,
                                "brake": 1.0, "steering": 0.0} for v in VIDS]})
    if n % 30 == 0:
        ev = call({"command": "get_events"}).get("events") or []
        o = call({"command": "get_observation", "vehicle_id": "Car_D"}).get("observation") or {}
        p = o.get("position") or {}
        print(f"[{time.time()-t0:5.1f}s] events={len(ev)} "
              f"last_t={round(ev[-1].get('elapsed_seconds',0),1) if ev else 0} "
              f"Car_D=({p.get('x',0):.1f},{p.get('y',0):.1f})", flush=True)
    time.sleep(0.1)
print("done, socket kept", flush=True)
s.close()
