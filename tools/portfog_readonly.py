# -*- coding: utf-8 -*-
"""港口迷雾：纯读数，不发任何控制，看 elapsed 会不会自己走。"""
from __future__ import annotations

import json
import socket
import time


def connect():
    for _ in range(180):
        try:
            s = socket.create_connection(("127.0.0.1", 9100), timeout=5)
            s.settimeout(5)
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
            raise ConnectionError
        buf += c
    line, buf = buf.split(b"\n", 1)
    return json.loads(line.decode("utf-8", "replace"))


print("connected (read-only, 不发控制)", flush=True)
t0 = time.time()
while time.time() - t0 < 20:
    sc = call({"command": "get_task"}).get("score") or {}
    o = call({"command": "get_observation", "vehicle_id": "Car_C"}).get("observation") or {}
    p = o.get("position") or {}
    print(f"[{time.time()-t0:5.1f}s] elapsed={sc.get('elapsed_seconds'):8} "
          f"pos=({p.get('x', 0):7.1f},{p.get('y', 0):7.1f}) phase={o.get('phase')}", flush=True)
    time.sleep(2.0)
