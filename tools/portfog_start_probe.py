# -*- coding: utf-8 -*-
"""港口迷雾：找"启动信号"——试 interact / demo_phase / 首次心跳后看 elapsed 是否开始走。"""
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


def elapsed():
    sc = call({"command": "get_task"}).get("score") or {}
    return sc.get("elapsed_seconds"), sc.get("total_score")


print("start elapsed:", elapsed(), flush=True)

print("\n-- 1) demo_phase 各候选 --", flush=True)
for ph in ("start", "running", "play", "begin", "mission", "ready"):
    r = call({"command": "demo_phase", "phase": ph})
    print(f"  demo_phase={ph:8s} ok={r.get('ok')} msg={str(r.get('message'))[:70]}", flush=True)
    time.sleep(0.2)
print("  elapsed after:", elapsed(), flush=True)

print("\n-- 2) 一次 interact --", flush=True)
r = call({"command": "interact", "action": "scan", "object_id": "Obstacle_1", "vehicle_id": "Car_C"})
print("  scan:", r.get("ok"), str(r.get("message"))[:80], flush=True)
time.sleep(1)
print("  elapsed after:", elapsed(), flush=True)

print("\n-- 3) 持续心跳 6 s --", flush=True)
t0 = time.time()
while time.time() - t0 < 6:
    call({"command": "set_vehicle_control", "vehicle_id": "Car_C", "gear": 1,
          "throttle": 0.2, "brake": 0.0, "steering": 0.0})
    time.sleep(0.2)
print("  elapsed after:", elapsed(), flush=True)

print("\n-- 4) 事件尾部 --", flush=True)
ev = call({"command": "get_events"}).get("events") or []
for e in ev[-5:]:
    print(f"  #{e.get('sequence')} {e.get('elapsed_seconds'):7.2f} {e.get('event_type')}: "
          f"{str(e.get('message'))[:80]}", flush=True)
