# -*- coding: utf-8 -*-
"""看 vehicle_states（复数）里是否有实时位姿——observation.position 可能只是出生点。"""
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


def show(tag):
    r = call({"command": "get_observation", "vehicle_id": "Car_C"})
    o = r.get("observation") or {}
    vs = r.get("vehicle_states") or []
    print(f"-- {tag}", flush=True)
    print("   obs.position:", o.get("position"), flush=True)
    print(f"   vehicle_states: {len(vs)}", flush=True)
    for v in vs[:4]:
        p = v.get("position") or {}
        print(f"     {v.get('vehicle_id')}: ({p.get('x')},{p.get('y')}) "
              f"state={v.get('state')} spd={v.get('speed_mps')}", flush=True)


show("before")
for _ in range(30):
    call({"command": "set_vehicle_control", "vehicle_id": "Car_C", "gear": 1,
          "throttle": 0.4, "brake": 0.0, "steering": 0.0})
    time.sleep(0.1)
show("after 3s throttle")
