# -*- coding: utf-8 -*-
"""开一段路，逐帧记录 visible_objects，看场景自带视觉给的是什么信息。"""
from __future__ import annotations

import json
import math
import socket
import time

s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
s.settimeout(10)
buf = b""


def call(p):
    global buf
    s.sendall((json.dumps(p) + "\n").encode())
    while b"\n" not in buf:
        c = s.recv(1 << 20)
        if not c:
            raise ConnectionError
        buf += c
    line, rest = buf.split(b"\n", 1)
    buf = rest
    return json.loads(line.decode("utf-8", "replace"))


def ctrl(vid, throttle=0.0, brake=0.0, steering=0.0, gear=2):
    return call({"command": "set_vehicle_control", "vehicle_id": vid,
                 "vehicle_control": {"gear": gear, "throttle": throttle,
                                     "brake": brake, "steering": steering}})


def obs(vid):
    return call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}


vid = "Car_C"
o = obs(vid)
p = o.get("position") or {}
print(f"start ({p.get('x'):.1f},{p.get('y'):.1f})", flush=True)

seen: dict[str, dict] = {}
t0 = time.time()
last_log = 0.0
steer = 0.0
while time.time() - t0 < 25.0:
    ctrl(vid, throttle=0.25, steering=steer, gear=2)
    time.sleep(0.1)
    now = time.time() - t0
    if now - last_log > 1.0:
        last_log = now
        o = obs(vid)
        p = o.get("position") or {}
        objs = o.get("visible_objects") or []
        for ob in objs:
            oid = ob.get("object_id") or ob.get("id") or str(ob)[:20]
            seen[oid] = ob
        line = (f"[{now:5.1f}s] ({p.get('x'):7.1f},{p.get('y'):7.1f}) "
                f"可见={len(objs)} 累计={len(seen)} phase={o.get('phase')}")
        print(line, flush=True)
        for ob in objs[:3]:
            print("      ", json.dumps(ob, ensure_ascii=False)[:220], flush=True)
ctrl(vid, brake=1.0, gear=3)
print("\n== 累计可见物体 ==", flush=True)
for k, v in seen.items():
    print(f"  {k}: {json.dumps(v, ensure_ascii=False)[:260]}", flush=True)
s.close()
