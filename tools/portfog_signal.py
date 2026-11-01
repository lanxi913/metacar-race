# -*- coding: utf-8 -*-
"""靠近 Signal_A 并提交 Key：低速直线接近 + 到位后 interact submit_key。"""
from __future__ import annotations

import json
import math
import socket
import time


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


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
    call({"command": "set_vehicle_control", "vehicle_id": vid,
          "vehicle_control": {"gear": gear, "throttle": throttle,
                              "brake": brake, "steering": steering}})


def obs(vid):
    o = call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
    p = o.get("position") or {}
    return float(p.get("x", 0.0)), float(p.get("y", 0.0)), o


def find_signal(o, oid="Signal_A"):
    for ob in (o.get("visible_objects") or []):
        if ob.get("object_id") == oid:
            p = ob.get("position") or {}
            return float(p.get("x", 0)), float(p.get("y", 0)), ob
    return None


vid = "Car_C"
x, y, o = obs(vid)
print(f"{vid} at ({x:.1f},{y:.1f})", flush=True)
sig = find_signal(o)
if sig is None:
    print("Signal_A 当前不在视野（>30 m），先向 (587.5,124.8) 靠近", flush=True)
    tx, ty = 587.5, 124.8
else:
    tx, ty, ob = sig
    print(f"看到 Signal_A at ({tx:.1f},{ty:.1f}) d={ob.get('distance_m'):.1f}", flush=True)

pts = [(x, y)]
head = None
t0 = time.time()
while time.time() - t0 < 60:
    x, y, o = obs(vid)
    pts.append((x, y))
    if len(pts) > 30:
        pts.pop(0)
    for bx, by in reversed(pts[:-1]):
        if math.hypot(x - bx, y - by) > 2.0:
            h = math.degrees(math.atan2(y - by, x - bx))
            head = h if head is None else head + 0.7 * wrap180(h - head)
            break
    sig = find_signal(o)
    if sig:
        tx, ty, ob = sig
    d = math.hypot(tx - x, ty - y)
    if d <= 5.0:
        ctrl(vid, brake=1.0, gear=3)
        print(f"到位 ({x:.1f},{y:.1f}) d={d:.1f}", flush=True)
        break
    if head is None:
        ctrl(vid, throttle=0.25, steering=0.0, gear=2)
    else:
        want = math.degrees(math.atan2(ty - y, tx - x))
        err = wrap180(want - head)
        steer = max(-0.5, min(0.5, err / 90.0))
        thr = 0.20 if abs(err) > 40 else 0.26
        ctrl(vid, throttle=thr, steering=steer, gear=2)
    time.sleep(0.06)
ctrl(vid, brake=1.0, gear=3)
time.sleep(0.5)

for action in ("submit_key", "report_observation"):
    r = call({"command": "interact", "action": action, "object_id": "Signal_A", "vehicle_id": vid})
    print(f"interact {action}: ok={r.get('ok')} msg={str(r.get('message'))[:90]}", flush=True)
    time.sleep(0.3)

x, y, o = obs(vid)
sc = call({"command": "get_task"}).get("score") or {}
print(f"pos ({x:.1f},{y:.1f}) sig={o.get('verified_signal_count')}/{o.get('total_signal_count')}",
      flush=True)
print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)
s.close()
