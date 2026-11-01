# -*- coding: utf-8 -*-
"""按车道接近信号：倒退回到车道 y=120.78，开到信号正下方，朝北面对，再 submit_key。"""
from __future__ import annotations

import json
import math
import socket
import time


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class P:
    def __init__(self):
        self.s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
        self.s.settimeout(10)
        self.buf = b""
        self.pts = []
        self.head = None

    def call(self, p):
        self.s.sendall((json.dumps(p) + "\n").encode())
        while b"\n" not in self.buf:
            c = self.s.recv(1 << 20)
            if not c:
                raise ConnectionError
            self.buf += c
        line, rest = self.buf.split(b"\n", 1)
        self.buf = rest
        return json.loads(line.decode("utf-8", "replace"))

    def ctrl(self, vid, throttle=0.0, brake=0.0, steering=0.0, gear=2):
        self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                   "vehicle_control": {"gear": gear, "throttle": throttle,
                                       "brake": brake, "steering": steering}})

    def obs(self, vid):
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        x, y = float(p.get("x", 0.0)), float(p.get("y", 0.0))
        self.pts.append((x, y))
        if len(self.pts) > 40:
            self.pts.pop(0)
        for bx, by in reversed(self.pts[:-1]):
            if math.hypot(x - bx, y - by) > 0.8:
                h = math.degrees(math.atan2(y - by, x - bx))
                self.head = h if self.head is None else self.head + 0.5 * wrap180(h - self.head)
                break
        return x, y, o

    def drive(self, vid, secs, thr, steer, gear):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, throttle=thr, steering=steer, gear=gear)
            time.sleep(0.07)
        self.ctrl(vid, brake=1.0, gear=3)
        time.sleep(0.25)

    def goto(self, vid, tx, ty, tol=2.5, thr=0.28, deadline=60.0):
        t0 = time.time()
        while time.time() - t0 < deadline:
            x, y, o = self.obs(vid)
            d = math.hypot(tx - x, ty - y)
            if d <= tol:
                self.ctrl(vid, brake=1.0, gear=3)
                return True
            if self.head is None:
                self.ctrl(vid, throttle=0.30, steering=0.0, gear=2)
                time.sleep(0.07)
                continue
            want = math.degrees(math.atan2(ty - y, tx - x))
            err = wrap180(want - self.head)
            steer = max(-0.6, min(0.6, err / 70.0))
            t = thr if abs(err) < 50 else 0.20
            self.ctrl(vid, throttle=t, steering=steer, gear=2)
            time.sleep(0.07)
        self.ctrl(vid, brake=1.0, gear=3)
        return False


p = P()
vid = "Car_C"
sig = (587.47, 124.78)
lane_y = 120.78

x, y, o = p.obs(vid)
print(f"起点 ({x:.1f},{y:.1f})", flush=True)

# 1) 倒车回到车道
p.drive(vid, 2.0, 0.5, 0.0, 4)
x, y, o = p.obs(vid)
print(f"倒车后 ({x:.1f},{y:.1f})", flush=True)

# 2) 开到信号正下方的车道上
ok = p.goto(vid, sig[0], lane_y, tol=2.0)
x, y, o = p.obs(vid)
print(f"到车道 ({x:.1f},{y:.1f}) ok={ok}", flush=True)

# 3) 朝北（+y = 90°）面对信号
t0 = time.time()
while time.time() - t0 < 12.0:
    x, y, o = p.obs(vid)
    if abs(y - lane_y) > 4.0:
        break
    h = p.head if p.head is not None else 0.0
    err = wrap180(90.0 - h)
    if abs(err) < 15:
        break
    steer = max(-0.8, min(0.8, err / 40.0))
    p.ctrl(vid, throttle=0.26, steering=steer, gear=2)
    time.sleep(0.07)
p.ctrl(vid, brake=1.0, gear=3)
time.sleep(0.3)
x, y, o = p.obs(vid)
print(f"对准后 ({x:.1f},{y:.1f}) 航向={p.head}", flush=True)
objs = o.get("visible_objects") or []
print(f"可见 {len(objs)}: {[b.get('object_id') for b in objs]}", flush=True)
for ob in objs:
    if ob.get("object_id") == "Signal_A":
        print("  ", json.dumps(ob, ensure_ascii=False)[:300], flush=True)

for act in ("submit_key", "report_observation", "scan"):
    r = p.call({"command": "interact", "action": act, "object_id": "Signal_A", "vehicle_id": vid})
    print(f"{act}: ok={r.get('ok')} msg={str(r.get('message'))[:80]}", flush=True)
    time.sleep(0.3)
sc = p.call({"command": "get_task"}).get("score") or {}
print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)
