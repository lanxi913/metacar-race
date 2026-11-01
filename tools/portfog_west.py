# -*- coding: utf-8 -*-
"""沿 y=120.78 车道向西纯跟踪到 Signal_A 下方（不做路由，短程直行 + 横向纠偏）。"""
from __future__ import annotations

import json
import math
import socket
import time

LANE_Y = 120.78
SIG = (587.47, 124.78)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Car:
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

    def ctrl(self, vid, thr=0.0, brk=0.0, st=0.0, gear=2):
        self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                   "vehicle_control": {"gear": gear, "throttle": thr,
                                       "brake": brk, "steering": st}})

    def st(self, vid):
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        x, y = float(p.get("x", 0.0)), float(p.get("y", 0.0))
        self.pts.append((x, y))
        if len(self.pts) > 60:
            self.pts.pop(0)
        for bx, by in reversed(self.pts[:-1]):
            if math.hypot(x - bx, y - by) > 1.2:
                h = math.degrees(math.atan2(y - by, x - bx))
                self.head = h if self.head is None else wrap180(self.head + 0.5 * wrap180(h - self.head))
                break
        return x, y, o


car = Car()
import os
vid = os.environ.get("PF_VID", "Car_D")
x, y, o = car.st(vid)
print(f"{vid} ({x:.1f},{y:.1f})", flush=True)

t0 = time.time()
last = 0.0
stall_t = time.time()
best = 1e9
while time.time() - t0 < 100:
    x, y, o = car.st(vid)
    if x <= SIG[0] + 0.5:
        break
    # 前视点：正西 6 m，钉在车道上
    cx, cy = x - 6.0, LANE_Y
    if car.head is None:
        car.ctrl(vid, thr=0.25, st=0.0, gear=2)
        time.sleep(0.07)
        continue
    want = math.degrees(math.atan2(cy - y, cx - x))
    err = wrap180(want - car.head)
    steer = max(-0.5, min(0.5, err / 60.0))
    car.ctrl(vid, thr=0.20 if abs(err) < 35 else 0.14, st=steer, gear=2)
    if x < best - 0.5:
        best, stall_t = x, time.time()
    if time.time() - stall_t > 4.0:
        print(f"  !! 卡住 ({x:.1f},{y:.1f})，倒车", flush=True)
        t1 = time.time()
        while time.time() - t1 < 1.5:
            car.ctrl(vid, thr=0.45, st=-0.9, gear=4)
            time.sleep(0.07)
        car.ctrl(vid, brk=1.0, gear=3)
        time.sleep(0.3)
        best, stall_t = 1e9, time.time()
        continue
    if time.time() - last > 4.0:
        last = time.time()
        print(f"  ({x:7.1f},{y:7.1f}) 航向={car.head:6.1f} err={err:+6.1f} "
              f"横向={y-LANE_Y:+5.2f} 距信号={math.hypot(SIG[0]-x, SIG[1]-y):5.1f}", flush=True)
    time.sleep(0.07)

car.ctrl(vid, brk=1.0, gear=3)
x, y, o = car.st(vid)
print(f"结束 ({x:.1f},{y:.1f}) 距信号 {math.hypot(SIG[0]-x, SIG[1]-y):.1f} "
      f"可见={[b.get('object_id') for b in (o.get('visible_objects') or [])]}", flush=True)

if math.hypot(SIG[0] - x, SIG[1] - y) < 9.0:
    import sys
    sys.path.insert(0, r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race")
    from metacar_multiagent import ScenarioTaskAPI
    with ScenarioTaskAPI() as api:
        obs = api.get_observation(vid)
        tob = next((b for b in obs.visible_objects if b.object_id == "Signal_A"), None)
        if tob:
            print("token:", repr(tob.observation_token), "hint:", tob.hint, flush=True)
            r = api.interact(vid, "Signal_A", "submit_key", {"key": tob.observation_token})
            print(f"submit granted={r.granted} msg={r.message}", flush=True)
        else:
            print("信号不可见", flush=True)
