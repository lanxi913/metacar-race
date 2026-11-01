# -*- coding: utf-8 -*-
"""对准信号方位 -> 直行接近 -> 提交 Key（一条龙）。"""
from __future__ import annotations

import json
import math
import socket
import time


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class C:
    def __init__(self):
        self.s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
        self.s.settimeout(10)
        self.buf = b""

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
        return self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                          "vehicle_control": {"gear": gear, "throttle": thr,
                                              "brake": brk, "steering": st}})

    def yaw(self, vid):
        return (float(self.ctrl(vid, brk=1.0, gear=3).get("rotation_y") or 0.0) + 180.0) % 360.0

    def obs(self, vid):
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        return float(p.get("x", 0.0)), float(p.get("y", 0.0)), o

    def orient(self, vid, target, tol=8.0, rounds=8):
        for k in range(rounds):
            y = self.yaw(vid)
            err = wrap180(target - y)
            if abs(err) <= tol:
                return True
            secs = min(1.6, max(0.35, abs(err) / 50.0))
            st = 1.0 if err > 0 else -1.0
            t0 = time.time()
            while time.time() - t0 < secs:
                self.ctrl(vid, thr=0.45, st=st, gear=4)
                time.sleep(0.07)
            self.ctrl(vid, brk=1.0, gear=3)
            time.sleep(0.15)
        return abs(wrap180(target - self.yaw(vid))) <= tol


c = C()
vid = "Car_D"
SIG = (587.47, 124.78)

x, y, o = c.obs(vid)
print(f"起点 ({x:.1f},{y:.1f}) 航向={c.yaw(vid):.1f}", flush=True)

# 1) 对准信号方位
for attempt in range(4):
    x, y, o = c.obs(vid)
    want = math.degrees(math.atan2(SIG[1] - y, SIG[0] - x))
    print(f"[{attempt}] ({x:.1f},{y:.1f}) d={math.hypot(SIG[0]-x, SIG[1]-y):.1f} 对准 {want:.0f}°",
          flush=True)
    c.orient(vid, want)
    # 2) 直行靠近
    t0 = time.time()
    while time.time() - t0 < 6.0:
        x, y, o = c.obs(vid)
        d = math.hypot(SIG[0] - x, SIG[1] - y)
        objs = [b.get("object_id") for b in (o.get("visible_objects") or [])]
        if d < 6.0:
            break
        c.ctrl(vid, thr=0.30, st=0.0, gear=2)
        time.sleep(0.07)
    c.ctrl(vid, brk=1.0, gear=3)
    time.sleep(0.2)
    x, y, o = c.obs(vid)
    objs = [b.get("object_id") for b in (o.get("visible_objects") or [])]
    d = math.hypot(SIG[0] - x, SIG[1] - y)
    print(f"    -> ({x:.1f},{y:.1f}) d={d:.1f} 可见={objs}", flush=True)
    if "Signal_A" in objs or d < 6.5:
        break

for act in ("submit_key", "report_observation"):
    r = c.call({"command": "interact", "action": act, "object_id": "Signal_A", "vehicle_id": vid})
    print(f"{act}: ok={r.get('ok')} msg={str(r.get('message'))[:90]}", flush=True)
    time.sleep(0.3)
x, y, o = c.obs(vid)
sc = c.call({"command": "get_task"}).get("score") or {}
print(f"sig={o.get('verified_signal_count')}/{o.get('total_signal_count')}", flush=True)
print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)
