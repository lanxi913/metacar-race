# -*- coding: utf-8 -*-
"""对准信号方位 -> 小步直行逼近 -> 提交 Key（每步都重新观测）。"""
from __future__ import annotations

import json
import math
import socket
import time


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Car:
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

    def st(self, vid):
        r = self.ctrl(vid, brk=1.0, gear=3)
        h = (float(r.get("rotation_y") or 0.0) + 180.0) % 360.0
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        return float(p.get("x", 0.0)), float(p.get("y", 0.0)), h, o

    def rev(self, vid, secs, steer):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, thr=0.45, st=steer, gear=4)
            time.sleep(0.07)
        self.ctrl(vid, brk=1.0, gear=3)
        time.sleep(0.12)

    def orient(self, vid, target, tol=10.0, rounds=8):
        for _ in range(rounds):
            _, _, h, _ = self.st(vid)
            err = wrap180(target - h)
            if abs(err) <= tol:
                return True
            self.rev(vid, min(1.4, max(0.3, abs(err) / 50.0)), 1.0 if err > 0 else -1.0)
        _, _, h, _ = self.st(vid)
        return abs(wrap180(target - h)) <= tol + 6


def find(o, oid):
    for ob in (o.get("visible_objects") or []):
        if ob.get("object_id") == oid:
            p = ob.get("position") or {}
            return float(p.get("x", 0)), float(p.get("y", 0)), ob
    return None


car = Car()
vid = "Car_C"
OID = "Signal_A"

x, y, h, o = car.st(vid)
print(f"{vid} ({x:.1f},{y:.1f}) 航向={h:.1f}", flush=True)

for step in range(14):
    x, y, h, o = car.st(vid)
    sig = find(o, OID)
    if sig is None:
        print(f"[{step}] 信号不在视野 ({x:.1f},{y:.1f}) 航向={h:.1f}", flush=True)
        # 试朝 y=124.8 的西侧方向转
        car.orient(vid, 180.0)
        continue
    tx, ty, ob = sig
    d = math.hypot(tx - x, ty - y)
    print(f"[{step}] ({x:.1f},{y:.1f}) 航向={h:.1f} d={d:.1f}", flush=True)
    if d <= 6.0:
        break
    want = math.degrees(math.atan2(ty - y, tx - x))
    err = wrap180(want - h)
    if abs(err) > 20:
        car.orient(vid, want)
        continue
    # 小步直行（0.6 s），每步重新观测
    t0 = time.time()
    while time.time() - t0 < 0.6:
        car.ctrl(vid, thr=0.20, st=max(-0.5, min(0.5, err / 60.0)), gear=2)
        time.sleep(0.06)
    car.ctrl(vid, brk=1.0, gear=3)

x, y, h, o = car.st(vid)
sig = find(o, OID)
if sig:
    d = math.hypot(sig[0] - x, sig[1] - y)
    print(f"最终 ({x:.1f},{y:.1f}) d={d:.1f} 航向={h:.1f}", flush=True)
    if abs(wrap180(math.degrees(math.atan2(sig[1] - y, sig[0] - x)) - h)) > 15:
        car.orient(vid, math.degrees(math.atan2(sig[1] - y, sig[0] - x)))
    for act in ("submit_key", "report_observation"):
        r = car.call({"command": "interact", "action": act, "object_id": OID, "vehicle_id": vid})
        print(f"{act}: ok={r.get('ok')} msg={str(r.get('message'))[:90]}", flush=True)
        time.sleep(0.3)
    sc = car.call({"command": "get_task"}).get("score") or {}
    print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")},
          flush=True)
else:
    print("最后信号不在视野", flush=True)
