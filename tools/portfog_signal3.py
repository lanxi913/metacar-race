# -*- coding: utf-8 -*-
"""后退到车道 -> 朝北对准 -> 提交信号 Key（港口迷雾 Signal_A）。"""
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

    def ctrl(self, vid, throttle=0.0, brake=0.0, steering=0.0, gear=2):
        return self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                          "vehicle_control": {"gear": gear, "throttle": throttle,
                                              "brake": brake, "steering": steering}})

    def pose(self, vid):
        r = self.ctrl(vid, brake=1.0, gear=3)
        yaw = (float(r.get("rotation_y") or 0.0) + 180.0) % 360.0
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        return float(p.get("x", 0.0)), float(p.get("y", 0.0)), yaw, o

    def hold(self, vid, secs, thr, steer, gear):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, throttle=thr, steering=steer, gear=gear)
            time.sleep(0.07)
        self.ctrl(vid, brake=1.0, gear=3)
        time.sleep(0.12)


c = C()
vid = "Car_D"
x, y, yaw, o = c.pose(vid)
print(f"起点 ({x:.1f},{y:.1f}) 航向={yaw:.1f}", flush=True)

# 1) 倒车向南退到车道
for k in range(3):
    c.hold(vid, 1.0, 0.45, 0.0, 4)
    x, y, yaw, o = c.pose(vid)
    print(f"  倒车{k}: ({x:.1f},{y:.1f})", flush=True)
    if y < 119.5:
        break

# 2) 朝北（+y，航向 90）
for k in range(8):
    x, y, yaw, o = c.pose(vid)
    err = wrap180(90.0 - yaw)
    print(f"  对准{k}: ({x:.1f},{y:.1f}) 航向={yaw:6.1f} 误差={err:+6.1f}", flush=True)
    if abs(err) <= 15:
        break
    sgn = 1.0 if err > 0 else -1.0
    c.hold(vid, 0.55, 0.35, sgn * 0.9, 2)
    c.hold(vid, 0.55, 0.35, -sgn * 0.9, 4)

x, y, yaw, o = c.pose(vid)
objs = o.get("visible_objects") or []
print(f"最终 ({x:.1f},{y:.1f}) 航向={yaw:.1f} 可见={[b.get('object_id') for b in objs]}", flush=True)
for ob in objs:
    print("  ", json.dumps(ob, ensure_ascii=False)[:260], flush=True)
for act in ("submit_key", "report_observation"):
    r = c.call({"command": "interact", "action": act, "object_id": "Signal_A", "vehicle_id": vid})
    print(f"{act}: ok={r.get('ok')} msg={str(r.get('message'))[:90]}", flush=True)
    time.sleep(0.3)
sc = c.call({"command": "get_task"}).get("score") or {}
print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)
