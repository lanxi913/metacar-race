# -*- coding: utf-8 -*-
"""对准指定航向后读视觉并提交 Key。"""
from __future__ import annotations

import argparse
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

    def arc(self, vid, secs, thr, steer, gear):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, throttle=thr, steering=steer, gear=gear)
            time.sleep(0.07)
        self.ctrl(vid, brake=1.0, gear=3)
        time.sleep(0.12)


ap = argparse.ArgumentParser()
ap.add_argument("--vehicle", default="Car_D")
ap.add_argument("--object", default="Signal_A")
ap.add_argument("--heading", type=float, default=None, help="目标航向；不给就朝信号")
ap.add_argument("--tx", type=float, default=587.5)
ap.add_argument("--ty", type=float, default=124.8)
args = ap.parse_args()

c = C()
vid = args.vehicle
x, y, yaw, o = c.pose(vid)
want = args.heading if args.heading is not None else math.degrees(math.atan2(args.ty - y, args.tx - x))
print(f"{vid} ({x:.1f},{y:.1f}) 航向={yaw:.1f} 目标航向={want:.1f}", flush=True)

for k in range(10):
    x, y, yaw, o = c.pose(vid)
    err = wrap180(want - yaw)
    objs = [b.get("object_id") for b in (o.get("visible_objects") or [])]
    print(f"  轮{k}: ({x:.1f},{y:.1f}) 航向={yaw:6.1f} 误差={err:+6.1f} 可见={objs}", flush=True)
    if abs(err) <= 12 and args.object in objs:
        break
    sgn = 1.0 if err > 0 else -1.0
    if abs(err) > 120:
        c.arc(vid, 0.8, 0.40, sgn * 1.0, 2)
        c.arc(vid, 0.8, 0.40, -sgn * 1.0, 4)
    else:
        c.arc(vid, 0.5, 0.30, sgn * 0.8, 2)

x, y, yaw, o = c.pose(vid)
objs = o.get("visible_objects") or []
print(f"最终 ({x:.1f},{y:.1f}) 航向={yaw:.1f} 可见={[b.get('object_id') for b in objs]}", flush=True)
for ob in objs:
    if ob.get("object_id") == args.object:
        print("  ", json.dumps(ob, ensure_ascii=False)[:300], flush=True)
for act in ("submit_key", "report_observation"):
    r = c.call({"command": "interact", "action": act, "object_id": args.object, "vehicle_id": vid})
    print(f"{act}: ok={r.get('ok')} msg={str(r.get('message'))[:90]}", flush=True)
    time.sleep(0.3)
sc = c.call({"command": "get_task"}).get("score") or {}
print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)
