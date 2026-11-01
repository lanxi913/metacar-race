# -*- coding: utf-8 -*-
"""用指定车辆接近信号并提交 Key（含对准与距离打印）。"""
from __future__ import annotations

import argparse
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_A")
    ap.add_argument("--object", default="Signal_A")
    ap.add_argument("--tx", type=float, required=True)
    ap.add_argument("--ty", type=float, required=True)
    ap.add_argument("--tol", type=float, default=4.5)
    ap.add_argument("--secs", type=float, default=40.0)
    args = ap.parse_args()

    p = P()
    vid = args.vehicle
    x, y, o = p.obs(vid)
    print(f"{vid} ({x:.1f},{y:.1f}) -> ({args.tx},{args.ty})", flush=True)
    t0 = time.time()
    while time.time() - t0 < args.secs:
        x, y, o = p.obs(vid)
        d = math.hypot(args.tx - x, args.ty - y)
        if d <= args.tol:
            p.ctrl(vid, brake=1.0, gear=3)
            print(f"到位 ({x:.1f},{y:.1f}) d={d:.1f}", flush=True)
            break
        if p.head is None:
            p.ctrl(vid, throttle=0.30, steering=0.0, gear=2)
        else:
            want = math.degrees(math.atan2(args.ty - y, args.tx - x))
            err = wrap180(want - p.head)
            steer = max(-0.6, min(0.6, err / 80.0))
            thr = 0.24 if abs(err) < 60 else 0.18
            p.ctrl(vid, throttle=thr, steering=steer, gear=2)
        time.sleep(0.07)
    p.ctrl(vid, brake=1.0, gear=3)
    time.sleep(0.3)
    x, y, o = p.obs(vid)
    d = math.hypot(args.tx - x, args.ty - y)
    print(f"最终 ({x:.1f},{y:.1f}) d={d:.1f} 可见={len(o.get('visible_objects') or [])}", flush=True)
    for ob in (o.get("visible_objects") or []):
        print("  ", json.dumps(ob, ensure_ascii=False)[:260], flush=True)
    for act in ("submit_key", "report_observation"):
        r = p.call({"command": "interact", "action": act, "object_id": args.object, "vehicle_id": vid})
        print(f"{act}: ok={r.get('ok')} msg={str(r.get('message'))[:80]}", flush=True)
        time.sleep(0.3)
    sc = p.call({"command": "get_task"}).get("score") or {}
    print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
