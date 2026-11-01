# -*- coding: utf-8 -*-
"""原地转向对准目标，再读观测 / 提交 Key。"""
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

    def turn_to(self, vid, want, tol=12.0, deadline=25.0):
        """低速边走边转到目标航向（没有原地转向，就用最小可行动作）。"""
        t0 = time.time()
        while time.time() - t0 < deadline:
            x, y, o = self.obs(vid)
            if self.head is None:
                self.ctrl(vid, throttle=0.30, steering=0.7, gear=2)
                time.sleep(0.08)
                continue
            err = wrap180(want - self.head)
            if abs(err) <= tol:
                self.ctrl(vid, brake=1.0, gear=3)
                return True, err
            steer = max(-1.0, min(1.0, err / 35.0))
            self.ctrl(vid, throttle=0.30, steering=steer, gear=2)
            time.sleep(0.08)
        self.ctrl(vid, brake=1.0, gear=3)
        return False, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_C")
    ap.add_argument("--object", default="Signal_A")
    ap.add_argument("--tx", type=float, required=True)
    ap.add_argument("--ty", type=float, required=True)
    args = ap.parse_args()

    p = P()
    vid = args.vehicle
    x, y, o = p.obs(vid)
    want = math.degrees(math.atan2(args.ty - y, args.tx - x))
    print(f"{vid} ({x:.1f},{y:.1f}) -> 目标 ({args.tx},{args.ty}) 航向 {want:.0f}°", flush=True)
    ok, err = p.turn_to(vid, want)
    x, y, o = p.obs(vid)
    print(f"对准={ok} 残余={err} 位置=({x:.1f},{y:.1f})", flush=True)
    objs = o.get("visible_objects") or []
    print(f"可见 {len(objs)}", flush=True)
    for ob in objs:
        print("  ", json.dumps(ob, ensure_ascii=False)[:300], flush=True)
    r = p.call({"command": "interact", "action": "submit_key",
                "object_id": args.object, "vehicle_id": vid})
    print("submit_key:", r.get("ok"), str(r.get("message"))[:90], flush=True)
    r = p.call({"command": "interact", "action": "scan",
                "object_id": args.object, "vehicle_id": vid})
    print("scan:", r.get("ok"), str(r.get("message"))[:90], flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
