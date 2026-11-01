# -*- coding: utf-8 -*-
"""用回包 rotation_y 作即时航向的控制器（无滞后），接近目标并交互。"""
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
        self.yaw = None

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
        r = self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                       "vehicle_control": {"gear": gear, "throttle": throttle,
                                           "brake": brake, "steering": steering}})
        ry = r.get("rotation_y")
        if isinstance(ry, (int, float)):
            self.yaw = float(ry)
        return r

    def obs(self, vid):
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        return float(p.get("x", 0.0)), float(p.get("y", 0.0)), o


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_D")
    ap.add_argument("--object", default="Signal_A")
    ap.add_argument("--tx", type=float, required=True)
    ap.add_argument("--ty", type=float, required=True)
    ap.add_argument("--tol", type=float, default=4.0)
    ap.add_argument("--secs", type=float, default=45.0)
    ap.add_argument("--thr", type=float, default=0.26)
    args = ap.parse_args()

    p = P()
    vid = args.vehicle
    x, y, o = p.obs(vid)
    print(f"{vid} ({x:.1f},{y:.1f}) -> ({args.tx},{args.ty})", flush=True)
    t0 = time.time()
    n = 0
    while time.time() - t0 < args.secs:
        x, y, o = p.obs(vid)
        d = math.hypot(args.tx - x, args.ty - y)
        if d <= args.tol:
            p.ctrl(vid, brake=1.0, gear=3)
            print(f"到位 ({x:.1f},{y:.1f}) d={d:.1f}", flush=True)
            break
        yaw = p.yaw if p.yaw is not None else 0.0
        want = math.degrees(math.atan2(args.ty - y, args.tx - x))
        err = wrap180(want - yaw)
        steer = max(-0.7, min(0.7, err / 45.0))
        thr = args.thr if abs(err) < 50 else 0.20
        p.ctrl(vid, throttle=thr, steering=steer, gear=2)
        if n % 20 == 0:
            print(f"   ({x:7.1f},{y:7.1f}) d={d:5.1f} yaw={yaw:6.1f} want={want:6.1f} err={err:+6.1f}",
                  flush=True)
        n += 1
        time.sleep(0.07)
    p.ctrl(vid, brake=1.0, gear=3)
    time.sleep(0.3)
    x, y, o = p.obs(vid)
    print(f"最终 ({x:.1f},{y:.1f}) 偏差 {math.hypot(args.tx-x, args.ty-y):.1f} "
          f"可见={[b.get('object_id') for b in (o.get('visible_objects') or [])]}", flush=True)
    for act in ("submit_key", "report_observation"):
        r = p.call({"command": "interact", "action": act, "object_id": args.object, "vehicle_id": vid})
        print(f"{act}: ok={r.get('ok')} msg={str(r.get('message'))[:80]}", flush=True)
        time.sleep(0.3)
    sc = p.call({"command": "get_task"}).get("score") or {}
    print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
