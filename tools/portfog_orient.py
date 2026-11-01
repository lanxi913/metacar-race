# -*- coding: utf-8 -*-
"""用车尾旋转对准航向（实测：倒车+满舵 ≈ 50°/s，前进几乎不转）。

orient(vid, target) -> 反复倒车打舵直到航向误差 < tol
"""
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

    def pos(self, vid):
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        return float(p.get("x", 0.0)), float(p.get("y", 0.0))

    def orient(self, vid, target, tol=10.0, rounds=10):
        for k in range(rounds):
            y = self.yaw(vid)
            err = wrap180(target - y)
            x, py = self.pos(vid)
            objs = [b.get("object_id") for b in
                    (self.call({"command": "get_observation", "vehicle_id": vid})
                     .get("observation", {}).get("visible_objects") or [])]
            print(f"  轮{k}: ({x:.1f},{py:.1f}) 航向={y:6.1f} 误差={err:+6.1f} 可见={objs}", flush=True)
            if abs(err) <= tol:
                return True
            secs = min(1.6, max(0.35, abs(err) / 50.0))
            st = 1.0 if err > 0 else -1.0
            t0 = time.time()
            while time.time() - t0 < secs:
                self.ctrl(vid, thr=0.45, st=st, gear=4)     # 倒车打舵
                time.sleep(0.07)
            self.ctrl(vid, brk=1.0, gear=3)
            time.sleep(0.15)
        return False


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_D")
    ap.add_argument("--heading", type=float, required=True)
    ap.add_argument("--then-object", default=None)
    args = ap.parse_args()
    c = C()
    ok = c.orient(args.vehicle, args.heading)
    print("对准:", ok, flush=True)
    if args.then_object:
        for act in ("submit_key", "report_observation"):
            r = c.call({"command": "interact", "action": act,
                        "object_id": args.then_object, "vehicle_id": args.vehicle})
            print(f"{act}: ok={r.get('ok')} msg={str(r.get('message'))[:90]}", flush=True)
        sc = c.call({"command": "get_task"}).get("score") or {}
        print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")},
              flush=True)
