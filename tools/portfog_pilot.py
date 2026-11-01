# -*- coding: utf-8 -*-
"""低速纯跟踪接近目标并交互（港口迷雾）。

控制要点（实测）：
  gear 2=前进 3=驻车 4=倒车；steering +1 左 / -1 右
  控制必须嵌套 vehicle_control
  航向用 1 m 位移基线 + EMA，低速(0.22 油门)下滞后小
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import time


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Pilot:
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
        if len(self.pts) > 30:
            self.pts.pop(0)
        for bx, by in reversed(self.pts[:-1]):
            if math.hypot(x - bx, y - by) > 1.0:
                h = math.degrees(math.atan2(y - by, x - bx))
                self.head = h if self.head is None else self.head + 0.6 * wrap180(h - self.head)
                break
        return x, y, o

    def score(self):
        return self.call({"command": "get_task"}).get("score") or {}

    def interact(self, action, oid, vid):
        return self.call({"command": "interact", "action": action,
                          "object_id": oid, "vehicle_id": vid})

    def goto(self, vid, tx, ty, tol=5.0, thr=0.24, deadline=90.0):
        t0 = time.time()
        stall = time.time()
        best = 1e9
        while time.time() - t0 < deadline:
            x, y, o = self.obs(vid)
            d = math.hypot(tx - x, ty - y)
            if d <= tol:
                self.ctrl(vid, brake=1.0, gear=3)
                return True, d
            if d < best - 0.5:
                best, stall = d, time.time()
            if time.time() - stall > 4.0:
                print(f"    ! 卡住 ({x:.1f},{y:.1f}) d={d:.1f}，倒车", flush=True)
                t1 = time.time()
                while time.time() - t1 < 1.8:
                    self.ctrl(vid, throttle=0.5, steering=-0.9, gear=4)
                    time.sleep(0.08)
                self.ctrl(vid, brake=1.0, gear=3)
                time.sleep(0.3)
                stall, best = time.time(), 1e9
                continue
            if self.head is None:
                self.ctrl(vid, throttle=0.2, steering=0.0, gear=2)
            else:
                want = math.degrees(math.atan2(ty - y, tx - x))
                err = wrap180(want - self.head)
                steer = max(-0.5, min(0.5, err / 70.0))
                self.ctrl(vid, throttle=thr, steering=steer, gear=2)
            time.sleep(0.06)
        self.ctrl(vid, brake=1.0, gear=3)
        return False, math.hypot(tx - x, ty - y)


def find_obj(o, oid):
    for ob in (o.get("visible_objects") or []):
        if ob.get("object_id") == oid:
            p = ob.get("position") or {}
            return float(p.get("x", 0)), float(p.get("y", 0)), ob
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_C")
    ap.add_argument("--object", default="Signal_A")
    ap.add_argument("--x", type=float, default=None)
    ap.add_argument("--y", type=float, default=None)
    ap.add_argument("--action", default="submit_key")
    ap.add_argument("--tol", type=float, default=5.0)
    args = ap.parse_args()

    p = Pilot()
    vid = args.vehicle
    x, y, o = p.obs(vid)
    print(f"{vid} at ({x:.1f},{y:.1f})", flush=True)

    tx, ty = args.x, args.y
    ob = find_obj(o, args.object)
    if ob:
        tx, ty = ob[0], ob[1]
        print(f"视野中 {args.object} at ({tx:.1f},{ty:.1f}) d={ob[2].get('distance_m'):.1f}", flush=True)
    if tx is None or ty is None:
        print("!! 没有目标坐标，也没在视野里", flush=True)
        return 2

    ok, d = p.goto(vid, tx, ty, tol=args.tol)
    x, y, o = p.obs(vid)
    print(f"到位={ok} 最后距离={d:.1f} 位置=({x:.1f},{y:.1f})", flush=True)

    r = p.interact(args.action, args.object, vid)
    print(f"interact {args.action} {args.object}: ok={r.get('ok')} msg={str(r.get('message'))[:90]}",
          flush=True)
    o = p.obs(vid)[2]
    sc = p.score()
    print(f"sig={o.get('verified_signal_count')}/{o.get('total_signal_count')} "
          f"score={sc.get('total_score')} penalties={sc.get('penalties')}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
