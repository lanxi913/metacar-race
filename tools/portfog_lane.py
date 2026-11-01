# -*- coding: utf-8 -*-
"""港口迷雾：沿 mapd 车道走廊的纯跟踪（低速，求不撞墙）。

- 车道几何来自场景资源 mapd
- 前视点 5 m、速度 throttle 0.20、每 0.07 s 重算
- 横向误差 + 航向误差合成转向；卡住就倒车脱困
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

from portfog_route import RoadMap  # noqa: E402


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

    def score(self):
        return self.call({"command": "get_task"}).get("score") or {}


def project(lane, x, y):
    ax, ay = lane["a"]
    bx, by = lane["b"]
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
    px, py = ax + t * vx, ay + t * vy
    cross = ((x - px) * (-vy) + (y - py) * vx) / (math.sqrt(L2) + 1e-9)   # 左正
    return t, math.hypot(x - px, y - py), cross, (px, py)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_C")
    ap.add_argument("--tx", type=float, required=True)
    ap.add_argument("--ty", type=float, required=True)
    ap.add_argument("--seconds", type=float, default=150.0)
    args = ap.parse_args()

    rm = RoadMap()
    car = Car()
    vid = args.vehicle
    x, y, o = car.st(vid)
    print(f"{vid} ({x:.1f},{y:.1f}) -> ({args.tx},{args.ty})", flush=True)

    # 路由：给一串车道端点，逐段跟
    wps, total = rm.route((x, y), (args.tx, args.ty))
    print(f"路线 {len(wps)} 点 / {total:.0f} m", flush=True)
    if not wps:
        print("!! 没路由", flush=True)
        return 2

    seg = 0
    t0 = time.time()
    stall_t = time.time()
    best = 1e9
    last_log = 0.0
    while seg < len(wps) and time.time() - t0 < args.seconds:
        x, y, o = car.st(vid)
        tx, ty = wps[seg]
        d = math.hypot(tx - x, ty - y)
        if d < 4.0:
            seg += 1
            stall_t, best = time.time(), 1e9
            print(f"  [ok] 航点 {seg}/{len(wps)}", flush=True)
            continue
        if d < best - 0.6:
            best, stall_t = d, time.time()
        if time.time() - stall_t > 4.0:
            print(f"  !! 卡住 ({x:.1f},{y:.1f})，倒车", flush=True)
            t1 = time.time()
            while time.time() - t1 < 1.8:
                car.ctrl(vid, thr=0.45, st=-0.9, gear=4)
                time.sleep(0.07)
            car.ctrl(vid, brk=1.0, gear=3)
            time.sleep(0.3)
            stall_t, best = time.time(), 1e9
            continue
        if car.head is None:
            car.ctrl(vid, thr=0.25, st=0.0, gear=2)
            time.sleep(0.07)
            continue
        want = math.degrees(math.atan2(ty - y, tx - x))
        err = wrap180(want - car.head)
        steer = max(-0.55, min(0.55, err / 60.0))
        thr = 0.22 if abs(err) < 35 else 0.15
        car.ctrl(vid, thr=thr, st=steer, gear=2)
        if time.time() - last_log > 4.0:
            last_log = time.time()
            print(f"  ({x:7.1f},{y:7.1f}) 目标节点{seg} d={d:5.1f} "
                  f"航向={car.head:6.1f} 期望={want:6.1f} err={err:+6.1f}", flush=True)
        time.sleep(0.07)

    car.ctrl(vid, brk=1.0, gear=3)
    x, y, o = car.st(vid)
    print(f"结束 ({x:.1f},{y:.1f}) 距目标 {math.hypot(args.tx-x, args.ty-y):.1f} "
          f"可见={[b.get('object_id') for b in (o.get('visible_objects') or [])]}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
