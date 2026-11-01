# -*- coding: utf-8 -*-
"""港口迷雾驾驶原语 + 信号任务。

原语：
  y_turn(vid, heading)  三点掉头/对准到指定航向（用回包 rotation_y 做即时航向）
  follow_lane(vid, ...) 沿车道线走，用横向误差纠偏
  submit_signal(...)    停在车道、朝北对着信号、submit_key
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import time


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class Car:
    def __init__(self) -> None:
        self.s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
        self.s.settimeout(10)
        self.buf = b""

    def call(self, p: dict) -> dict:
        self.s.sendall((json.dumps(p) + "\n").encode())
        while b"\n" not in self.buf:
            c = self.s.recv(1 << 20)
            if not c:
                raise ConnectionError
            self.buf += c
        line, rest = self.buf.split(b"\n", 1)
        self.buf = rest
        return json.loads(line.decode("utf-8", "replace"))

    def ctrl(self, vid: str, throttle=0.0, brake=0.0, steering=0.0, gear=2) -> dict:
        return self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                          "vehicle_control": {"gear": gear, "throttle": throttle,
                                              "brake": brake, "steering": steering}})

    def pose(self, vid: str):
        r = self.ctrl(vid, brake=1.0, gear=3)     # 顺手拿即时 yaw
        yaw_raw = float(r.get("rotation_y") or 0.0)
        yaw = (yaw_raw + 180.0) % 360.0        # 实测：车身航向 = rotation_y + 180
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        return float(p.get("x", 0.0)), float(p.get("y", 0.0)), yaw, o

    def score(self) -> dict:
        return self.call({"command": "get_task"}).get("score") or {}

    def interact(self, action: str, oid: str, vid: str) -> dict:
        return self.call({"command": "interact", "action": action,
                          "object_id": oid, "vehicle_id": vid})

    # ---------- 原语 ----------
    def arc(self, vid: str, secs: float, thr: float, steer: float, gear: int) -> None:
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, throttle=thr, steering=steer, gear=gear)
            time.sleep(0.07)
        self.ctrl(vid, brake=1.0, gear=3)
        time.sleep(0.15)

    def y_turn(self, vid: str, target: float, tol: float = 12.0, rounds: int = 12) -> bool:
        """三点掉头：前进满舵 / 倒车反舵 交替，直到航向对齐。"""
        for k in range(rounds):
            x, y, yaw, _ = self.pose(vid)
            err = wrap180(target - yaw)
            print(f"    掉头轮{k}: ({x:.1f},{y:.1f}) yaw={yaw:6.1f} 误差={err:+6.1f}", flush=True)
            if abs(err) <= tol:
                return True
            sgn = 1.0 if err > 0 else -1.0
            self.arc(vid, 0.9, 0.40, sgn * 1.0, 2)      # 前进满舵
            x2, y2, yaw2, _ = self.pose(vid)
            err2 = wrap180(target - yaw2)
            if abs(err2) <= tol:
                return True
            if abs(err2) > abs(err) - 5.0:              # 前进没改善 -> 倒车反舵
                self.arc(vid, 0.9, 0.40, -sgn * 1.0, 4)
        return False

    def drive_heading(self, vid: str, heading: float, secs: float,
                      thr: float = 0.28, gain: float = 45.0, lon: float = 0.55) -> None:
        """沿固定航向行驶，横向偏差用位置纠偏（lon 为车道 y 或 x 的目标值）。"""
        t0 = time.time()
        while time.time() - t0 < secs:
            r = self.ctrl(vid, throttle=thr, steering=0.0, gear=2)
            yaw = (float(r.get("rotation_y") or 0.0) + 180.0) % 360.0
            o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
            p = o.get("position") or {}
            x, y = float(p.get("x", 0.0)), float(p.get("y", 0.0))
            # 目标航向修正：横向偏差 -> 期望航向偏移
            if abs(math.cos(math.radians(heading))) > 0.5:      # 东西向
                lat = y - lon
                corr = -lat * 2.0
            else:                                               # 南北向
                lat = x - lon
                corr = -lat * 2.0
            want = heading + max(-25.0, min(25.0, corr))
            err = wrap180(want - yaw)
            steer = max(-0.7, min(0.7, err / gain))
            self.ctrl(vid, throttle=thr, steering=steer, gear=2)
            time.sleep(0.07)
        self.ctrl(vid, brake=1.0, gear=3)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_D")
    ap.add_argument("--tx", type=float, required=True)
    ap.add_argument("--ty", type=float, required=True)
    ap.add_argument("--object", default="Signal_A")
    args = ap.parse_args()

    car = Car()
    vid = args.vehicle
    x, y, yaw, o = car.pose(vid)
    print(f"{vid} ({x:.1f},{y:.1f}) yaw={yaw:.1f}", flush=True)
    print(f"目标 ({args.tx:.1f},{args.ty:.1f})", flush=True)

    # 目标航向：朝西（目标在车西侧）
    want = math.degrees(math.atan2(args.ty - y, args.tx - x))
    print(f"期望航向 {want:.0f}°", flush=True)
    ok = car.y_turn(vid, want)
    x, y, yaw, o = car.pose(vid)
    print(f"对准={ok} ({x:.1f},{y:.1f}) yaw={yaw:.1f}", flush=True)

    # 直行接近
    t0 = time.time()
    while time.time() - t0 < 60:
        x, y, yaw, o = car.pose(vid)
        d = math.hypot(args.tx - x, args.ty - y)
        if d < 3.0:
            break
        err = wrap180(want - yaw)
        steer = max(-0.6, min(0.6, err / 55.0))
        car.ctrl(vid, throttle=0.26 if d > 8 else 0.18, steering=steer, gear=2)
        time.sleep(0.07)
    car.ctrl(vid, brake=1.0, gear=3)
    x, y, yaw, o = car.pose(vid)
    print(f"到位 ({x:.1f},{y:.1f}) d={math.hypot(args.tx-x, args.ty-y):.1f} "
          f"可见={[b.get('object_id') for b in (o.get('visible_objects') or [])]}", flush=True)
    for act in ("submit_key", "report_observation"):
        r = car.interact(act, args.object, vid)
        print(f"{act}: ok={r.get('ok')} msg={str(r.get('message'))[:80]}", flush=True)
    sc = car.score()
    print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
