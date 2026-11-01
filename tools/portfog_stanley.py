# -*- coding: utf-8 -*-
"""港口迷雾：Stanley 横向误差闭环 + 倒车掉头，开到目标点并对准后交互。

关键标定（实测）：
  heading = (rotation_y + 180) % 360
  前进满舵几乎不转（-0.5°/1.5s），倒车满舵 +78.8°/1.5s（≈50°/s）
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

    def ctrl(self, vid: str, thr=0.0, brk=0.0, st=0.0, gear=2) -> dict:
        return self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                          "vehicle_control": {"gear": gear, "throttle": thr,
                                              "brake": brk, "steering": st}})

    def state(self, vid: str):
        """返回 (x, y, heading, obs)，heading 已 +180 修正。"""
        r = self.ctrl(vid, brk=1.0, gear=3)
        heading = (float(r.get("rotation_y") or 0.0) + 180.0) % 360.0
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        return float(p.get("x", 0.0)), float(p.get("y", 0.0)), heading, o

    def score(self) -> dict:
        return self.call({"command": "get_task"}).get("score") or {}

    def interact(self, action: str, oid: str, vid: str) -> dict:
        return self.call({"command": "interact", "action": action,
                          "object_id": oid, "vehicle_id": vid})

    def reverse_arc(self, vid: str, secs: float, steer: float) -> None:
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, thr=0.45, st=steer, gear=4)
            time.sleep(0.07)
        self.ctrl(vid, brk=1.0, gear=3)
        time.sleep(0.12)

    def orient(self, vid: str, target: float, tol: float = 10.0, rounds: int = 8) -> bool:
        """用倒车满舵把航向转到目标（前进几乎不转向，只能靠倒车）。"""
        for _ in range(rounds):
            _, _, h, _ = self.state(vid)
            err = wrap180(target - h)
            if abs(err) <= tol:
                return True
            self.reverse_arc(vid, min(1.6, max(0.35, abs(err) / 50.0)),
                             1.0 if err > 0 else -1.0)
        _, _, h, _ = self.state(vid)
        return abs(wrap180(target - h)) <= tol + 5

    def goto(self, vid: str, tx: float, ty: float, tol: float = 3.0,
             thr: float = 0.22, deadline: float = 90.0) -> bool:
        """Stanley：航向误差 + 横向偏差，短步长闭环。"""
        t0 = time.time()
        sx, sy, _, _ = self.state(vid)
        last_log = 0.0
        while time.time() - t0 < deadline:
            x, y, h, o = self.state(vid)
            d = math.hypot(tx - x, ty - y)
            if d <= tol:
                return True
            bearing = math.degrees(math.atan2(ty - sy, tx - sx))
            err_h = wrap180(bearing - h)
            # 横向偏差（左正）
            nx, ny = -math.sin(math.radians(bearing)), math.cos(math.radians(bearing))
            cross = (x - sx) * nx + (y - sy) * ny
            corr = math.degrees(math.atan(0.25 * cross))
            steer = max(-1.0, min(1.0, (err_h + corr) / 40.0))
            if abs(err_h) > 55:
                # 大角度：先倒车甩头
                self.orient(vid, math.degrees(math.atan2(ty - y, tx - x)))
                sx, sy, _, _ = self.state(vid)
                continue
            t = thr if abs(err_h) < 25 else 0.18
            if d < 8:
                t = min(t, 0.16)
            self.ctrl(vid, thr=t, st=steer, gear=2)
            if time.time() - last_log > 4.0:
                last_log = time.time()
                print(f"    ({x:7.1f},{y:7.1f}) d={d:5.1f} h={h:6.1f} 偏={cross:+5.2f} "
                      f"err={err_h:+6.1f} steer={steer:+.2f}", flush=True)
            time.sleep(0.07)
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_C")
    ap.add_argument("--object", default="Signal_A")
    ap.add_argument("--tx", type=float, required=True)
    ap.add_argument("--ty", type=float, required=True)
    ap.add_argument("--face", type=float, default=None, help="到位后要对的航向")
    ap.add_argument("--action", default="submit_key")
    args = ap.parse_args()

    car = Car()
    vid = args.vehicle
    x, y, h, o = car.state(vid)
    print(f"{vid} ({x:.1f},{y:.1f}) 航向={h:.1f} -> ({args.tx},{args.ty})", flush=True)
    ok = car.goto(vid, args.tx, args.ty)
    x, y, h, o = car.state(vid)
    d = math.hypot(args.tx - x, args.ty - y)
    print(f"到位={ok} ({x:.1f},{y:.1f}) 偏差={d:.1f} 航向={h:.1f} "
          f"可见={[b.get('object_id') for b in (o.get('visible_objects') or [])]}", flush=True)

    if args.face is not None:
        ok2 = car.orient(vid, args.face)
        x, y, h, o = car.state(vid)
        print(f"对准={ok2} 航向={h:.1f} 可见={[b.get('object_id') for b in (o.get('visible_objects') or [])]}",
              flush=True)
        for ob in (o.get("visible_objects") or []):
            if ob.get("object_id") == args.object:
                print("   ", json.dumps(ob, ensure_ascii=False)[:280], flush=True)

    r = car.interact(args.action, args.object, vid)
    print(f"{args.action}: ok={r.get('ok')} msg={str(r.get('message'))[:90]}", flush=True)
    sc = car.score()
    print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
