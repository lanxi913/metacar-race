# -*- coding: utf-8 -*-
"""港口迷雾 mission：按路网导航 + 沿途记录视觉 + 到达后尝试交互。

用法（先人工启动场景，再运行）：
    python tools/portfog_mission.py --vehicle Car_D --tx 631 --ty -66 --seconds 180
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
sys.path.insert(0, str(REPO / "tools"))

from portfog_route import RoadMap  # noqa: E402


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class Mission:
    def __init__(self) -> None:
        self.sock = socket.create_connection(("127.0.0.1", 9100), timeout=3)
        self.sock.settimeout(10)
        self.buf = b""
        self.pts: list[tuple[float, float]] = []
        self.head: float | None = None
        self.seen: dict[str, dict] = {}

    def call(self, p: dict) -> dict:
        self.sock.sendall((json.dumps(p) + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                raise ConnectionError("closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8", "replace"))

    def ctrl(self, vid: str, throttle=0.0, brake=0.0, steering=0.0, gear=2) -> None:
        self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                   "vehicle_control": {"gear": gear, "throttle": throttle,
                                       "brake": brake, "steering": steering}})

    def obs(self, vid: str) -> tuple[float, float, dict]:
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        x, y = float(p.get("x", 0.0)), float(p.get("y", 0.0))
        self.pts.append((x, y))
        if len(self.pts) > 60:
            self.pts.pop(0)
        for bx, by in reversed(self.pts[:-1]):
            if math.hypot(x - bx, y - by) > 2.0:
                h = math.degrees(math.atan2(y - by, x - bx))
                self.head = h if self.head is None else self.head + 0.7 * wrap180(h - self.head)
                break
        for ob in (o.get("visible_objects") or []):
            oid = ob.get("object_id") or str(ob)[:20]
            if oid not in self.seen:
                self.seen[oid] = ob
                print(f"  [发现] {json.dumps(ob, ensure_ascii=False)[:240]}", flush=True)
        return x, y, o

    def score(self) -> dict:
        return self.call({"command": "get_task"}).get("score") or {}

    def interact(self, action: str, object_id: str, vid: str) -> dict:
        return self.call({"command": "interact", "action": action,
                          "object_id": object_id, "vehicle_id": vid})

    def unstick(self, vid: str, secs: float = 1.8) -> None:
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, throttle=0.5, steering=-0.8, gear=4)
            time.sleep(0.08)
        self.ctrl(vid, brake=1.0, gear=3)
        time.sleep(0.2)

    def follow(self, vid: str, waypoints: list[tuple[float, float]],
               tol: float = 6.0, deadline: float = 240.0, thr: float = 0.32) -> bool:
        t0 = time.time()
        i = 0
        stall_t = time.time()
        best_d = 1e9
        while i < len(waypoints) and time.time() - t0 < deadline:
            tx, ty = waypoints[i]
            x, y, o = self.obs(vid)
            d = math.hypot(tx - x, ty - y)
            if d <= tol:
                i += 1
                print(f"    [ok] 航点 {i}/{len(waypoints)} ({x:.1f},{y:.1f})", flush=True)
                stall_t = time.time()
                best_d = 1e9
                continue
            if d < best_d - 0.8:
                best_d, stall_t = d, time.time()
            if time.time() - stall_t > 3.5:
                print(f"    !! 卡住 ({x:.1f},{y:.1f}) 目标 {i+1} d={d:.1f}，脱困", flush=True)
                self.unstick(vid)
                stall_t, best_d = time.time(), 1e9
                continue
            if self.head is None:
                self.ctrl(vid, throttle=0.25, steering=0.0, gear=2)
            else:
                want = math.degrees(math.atan2(ty - y, tx - x))
                err = wrap180(want - self.head)
                steer = max(-0.7, min(0.7, err / 60.0))
                t = thr if abs(err) < 45 else 0.20
                if d < 12:
                    t = min(t, 0.26)
                self.ctrl(vid, throttle=t, steering=steer, gear=2)
            time.sleep(0.06)
        self.ctrl(vid, brake=1.0, gear=3)
        return i >= len(waypoints)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_D")
    ap.add_argument("--tx", type=float, required=True)
    ap.add_argument("--ty", type=float, required=True)
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--tol", type=float, default=6.0)
    args = ap.parse_args()

    rm = RoadMap()
    m = Mission()
    vid = args.vehicle
    x, y, o = m.obs(vid)
    print(f"{vid} ({x:.1f},{y:.1f}) -> ({args.tx},{args.ty})", flush=True)
    wps, total = rm.route((x, y), (args.tx, args.ty))
    print(f"路线 {len(wps)} 点 / {total:.0f} m", flush=True)
    ok = m.follow(vid, wps, tol=args.tol, deadline=args.seconds)
    x, y, o = m.obs(vid)
    print(f"\n结束 ({x:.1f},{y:.1f}) ok={ok}", flush=True)
    print("发现:", list(m.seen.keys()), flush=True)
    print("score:", json.dumps(m.score(), ensure_ascii=False)[:260], flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
