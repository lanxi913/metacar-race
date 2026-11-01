# -*- coding: utf-8 -*-
"""港口迷雾 Signal_A —— 纯跟踪（carrot）+ 位移航向，沿车道 y=120.78 向西。

定标事实：
  rotation_y ≈ 0（直线行驶时），**不是航向**，只能用位移差算
  gear 2=前进 3=驻车 4=倒车；steering +1 左 / -1 右
"""
from __future__ import annotations

import json
import math
import socket
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar_multiagent import ScenarioTaskAPI  # noqa: E402

LANE_Y = 120.78
SIG = (587.47, 124.78)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Raw:
    def __init__(self):
        self.s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
        self.s.settimeout(10)
        self.buf = b""
        self.pts = []
        self.head = None
        self.moved = 0.0

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

    def st(self, vid):
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        x, y = float(p.get("x", 0.0)), float(p.get("y", 0.0))
        self.pts.append((x, y))
        if len(self.pts) > 50:
            self.pts.pop(0)
        for bx, by in reversed(self.pts[:-1]):
            if math.hypot(x - bx, y - by) > 1.5:
                h = math.degrees(math.atan2(y - by, x - bx))
                self.head = h if self.head is None else self.head + 0.5 * wrap180(h - self.head)
                break
        return x, y, o


def main():
    raw = Raw()
    vid = "Car_C"
    x, y, o = raw.st(vid)
    print(f"{vid} ({x:.1f},{y:.1f})", flush=True)

    t0 = time.time()
    last_log = 0.0
    while time.time() - t0 < 150:
        x, y, o = raw.st(vid)
        if x <= SIG[0] + 0.5:
            break
        # carrot：前方 8 m、钉在车道上
        cx = x - 8.0
        cy = LANE_Y
        if raw.head is None:
            raw.ctrl(vid, thr=0.30, st=0.0, gear=2)
            time.sleep(0.07)
            continue
        want = math.degrees(math.atan2(cy - y, cx - x))
        err = wrap180(want - raw.head)
        steer = max(-0.7, min(0.7, err / 50.0))
        thr = 0.28 if abs(err) < 30 else 0.16
        raw.ctrl(vid, thr=thr, st=steer, gear=2)
        if time.time() - last_log > 3.0:
            last_log = time.time()
            print(f"  ({x:7.1f},{y:7.1f}) 航向={raw.head if raw.head is None else round(raw.head,1)} "
                  f"横向={y-LANE_Y:+5.2f} 期望={want:6.1f} err={err:+6.1f}", flush=True)
        time.sleep(0.07)
    raw.ctrl(vid, brk=1.0, gear=3)
    x, y, o = raw.st(vid)
    print(f"西行结束 ({x:.1f},{y:.1f}) 距信号 {math.hypot(SIG[0]-x, SIG[1]-y):.1f} m", flush=True)

    with ScenarioTaskAPI() as api:
        obs = api.get_observation(vid)
        print("可见:", [b.object_id for b in obs.visible_objects], flush=True)
        tob = next((b for b in obs.visible_objects if b.object_id == "Signal_A"), None)
        if tob:
            print("token:", repr(tob.observation_token), "hint:", tob.hint, flush=True)
            r = api.interact(vid, "Signal_A", "submit_key", {"key": tob.observation_token})
            print(f"submit granted={r.granted} msg={r.message}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
