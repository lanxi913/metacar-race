# -*- coding: utf-8 -*-
"""原始协议控制（已标定 gear 2=前进 / heading=yaw+180）+ SDK 读数据。

目标：开到 Signal_A 6 m 内，把观测全字段打出来找验证码，并试 submit_key。
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


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Raw:
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

    def st(self, vid):
        r = self.ctrl(vid, brk=1.0, gear=3)
        h = (float(r.get("rotation_y") or 0.0) + 180.0) % 360.0
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        return float(p.get("x", 0.0)), float(p.get("y", 0.0)), h, o

    def rev(self, vid, secs, steer):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, thr=0.45, st=steer, gear=4)
            time.sleep(0.07)
        self.ctrl(vid, brk=1.0, gear=3)
        time.sleep(0.12)

    def orient(self, vid, target, tol=10.0, rounds=10):
        for _ in range(rounds):
            _, _, h, _ = self.st(vid)
            err = wrap180(target - h)
            if abs(err) <= tol:
                return True
            self.rev(vid, min(1.4, max(0.3, abs(err) / 50.0)), 1.0 if err > 0 else -1.0)
        return False


def main():
    raw = Raw()
    vid = "Car_C"
    SIG = None
    with ScenarioTaskAPI() as api:
        task = api.get_task()
        sig = next(o for o in task.shared_task_objects if o.object_type == "signal")
        SIG = (float(sig.position.x), float(sig.position.y))
        print("信号:", sig.object_id, SIG, flush=True)

    x, y, h, o = raw.st(vid)
    print(f"{vid} ({x:.1f},{y:.1f}) h={h:.1f}", flush=True)

    for step in range(20):
        x, y, h, o = raw.st(vid)
        d = math.hypot(SIG[0] - x, SIG[1] - y)
        print(f"[{step}] ({x:.1f},{y:.1f}) h={h:6.1f} d={d:5.1f} "
              f"可见={[b.get('object_id') for b in (o.get('visible_objects') or [])]}", flush=True)
        if d <= 6.0:
            break
        want = math.degrees(math.atan2(SIG[1] - y, SIG[0] - x))
        err = wrap180(want - h)
        if abs(err) > 30:
            raw.orient(vid, want)
            continue
        t0 = time.time()
        while time.time() - t0 < 1.0:
            raw.ctrl(vid, thr=0.30, st=max(-0.6, min(0.6, err / 50.0)), gear=2)
            time.sleep(0.06)
        raw.ctrl(vid, brk=1.0, gear=3)

    x, y, h, o = raw.st(vid)
    print(f"到达 ({x:.1f},{y:.1f}) d={math.hypot(SIG[0]-x, SIG[1]-y):.1f}", flush=True)

    with ScenarioTaskAPI() as api:
        obs = api.get_observation(vid)
        for ob in obs.visible_objects:
            if ob.object_id == "Signal_A":
                print("=== Signal_A 全字段 ===", flush=True)
                print(json.dumps(ob.model_dump(), ensure_ascii=False, indent=1), flush=True)
                for payload in ({"key": ob.observation_token}, {"key": ob.hint}, None):
                    try:
                        r = api.interact(vid, "Signal_A", "submit_key", payload)
                        print(f"submit_key({str(payload)[:60]}) -> granted={r.granted} "
                              f"msg={r.message}", flush=True)
                    except Exception as e:
                        print("err:", e, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
