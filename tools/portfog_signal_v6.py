# -*- coding: utf-8 -*-
"""港口迷雾 Signal_A：沿车道 y=120.78 向西 + 横向纠偏 -> 对准 -> 读 token -> submit_key。

控制走已标定的原始 JSON（gear 2 前进 / 3 驻车 / 4 倒车，heading = yaw + 180），
读数据与交互走官方 SDK。
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
    x, y, h, o = raw.st(vid)
    print(f"{vid} ({x:.1f},{y:.1f}) h={h:.1f} 目标 x≈{SIG[0]:.0f}, 车道 y={LANE_Y}", flush=True)

    # 1) 先对准正西
    raw.orient(vid, 180.0)
    x, y, h, o = raw.st(vid)
    print(f"对准西向: ({x:.1f},{y:.1f}) h={h:.1f}", flush=True)

    # 2) 沿车道向西，横向纠偏
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < 120:
        x, y, h, o = raw.st(vid)
        if x <= SIG[0] + 1.0:
            break
        lat = y - LANE_Y                      # 正=偏北
        corr = max(-22.0, min(22.0, -lat * 3.0))
        want = 180.0 + corr
        err = wrap180(want - h)
        steer = max(-0.7, min(0.7, err / 45.0))
        raw.ctrl(vid, thr=0.28 if abs(err) < 40 else 0.18, st=steer, gear=2)
        if time.time() - last > 3.0:
            last = time.time()
            print(f"  ({x:7.1f},{y:7.1f}) h={h:6.1f} 横向={lat:+5.2f} 期望={want:6.1f} "
                  f"err={err:+6.1f}", flush=True)
        time.sleep(0.07)
    raw.ctrl(vid, brk=1.0, gear=3)
    x, y, h, o = raw.st(vid)
    print(f"西行结束 ({x:.1f},{y:.1f}) h={h:.1f}", flush=True)

    # 3) 朝信号方位对准
    want = math.degrees(math.atan2(SIG[1] - y, SIG[0] - x))
    raw.orient(vid, want, tol=8.0)
    x, y, h, o = raw.st(vid)
    print(f"对准信号: ({x:.1f},{y:.1f}) h={h:.1f} d={math.hypot(SIG[0]-x, SIG[1]-y):.1f}", flush=True)

    # 4) 小步逼近到 6 m 内
    for _ in range(10):
        x, y, h, o = raw.st(vid)
        d = math.hypot(SIG[0] - x, SIG[1] - y)
        if d <= 6.0:
            break
        want = math.degrees(math.atan2(SIG[1] - y, SIG[0] - x))
        err = wrap180(want - h)
        if abs(err) > 20:
            raw.orient(vid, want, tol=8.0)
            continue
        t1 = time.time()
        while time.time() - t1 < 0.7:
            raw.ctrl(vid, thr=0.18, st=max(-0.5, min(0.5, err / 55.0)), gear=2)
            time.sleep(0.06)
        raw.ctrl(vid, brk=1.0, gear=3)

    x, y, h, o = raw.st(vid)
    print(f"逼近完成 ({x:.1f},{y:.1f}) d={math.hypot(SIG[0]-x, SIG[1]-y):.1f}", flush=True)

    # 5) SDK 读 token 并提交
    with ScenarioTaskAPI() as api:
        obs = api.get_observation(vid)
        print("可见:", [b.object_id for b in obs.visible_objects], flush=True)
        tob = next((b for b in obs.visible_objects if b.object_id == "Signal_A"), None)
        if tob is None:
            print("!! Signal_A 不在视野", flush=True)
            return 0
        print(json.dumps(tob.model_dump(), ensure_ascii=False, indent=1)[:900], flush=True)
        for payload in ({"key": tob.observation_token}, None):
            r = api.interact(vid, "Signal_A", "submit_key", payload)
            print(f"submit_key({str(payload)[:40]}) granted={r.granted} msg={r.message}", flush=True)
            if r.granted:
                break
        st = api.get_task_state()
        print(f"sig={st.verified_signal_count}/{st.total_signal_count}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
