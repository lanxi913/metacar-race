# -*- coding: utf-8 -*-
"""港口迷雾：按路网航点行驶到目标，到位后读 hint 里的 Key 并 submit_key。

控制：低速纯跟踪（前视 6 m）+ 位移航向 + 卡住倒车脱困。
"""
from __future__ import annotations

import argparse
import json
import math
import re
import socket
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

from portfog_route import RoadMap  # noqa: E402

KEY_RE = re.compile(r"KEY[-_ ]?[A-Z0-9]{1,8}", re.IGNORECASE)


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_D")
    ap.add_argument("--tx", type=float, required=True)
    ap.add_argument("--ty", type=float, required=True)
    ap.add_argument("--seconds", type=float, default=240.0)
    ap.add_argument("--tol", type=float, default=6.0)
    args = ap.parse_args()

    car = Car()
    vid = args.vehicle
    x, y, o = car.st(vid)
    print(f"{vid} ({x:.1f},{y:.1f}) -> ({args.tx},{args.ty})", flush=True)

    rm = RoadMap()
    wps, total = rm.route((x, y), (args.tx, args.ty))
    if len(wps) < 2:
        wps = [(args.tx, args.ty)]
        print("路由为空，直连目标", flush=True)
    else:
        print(f"路线 {len(wps)} 点 / {total:.0f} m", flush=True)

    seg = 0
    t0 = time.time()
    stall_t = time.time()
    best = 1e9
    last_log = 0.0
    while time.time() - t0 < args.seconds:
        x, y, o = car.st(vid)
        d_final = math.hypot(args.tx - x, args.ty - y)
        if seg >= len(wps) or d_final <= args.tol:
            break
        tx, ty = wps[seg]
        d = math.hypot(tx - x, ty - y)
        if d < 5.0:
            seg += 1
            stall_t, best = time.time(), 1e9
            continue
        if d < best - 0.6:
            best, stall_t = d, time.time()
        if time.time() - stall_t > 4.5:
            print(f"  !! 卡住 ({x:.1f},{y:.1f})，倒车", flush=True)
            t1 = time.time()
            while time.time() - t1 < 1.6:
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
        if time.time() - last_log > 5.0:
            last_log = time.time()
            print(f"  ({x:7.1f},{y:7.1f}) 段{seg}/{len(wps)} d={d:5.1f} 终距={d_final:5.1f} "
                  f"航向={car.head:6.1f} err={err:+6.1f}", flush=True)
        time.sleep(0.07)

    car.ctrl(vid, brk=1.0, gear=3)
    x, y, o = car.st(vid)
    print(f"结束 ({x:.1f},{y:.1f}) 终距 {math.hypot(args.tx-x, args.ty-y):.1f}", flush=True)

    # 读 hint 里的 key 并提交
    from metacar_multiagent import ScenarioTaskAPI
    with ScenarioTaskAPI() as api:
        for cand in (vid, "Car_C", "Car_D"):
            obs = api.get_observation(cand)
            for ob in obs.visible_objects:
                m = KEY_RE.search(ob.hint or "")
                if not m:
                    continue
                key = m.group(0).replace(" ", "")
                print(f"  {cand} 看见 {ob.object_id} d={ob.distance_m:.1f} key={key}", flush=True)
                if ob.distance_m <= 7.5:
                    r = api.interact(cand, ob.object_id, "submit_key", {"key": key})
                    print(f"  submit_key({key}) granted={r.granted} msg={r.message}", flush=True)
                    st = api.get_task_state()
                    print(f"  信号 {st.verified_signal_count}/{st.total_signal_count}", flush=True)
                    nxt = next((o for o in st.shared_task_objects
                                if o.object_type == "signal" and o.state != "verified"), None)
                    if nxt:
                        print(f"  下一个: {nxt.object_id} @({nxt.position.x:.1f},"
                              f"{nxt.position.y:.1f})", flush=True)
                    return 0
        print("附近没有可读的信号", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
