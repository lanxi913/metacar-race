# -*- coding: utf-8 -*-
"""港口迷雾信号链 runner：逼近到 3 m + 方位对准 + 读 hint 提交，按序做 A/B/C。

关键实测参数：
  车满舵角速度 ≈11°/s（4 m/s 时半径 21 m），巡航必须 ≤2 m/s
  hint 只在 ≤4 m 且朝向目标时出现（5.7 m 能读到，6.8 m 读不到）
  倒车满舵 ≈50°/s，用来快速对准
"""
from __future__ import annotations

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

from portfog_grid import GridRoad  # noqa: E402

KEY_RE = re.compile(r"KEY[-_ ]?[A-Z0-9]{1,8}", re.IGNORECASE)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Bot:
    def __init__(self):
        self.s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
        self.s.settimeout(10)
        self.buf = b""
        self.pts = {}
        self.head = {}

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

    def reset(self, vid):
        """倒车后必须重置：倒车时位移方向反 180°，会污染航向估计。"""
        self.pts[vid] = []
        self.head[vid] = None

    def st(self, vid):
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        x, y = float(p.get("x", 0.0)), float(p.get("y", 0.0))
        pl = self.pts.setdefault(vid, [])
        pl.append((x, y))
        if len(pl) > 60:
            pl.pop(0)
        for bx, by in reversed(pl[:-1]):
            if math.hypot(x - bx, y - by) > 1.0:
                h = math.degrees(math.atan2(y - by, x - bx))
                prev = self.head.get(vid)
                self.head[vid] = h if prev is None else wrap180(prev + 0.6 * wrap180(h - prev))
                break
        return x, y, o


def drive_to(bot, vid, tx, ty, tol=3.0, tmax=90.0, vfar=0.22, vnear=0.12):
    """距离闭环逼近：远处正常速度，近处慢，卡住倒车。"""
    t0 = time.time()
    stall = time.time()
    best = 1e9
    while time.time() - t0 < tmax:
        x, y, o = bot.st(vid)
        d = math.hypot(tx - x, ty - y)
        if d <= tol:
            bot.ctrl(vid, brk=1.0, gear=3)
            return True, d
        if d < best - 0.5:
            best, stall = d, time.time()
        if time.time() - stall > 3.0:
            t1 = time.time()
            while time.time() - t1 < 1.6:
                bot.ctrl(vid, thr=0.20, st=-0.9, gear=4)
                time.sleep(0.07)
            bot.ctrl(vid, brk=1.0, gear=3)
            time.sleep(0.2)
            bot.reset(vid)
            stall, best = time.time(), 1e9
            continue
        h = bot.head.get(vid)
        if h is None:
            bot.ctrl(vid, thr=0.20, st=0.0, gear=2)
            time.sleep(0.07)
            continue
        want = math.degrees(math.atan2(ty - y, tx - x))
        err = wrap180(want - h)
        st = max(-0.9, min(0.9, err / 35.0))
        thr = vfar if d > 8 else vnear
        if abs(err) > 40:
            thr = min(thr, 0.12)
        bot.ctrl(vid, thr=thr, st=st, gear=2)
        time.sleep(0.07)
    bot.ctrl(vid, brk=1.0, gear=3)
    return False, best


def face(bot, vid, tx, ty, tol=15.0, rounds=8):
    """倒车满舵对准目标方位（≈50°/s）。"""
    for _ in range(rounds):
        x, y, _ = bot.st(vid)
        want = math.degrees(math.atan2(ty - y, tx - x))
        h = bot.head.get(vid)
        if h is None:
            bot.ctrl(vid, thr=0.20, st=0.0, gear=2)
            time.sleep(0.1)
            continue
        err = wrap180(want - h)
        if abs(err) <= tol:
            return True
        secs = min(1.2, max(0.3, abs(err) / 50.0))
        t0 = time.time()
        while time.time() - t0 < secs:
            bot.ctrl(vid, thr=0.38, st=1.0 if err > 0 else -1.0, gear=4)
            time.sleep(0.07)
        bot.ctrl(vid, brk=1.0, gear=3)
        time.sleep(0.2)
        bot.reset(vid)
    return False


def main():
    budget = float(sys.argv[1]) if len(sys.argv) > 1 else 540.0
    t_start = time.time()
    for _ in range(180):
        try:
            bot = Bot()
            break
        except Exception:
            time.sleep(1.0)
    else:
        print("!! 9100 起不来", flush=True)
        return 2
    from metacar_multiagent import ScenarioTaskAPI
    api = ScenarioTaskAPI().connect()
    grid = GridRoad()
    print("已连接", flush=True)

    def left():
        return budget - (time.time() - t_start)

    try:
        for rnd in range(8):
            if left() < 70:
                print("时间到，收尾", flush=True)
                break
            st = api.get_task_state()
            sigs = [o for o in st.shared_task_objects if o.object_type == "signal"]
            print(f"\n[轮{rnd}] 信号 {st.verified_signal_count}/{st.total_signal_count} "
                  f"剩余 {left():.0f}s 目标 {[s.object_id for s in sigs]}", flush=True)
            if not sigs:
                break
            sig = sigs[0]
            tx, ty = float(sig.position.x), float(sig.position.y)

            # 1) 栅格路由到信号附近 12 m 处
            x, y, _ = bot.st("Car_C")
            path = grid.simplify(grid.astar((x, y), (tx, ty - 10.0)))
            path = (path[1:] or [(tx, ty - 10.0)]) + [(tx, ty - 4.0)]
            print(f"  路由 {len(path)} 点", flush=True)
            ok = True
            for wp in path:
                if left() < 60:
                    break
                ok, d = drive_to(bot, "Car_C", wp[0], wp[1], tol=6.0,
                                 tmax=min(90.0, left() - 40))
                print(f"    -> ({wp[0]:.0f},{wp[1]:.0f}) ok={ok} d={d:.1f}", flush=True)

            # 2) 精确逼近到 3 m 内
            ok, d = drive_to(bot, "Car_C", tx, ty, tol=3.0, tmax=45.0)
            x, y, _ = bot.st("Car_C")
            print(f"  逼近 ok={ok} ({x:.1f},{y:.1f}) d={math.hypot(tx-x,ty-y):.1f}", flush=True)

            # 3) 对准 + 读 key + 提交
            done = False
            for attempt in range(5):
                face(bot, "Car_C", tx, ty)
                obs = api.get_observation("Car_C")
                for ob in obs.visible_objects:
                    m = KEY_RE.search(ob.hint or "")
                    if ob.object_type == "signal" and m and ob.distance_m <= 8.0:
                        key = m.group(0).replace(" ", "")
                        r = api.interact("Car_C", ob.object_id, "submit_key", {"key": key})
                        print(f"  submit {ob.object_id}({key}) granted={r.granted} "
                              f"{r.message} (d={ob.distance_m:.1f})", flush=True)
                        done = r.granted
                        break
                if done:
                    break
                # 再靠近一点
                drive_to(bot, "Car_C", tx, ty, tol=2.5, tmax=12.0)
            if done:
                st = api.get_task_state()
                print(f"  ✅ 进度 {st.verified_signal_count}/{st.total_signal_count}", flush=True)
            else:
                print("  ✗ 本轮没读到 key", flush=True)

        for v in ("Car_A", "Car_B", "Car_C", "Car_D"):
            bot.ctrl(v, brk=1.0, gear=3)
        st = api.get_task_state()
        print(f"\n[结束] 信号 {st.verified_signal_count}/{st.total_signal_count} "
              f"用时 {time.time()-t_start:.0f}s", flush=True)
    finally:
        try:
            api.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
