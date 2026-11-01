# -*- coding: utf-8 -*-
"""港口迷雾：纯跟踪（投影 + 前视 carrot）沿栅格路径行驶。

修复的两个 bug：
  1) 不再"到下一点才切"，改为把车投影到路径上、取前方 lookahead 的 carrot
     —— 避免路径首点在车后时触发掉头（这车掉头半径 21 m，必然撞墙）
  2) 不再乱简化路径（每 12 格连线会穿过建筑），carrot 永远在路径上
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

    def reverse(self, vid, secs=1.6):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, thr=0.20, st=-0.9, gear=4)
            time.sleep(0.07)
        self.ctrl(vid, brk=1.0, gear=3)
        time.sleep(0.2)
        self.reset(vid)


def project(path, x, y):
    """返回 (弧长位置 s, 横向距离, 总长)。"""
    best_s, best_d, acc = 0.0, 1e9, 0.0
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        vx, vy = bx - ax, by - ay
        L = math.hypot(vx, vy)
        if L < 1e-6:
            continue
        t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / (L * L)))
        px, py = ax + t * vx, ay + t * vy
        d = math.hypot(x - px, y - py)
        if d < best_d - 1e-9:
            best_d, best_s = d, acc + t * L
        acc += L
    return best_s, best_d, acc


def carrot_at(path, s):
    acc = 0.0
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        L = math.hypot(bx - ax, by - ay)
        if acc + L >= s:
            t = (s - acc) / L if L > 1e-6 else 0.0
            return (ax + t * (bx - ax), ay + t * (by - ay))
        acc += L
    return path[-1]


def follow_path(bot, vid, path, lookahead=8.0, tmax=180.0, thr_cruise=0.28,
                tol_end=4.0, log_every=5.0):
    t0 = time.time()
    stall_t = time.time()
    last_s = -1e9
    last_log = 0.0
    total_travel = 0.0
    prev_xy = None
    while time.time() - t0 < tmax:
        x, y, o = bot.st(vid)
        if prev_xy is not None:
            step = math.hypot(x - prev_xy[0], y - prev_xy[1])
            if step < 3.0:                     # 忽略瞬移
                total_travel += step
        prev_xy = (x, y)
        s, lat, total = project(path, x, y)
        if s >= total - tol_end:
            bot.ctrl(vid, brk=1.0, gear=3)
            return True
        # 卡住（弧长不前进）
        if s > last_s + 0.6:
            last_s, stall_t = s, time.time()
        if time.time() - stall_t > 3.0:
            # 开局倒车会撞到车后的墙（实测每次都撞坏），前 6 m 内只准前进+满舵
            if total_travel < 6.0:
                print(f"    .. 起步受阻 ({x:.1f},{y:.1f})，前进满舵摆脱", flush=True)
                t2 = time.time()
                while time.time() - t2 < 1.2:
                    bot.ctrl(vid, thr=0.26, st=1.0 if (int(time.time() * 2) % 2) else -1.0, gear=2)
                    time.sleep(0.07)
                bot.ctrl(vid, brk=1.0, gear=3)
            else:
                print(f"    !! 卡住 ({x:.1f},{y:.1f}) s={s:.1f}/{total:.0f}，短倒车", flush=True)
                bot.reverse(vid, secs=0.8)
            stall_t, last_s = time.time(), -1e9
            continue
        cx, cy = carrot_at(path, min(total, s + lookahead))
        h = bot.head.get(vid)
        if h is None:
            bot.ctrl(vid, thr=0.18, st=0.0, gear=2)
            time.sleep(0.07)
            continue
        want = math.degrees(math.atan2(cy - y, cx - x))
        err = wrap180(want - h)
        # 横向偏差也进转向（保持车道中心），巡航提速到 0.28
        steer = max(-0.9, min(0.9, err / 35.0 + max(-0.25, min(0.25, -lat * 0.06))))
        # 分段限速：直道放开，进入目标 18 m 内强制慢（避免冲过头读不到 key）
        gate = math.hypot(path[-1][0] - x, path[-1][1] - y)
        if gate < 18.0:
            thr = 0.14
        elif abs(err) < 20 and abs(lat) < 2.0:
            thr = 0.34          # 直道提速
        elif abs(err) < 40:
            thr = 0.24
        else:
            thr = 0.14
        if abs(lat) > 5.0:
            thr = min(thr, 0.18)
        bot.ctrl(vid, thr=thr, st=steer, gear=2)
        if time.time() - last_log > log_every:
            last_log = time.time()
            print(f"    ({x:7.1f},{y:7.1f}) s={s:6.1f}/{total:5.0f} 横向={lat:+5.2f} "
                  f"h={h:6.1f} err={err:+6.1f}", flush=True)
        time.sleep(0.07)
    bot.ctrl(vid, brk=1.0, gear=3)
    return False


def main():
    budget = float(sys.argv[1]) if len(sys.argv) > 1 else 520.0
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
        for rnd in range(6):
            if left() < 80:
                break
            st = api.get_task_state()
            sigs = [o for o in st.shared_task_objects if o.object_type == "signal"]
            print(f"\n[轮{rnd}] 信号 {st.verified_signal_count}/{st.total_signal_count} "
                  f"剩 {left():.0f}s", flush=True)
            if not sigs:
                break
            sig = sigs[0]
            tx, ty = float(sig.position.x), float(sig.position.y)
            x, y, _ = bot.st("Car_C")
            path = grid.astar((x, y), (tx, ty))
            if len(path) < 2:
                print("  路由退化，直连目标", flush=True)
                path = [(x, y), (tx, ty)]
            L = sum(math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1])
                    for i in range(len(path)-1))
            print(f"  路由 {len(path)} 点 / {L:.0f} m -> {sig.object_id}({tx:.1f},{ty:.1f})",
                  flush=True)
            if L < 8.0:
                # 路径太短（车已在目标附近）：直接低速逼近
                t_app = time.time()
                while time.time() - t_app < 25.0:
                    x2, y2, _ = bot.st("Car_C")
                    d2 = math.hypot(tx - x2, ty - y2)
                    if d2 <= 3.0:
                        break
                    h2 = bot.head.get("Car_C")
                    if h2 is None:
                        bot.ctrl("Car_C", thr=0.18, st=0.0, gear=2)
                        time.sleep(0.07)
                        continue
                    want2 = math.degrees(math.atan2(ty - y2, tx - x2))
                    err2 = wrap180(want2 - h2)
                    bot.ctrl("Car_C", thr=0.16, st=max(-0.9, min(0.9, err2 / 40.0)), gear=2)
                    time.sleep(0.07)
                bot.ctrl("Car_C", brk=1.0, gear=3)
                print(f"  直接逼近 -> ({x2:.1f},{y2:.1f}) d={d2:.1f}", flush=True)
            else:
                ok = follow_path(bot, "Car_C", path, tmax=min(240.0, left() - 40))

            # 读 key：必须"边开边提交"（实测停车后提交一律 not_available）
            done = False
            _attempts = []
            for _ in range(10):
                obs = api.get_observation("Car_C")
                for ob in obs.visible_objects:
                    m = KEY_RE.search(ob.hint or "")
                    if ob.object_type == "signal" and m and ob.distance_m <= 8.0:
                        key = m.group(0).replace(" ", "")
                        r = api.interact("Car_C", ob.object_id, "submit_key", {"key": key})
                        print(f"  submit {ob.object_id}({key}) granted={r.granted} "
                              f"{r.message} d={ob.distance_m:.1f}", flush=True)
                        done = r.granted
                        break
                if done:
                    break
                # 不依赖航向：走小弧同时连续读观测，一读到就提交
                sdir = 1.0 if len(_attempts) % 2 == 0 else -1.0
                _attempts.append(1)
                found = False
                t1 = time.time()
                while time.time() - t1 < 2.4 and not found:
                    bot.ctrl("Car_C", thr=0.22, st=sdir * 0.6, gear=2)
                    time.sleep(0.10)
                    o2 = api.get_observation("Car_C")
                    for ob2 in o2.visible_objects:
                        m2 = KEY_RE.search(ob2.hint or "")
                        if ob2.object_type == "signal" and m2 and ob2.distance_m <= 8.0:
                            key2 = m2.group(0).replace(" ", "")
                            r2 = api.interact("Car_C", ob2.object_id, "submit_key", {"key": key2})
                            print(f"    扫描提交 {ob2.object_id}({key2}) granted={r2.granted} "
                                  f"{r2.message} d={ob2.distance_m:.1f}", flush=True)
                            if r2.granted:
                                done = True
                                found = True
                bot.ctrl("Car_C", brk=1.0, gear=3)
                time.sleep(0.15)
                if done:
                    break
            if done:
                st = api.get_task_state()
                print(f"  ✅ {st.verified_signal_count}/{st.total_signal_count}", flush=True)
            else:
                print("  ✗ 没读到 key", flush=True)

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
