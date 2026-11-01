# -*- coding: utf-8 -*-
"""港口迷雾全流程 runner（v7）：栅格路由 + 低速纯跟踪 + 按序验证信号 + 路障扫描。

设计原则（来自实测教训）：
  * 控制走原始 JSON（gear 2/3/4），航向只能由位移 atan2 估，且每次归一化 ±180
  * 速度上限 20 km/h（5.5 m/s），throttle 0.30 起，超过 4.5 m/s 就收油
  * 卡住（2.5 s 位移 < 0.6 m）立刻倒车 1.6 s 脱困
  * 每 0.07 s 一轮闭环，不做开环长跑
  * 信号验证码在观测对象的 hint 里，正则 KEY-XX
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
SPEED_LIMIT_MPS = 4.6          # 留余量，平台 20 km/h = 5.56 m/s


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class Bot:
    def __init__(self) -> None:
        self.s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
        self.s.settimeout(10)
        self.buf = b""
        self.pts: dict[str, list] = {}
        self.head: dict[str, float] = {}

    # ---- 底层 ----
    def call(self, p: dict) -> dict:
        self.s.sendall((json.dumps(p) + "\n").encode())
        while b"\n" not in self.buf:
            c = self.s.recv(1 << 20)
            if not c:
                raise ConnectionError("closed")
            self.buf += c
        line, rest = self.buf.split(b"\n", 1)
        self.buf = rest
        return json.loads(line.decode("utf-8", "replace"))

    def ctrl(self, vid: str, thr=0.0, brk=0.0, st=0.0, gear=2) -> dict:
        return self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                          "vehicle_control": {"gear": gear, "throttle": thr,
                                              "brake": brk, "steering": st}})

    def state(self, vid: str):
        r = self.call({"command": "get_vehicle_state" if False else "get_observation",
                       "vehicle_id": vid})
        o = r.get("observation") or {}
        p = o.get("position") or {}
        x, y = float(p.get("x", 0.0)), float(p.get("y", 0.0))
        pl = self.pts.setdefault(vid, [])
        pl.append((x, y))
        if len(pl) > 60:
            pl.pop(0)
        for bx, by in reversed(pl[:-1]):
            if math.hypot(x - bx, y - by) > 1.2:
                h = math.degrees(math.atan2(y - by, x - bx))
                prev = self.head.get(vid)
                self.head[vid] = h if prev is None else wrap180(prev + 0.5 * wrap180(h - prev))
                break
        return x, y, o

    def speed(self, vid: str) -> float:
        r = self.call({"command": "get_vehicle_state", "vehicle_id": vid})
        vs = r.get("vehicle_state") or {}
        return float(vs.get("speed_mps") or 0.0)

    # ---- 驾驶 ----
    def unstick(self, vid: str, secs: float = 2.0) -> None:
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, thr=0.18, st=-0.9, gear=4)
            time.sleep(0.07)
        self.ctrl(vid, brk=1.0, gear=3)
        time.sleep(0.25)

    def follow(self, vid: str, path, tol: float = 7.0, deadline: float = 240.0,
               log_every: float = 6.0) -> bool:
        t0 = time.time()
        i = 0
        stall_t = time.time()
        best = 1e9
        last_log = 0.0
        while i < len(path) and time.time() - t0 < deadline:
            x, y, o = self.state(vid)
            tx, ty = path[i]
            d = math.hypot(tx - x, ty - y)
            if d <= tol:
                i += 1
                stall_t, best = time.time(), 1e9
                continue
            if d < best - 0.6:
                best, stall_t = d, time.time()
            if time.time() - stall_t > 3.0:
                print(f"    !! 卡住 ({x:.1f},{y:.1f})，倒车脱困", flush=True)
                self.unstick(vid)
                stall_t, best = time.time(), 1e9
                continue
            h = self.head.get(vid)
            if h is None:
                self.ctrl(vid, thr=0.25, st=0.0, gear=2)
                time.sleep(0.07)
                continue
            want = math.degrees(math.atan2(ty - y, tx - x))
            err = wrap180(want - h)
            # 这车满舵约 11°/s、转弯半径大，必须低速 + 提前打舵
            steer = max(-0.9, min(0.9, err / 35.0))
            thr = 0.18 if abs(err) < 25 else 0.12
            spd = self.speed(vid)
            if spd > SPEED_LIMIT_MPS:
                thr = 0.0
            self.ctrl(vid, thr=thr, st=steer, gear=2)
            if time.time() - last_log > log_every:
                last_log = time.time()
                print(f"    ({x:7.1f},{y:7.1f}) 段{i}/{len(path)} d={d:5.1f} "
                      f"h={h:6.1f} err={err:+6.1f} v={spd:.1f}", flush=True)
            time.sleep(0.07)
        self.ctrl(vid, brk=1.0, gear=3)
        return i >= len(path)

    def goto(self, vid: str, tx: float, ty: float, grid: GridRoad,
             deadline: float = 240.0) -> bool:
        x, y, _ = self.state(vid)
        path = grid.simplify(grid.astar((x, y), (tx, ty)))
        path = path[1:] if len(path) > 1 else [(tx, ty)]
        path.append((tx, ty))
        print(f"  路由 ({x:.0f},{y:.0f})->({tx:.0f},{ty:.0f}) {len(path)} 点", flush=True)
        return self.follow(vid, path, deadline=deadline)


def main() -> int:
    budget = float(sys.argv[1]) if len(sys.argv) > 1 else 560.0
    t_start = time.time()

    print("等待场景 9100 ...", flush=True)
    for _ in range(180):
        try:
            bot = Bot()
            break
        except Exception:
            time.sleep(1.0)
    else:
        print("!! 9100 起不来", flush=True)
        return 2
    print("已连接", flush=True)

    grid = GridRoad()
    api = None
    from metacar_multiagent import ScenarioTaskAPI
    api = ScenarioTaskAPI().connect()

    def remaining():
        return budget - (time.time() - t_start)

    try:
        # ===== 阶段 1：按序验证信号 =====
        for rnd in range(6):
            if remaining() < 60:
                print("!! 时间不够，收尾", flush=True)
                break
            st = api.get_task_state()
            print(f"\n[阶段1] 信号 {st.verified_signal_count}/{st.total_signal_count} "
                  f"路障={st.roadblock_active}", flush=True)
            sigs = [o for o in st.shared_task_objects if o.object_type == "signal"]
            if not sigs:
                print("没有待验证信号了", flush=True)
                break
            sig = sigs[0]
            tx, ty = float(sig.position.x), float(sig.position.y)
            print(f"  目标 {sig.object_id} ({tx:.1f},{ty:.1f}) lvl={sig.information_level}", flush=True)

            ok = bot.goto("Car_C", tx, ty - 3.0, grid, deadline=min(240, remaining() - 30))
            x, y, _ = bot.state("Car_C")
            print(f"  到位={ok} ({x:.1f},{y:.1f}) 距信号 {math.hypot(tx-x, ty-y):.1f}", flush=True)

            # 最后一段直冲：低速朝信号点本身逼近到 4 m 内（最长 25 s）
            t_app = time.time()
            while time.time() - t_app < 25.0:
                x3, y3, _ = bot.state("Car_C")
                d3 = math.hypot(tx - x3, ty - y3)
                if d3 <= 4.0:
                    break
                h3 = bot.head.get("Car_C")
                if h3 is None:
                    bot.ctrl("Car_C", thr=0.20, st=0.0, gear=2)
                    time.sleep(0.07)
                    continue
                want3 = math.degrees(math.atan2(ty - y3, tx - x3))
                err3 = wrap180(want3 - h3)
                bot.ctrl("Car_C", thr=0.16, st=max(-0.8, min(0.8, err3 / 40.0)), gear=2)
                time.sleep(0.07)
            bot.ctrl("Car_C", brk=1.0, gear=3)
            x3, y3, _ = bot.state("Car_C")
            print(f"  直冲后 ({x3:.1f},{y3:.1f}) 距信号 {math.hypot(tx-x3, ty-y3):.1f}", flush=True)

            # 先算方位，用倒车满舵（约 50°/s）精确对准信号，再读 key
            done = False
            for k in range(1):
              pass
            for k in range(10):
                obs = api.get_observation("Car_C")
                for ob in obs.visible_objects:
                    m = KEY_RE.search(ob.hint or "")
                    if ob.object_type == "signal" and m and ob.distance_m <= 8.0:
                        key = m.group(0).replace(" ", "")
                        r = api.interact("Car_C", ob.object_id, "submit_key", {"key": key})
                        print(f"  submit {ob.object_id}({key}) granted={r.granted} {r.message}",
                              flush=True)
                        done = True
                        break
                if done:
                    break
                # 朝信号方位对准（倒车满舵约 50°/s）
                x2, y2, _ = bot.state("Car_C")
                want = math.degrees(math.atan2(ty - y2, tx - x2))
                h2 = bot.head.get("Car_C")
                if h2 is not None:
                    err2 = wrap180(want - h2)
                    if abs(err2) > 12:
                        secs = min(1.3, max(0.3, abs(err2) / 50.0))
                        t0 = time.time()
                        while time.time() - t0 < secs:
                            bot.ctrl("Car_C", thr=0.40, st=1.0 if err2 > 0 else -1.0, gear=4)
                            time.sleep(0.07)
                        bot.ctrl("Car_C", brk=1.0, gear=3)
                        time.sleep(0.2)
                        continue
            if not done:
                print("  !! 这一轮没读到 key，重试", flush=True)
            else:
                st = api.get_task_state()
                print(f"  进度 {st.verified_signal_count}/{st.total_signal_count}", flush=True)

        # ===== 阶段 2：动态路障扫描 =====
        st = api.get_task_state()
        print(f"\n[阶段2] 路障激活={st.roadblock_active}", flush=True)
        if st.roadblock_active and remaining() > 60:
            rb = next((o for o in st.shared_task_objects if o.object_type == "roadblock"), None)
            if rb and rb.position:
                tx, ty = float(rb.position.x), float(rb.position.y)
                print(f"  路障 {rb.object_id} ({tx:.1f},{ty:.1f})", flush=True)
                bot.goto("Car_C", tx, ty - 4.0, grid, deadline=min(200, remaining() - 30))
                r = api.interact("Car_C", rb.object_id, "scan")
                print(f"  scan granted={r.granted} {r.message}", flush=True)

        # ===== 收尾：全部停车 =====
        for v in ("Car_A", "Car_B", "Car_C", "Car_D"):
            bot.ctrl(v, brk=1.0, gear=3)
        st = api.get_task_state()
        print(f"\n[结束] 信号 {st.verified_signal_count}/{st.total_signal_count} "
              f"路障={st.roadblock_active} 用时 {time.time()-t_start:.0f}s", flush=True)
    finally:
        try:
            api.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
