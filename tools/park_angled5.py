# -*- coding: utf-8 -*-
"""异形车道泊车 v5：车道内巡航 -> 摆头 -> 倒车入位。

与 v4 的区别：
  1. 路径不再跟着 static.route（route 在车位附近拐进停车区，v4 就是撞在这里）
  2. 用实测车体 4.81x2.18 做 OBB 碰撞检查
  3. 倒车段不再猜转向符号，改用验证过的运动学模型做"采样控制"：
     对每个候选 steer 预测 0.8 m 后的位姿，取离路径最近的
运动学模型（实测标定）：
     倒车 steer=+1 -> 场景 yaw 减小；kappa = 0.1834 * steer
     车心到后轴 H_RA = 1.45
"""
from __future__ import annotations

import heapq
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

# ---------- 车辆/控制参数 ----------
EGO_L, EGO_W = 4.81, 2.18
H_RA = 1.45
K_STEER = 0.1834

V_FWD = 1.4          # 巡航目标速度 m/s
V_REV = 0.55         # 倒车目标速度 m/s
THR = 0.10
BRK = 0.14

# ---------- 规划参数 ----------
STEP = 0.50
XY_RES = 0.30
TH_RES = math.radians(9)
STEERS = (-1.0, -0.6, -0.25, 0.0, 0.25, 0.6, 1.0)
MARGIN = 0.10


def hdg(ori):
    return math.radians(-ori)


def wrap_deg(d):
    return (d + 180) % 360 - 180


def obb(x, y, ori, L, W):
    th = hdg(ori)
    c, s = math.cos(th), math.sin(th)
    return [(x + dl * c - dw * s, y + dl * s + dw * c)
            for dl, dw in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2))]


def sep(p, q):
    best = 1e9
    for poly in (p, q):
        for i in range(4):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % 4]
            ax, ay = -(y2 - y1), (x2 - x1)
            n = math.hypot(ax, ay) or 1
            ax, ay = ax / n, ay / n
            a = [x * ax + y * ay for x, y in p]
            b = [x * ax + y * ay for x, y in q]
            best = min(best, min(max(a), max(b)) - max(min(a), min(b)))
    return -best


class Kin:
    """带符号弧长的自行车模型。direction=+1 前进 / -1 倒车。"""

    def __init__(self, obstacles):
        self.obs = [obb(*o) for o in obstacles]

    def move(self, x, y, ori, steer, direction, ds):
        th = hdg(ori)
        ax = x - H_RA * math.cos(th)
        ay = y - H_RA * math.sin(th)
        s_signed = direction * ds
        kappa = K_STEER * steer * direction
        if abs(kappa) < 1e-9:
            ax += s_signed * math.cos(th)
            ay += s_signed * math.sin(th)
        else:
            th2 = th + kappa * s_signed
            R = 1 / kappa
            ax += R * (math.sin(th2) - math.sin(th))
            ay -= R * (math.cos(th2) - math.cos(th))
            th = th2
        return (ax + H_RA * math.cos(th), ay + H_RA * math.sin(th), (-math.degrees(th)) % 360)

    def free(self, x, y, ori):
        e = obb(x, y, ori, EGO_L, EGO_W)
        for o in self.obs:
            if sep(e, o) < MARGIN:
                return False
        return True


def plan_path(kin, start, goal, max_expand=120000):
    """hybrid A*。start/goal = (x, y, ori)。"""
    def h(x, y, ori):
        return math.hypot(x - goal[0], y - goal[1]) + math.radians(abs(wrap_deg(ori - goal[2]))) * 1.2

    def key(x, y, ori):
        return (round(x / XY_RES), round(y / XY_RES), round(hdg(ori) / TH_RES))

    openq = [(h(*start), 0.0, 0, start[0], start[1], start[2], 0, None)]
    best = {key(*start): 0.0}
    came = {}
    nid = 1
    while openq and nid < max_expand:
        f, g, _, x, y, ori, dprev, parent = heapq.heappop(openq)
        if best.get(key(x, y, ori), 1e9) < g - 1e-6:
            continue
        if math.hypot(x - goal[0], y - goal[1]) <= 0.30 and abs(wrap_deg(ori - goal[2])) <= 6:
            path = [(x, y, ori)]
            while parent is not None:
                x, y, ori, parent = came[parent]
                path.append((x, y, ori))
            path.reverse()
            return path
        for d in (1, -1):
            for s in STEERS:
                nx, ny, no = kin.move(x, y, ori, s, d, STEP)
                if not kin.free(nx, ny, no):
                    continue
                ng = g + STEP * (1.3 if d < 0 else 1.0)
                if dprev != 0 and d != dprev:
                    ng += 4.0
                kk = key(nx, ny, no)
                if ng < best.get(kk, 1e9):
                    best[kk] = ng
                    came[nid] = (x, y, ori, parent)
                    heapq.heappush(openq, (ng + h(nx, ny, no), ng, nid, nx, ny, no, d, nid))
                    nid += 1
    return None


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...", flush=True)
    api.connect()
    print("已连接。", flush=True)

    state = {"steer": 0.0, "thr": 0.0, "brk": 0.0, "gear": None, "t0": time.time()}

    def send(gear, thr, brk, steer):
        # 转向与油门都做斜率限制，避免指令突变
        state["steer"] += max(-0.15, min(0.15, steer - state["steer"]))
        state["thr"] += max(-0.05, min(0.05, thr - state["thr"]))
        state["brk"] += max(-0.05, min(0.05, brk - state["brk"]))
        state["steer"] = max(-1.0, min(1.0, state["steer"]))
        state["thr"] = max(0.0, min(1.0, state["thr"]))
        state["brk"] = max(0.0, min(1.0, state["brk"]))
        api.set_vehicle_control(VehicleControl(gear=gear, throttle=state["thr"],
                                               brake=state["brk"], steering=state["steer"]))

    path = None
    idx = 0
    phase = "CRUISE"
    kin = None
    slot = None
    switch_i = None
    log_t = 0.0
    last_pose = None

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        car = msg.main_vehicle
        v = car.speed
        x, y, ori = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        now = time.time()

        if kin is None:
            obs = []
            for ob in (msg.obstacles or []):
                name = getattr(getattr(ob, "type", None), "name", str(getattr(ob, "type", "")))
                if "PARK" in name.upper():
                    slot = (float(ob.pos_x), float(ob.pos_y), float(ob.ori_z),
                            float(ob.length), float(ob.width))
                elif "CAR" in name.upper():
                    obs.append((float(ob.pos_x), float(ob.pos_y), float(ob.ori_z),
                                float(ob.length), float(ob.width)))
            if slot is None:
                send(GearMode.PARKING, 0.0, 0.3, 0.0)
                continue
            kin = Kin(obs)
            print(f"车位 {slot[:3]}  障碍 {len(obs)} 个", flush=True)

        if path is None:
            # 第一段：沿车道直线开到车位右前方的预置点
            sx, sy = slot[0], slot[1]
            stage = (-4.90, sy + 5.0)
            plan = [(x, y)] + [(x + (stage[0] - x) * i / 8.0, y + (stage[1] - y) * i / 8.0)
                               for i in range(1, 9)]
            path = [(px, py, 90.0) for px, py in plan]
            idx = 0
            phase = "CRUISE"
            print(f"巡航到预置点 {stage}", flush=True)

        if phase == "CRUISE":
            # 采样控制：跟直线
            best_s, best_c = 0.0, 1e9
            look = 3.0
            tx, ty = path[-1][0], path[-1][1]
            for s in (-1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0):
                px, py, po = kin.move(x, y, ori, s, 1, look)
                c = math.hypot(px - tx, py - ty)
                if c < best_c:
                    best_c, best_s = c, s
            thr = THR if v < V_FWD else 0.0
            brk = BRK if v > V_FWD + 0.4 else 0.0
            send(GearMode.DRIVE, thr, brk, best_s)
            if now - log_t > 1.0:
                log_t = now
                print(f"[{now - state['t0']:5.1f}s] CRUISE ({x:.2f},{y:.2f}) yaw={ori:.1f} "
                      f"v={v:.2f} 剩余 {math.hypot(tx - x, ty - y):.1f} m", flush=True)
            if math.hypot(tx - x, ty - y) < 1.2:
                phase = "STOP0"
                print(f"[{now - state['t0']:5.1f}s] 到预置点，滑行停稳", flush=True)
            continue

        if phase == "STOP0":
            send(GearMode.DRIVE, 0.0, 0.0, 0.0)
            if abs(v) < 0.08:
                print(f"[{now - state['t0']:5.1f}s] 已停稳，规划泊车路径", flush=True)
                goal = (slot[0], slot[1], slot[2])
                t_plan = time.time()
                p = plan_path(kin, (x, y, ori), goal)
                print(f"规划耗时 {time.time() - t_plan:.1f}s", flush=True)
                if not p:
                    print("规划失败，停在原地", flush=True)
                    send(GearMode.PARKING, 0.0, 0.5, 0.0)
                    break
                path = p
                idx = 0
                switch_i = None
                # 找换向点：第一个 y 开始回升的点（倒车阶段起点）
                for i in range(1, len(path)):
                    if path[i][1] > path[i - 1][1] + 0.05 and i > 3:
                        switch_i = i
                        break
                print(f"路径 {len(path)} 点，换向点 {switch_i}", flush=True)
                for k in range(0, len(path), max(1, len(path) // 20)):
                    px, py, po = path[k]
                    print(f"    {k:3d} ({px:7.2f},{py:7.2f}) {po:6.1f}", flush=True)
                phase = "FWD"
                last_pose = (x, y)
            continue

        if phase in ("FWD", "REV"):
            direction = 1 if phase == "FWD" else -1
            # 找最近路径点
            best_i, best_d = idx, 1e9
            for i in range(idx, min(len(path), idx + 40)):
                d = math.hypot(path[i][0] - x, path[i][1] - y)
                if d < best_d:
                    best_d, best_i = d, i
            idx = best_i
            # 前视点
            j = best_i
            acc = 0.0
            while 0 <= j + direction < len(path) and acc < 2.5:
                acc += math.hypot(path[j + direction][0] - path[j][0],
                                  path[j + direction][1] - path[j][1])
                j += direction
            tx, ty = path[j][0], path[j][1]

            best_s, best_c = 0.0, 1e9
            for s in (-1.0, -0.7, -0.4, -0.15, 0.0, 0.15, 0.4, 0.7, 1.0):
                px, py, po = kin.move(x, y, ori, s, direction, 0.9)
                c = math.hypot(px - tx, py - ty) + 0.35 * math.radians(abs(wrap_deg(po - path[j][2])))
                if c < best_c:
                    best_c, best_s = c, s

            vt = V_FWD if phase == "FWD" else V_REV
            thr = THR if abs(v) < vt else 0.0
            brk = BRK if abs(v) > vt + 0.35 else 0.0
            gear = GearMode.DRIVE if phase == "FWD" else GearMode.REVERSE
            send(gear, thr, brk, best_s)

            if now - log_t > 0.8:
                log_t = now
                print(f"[{now - state['t0']:5.1f}s] {phase} ({x:.2f},{y:.2f}) yaw={ori:.1f} "
                      f"v={v:.2f} steer={best_s:+.2f} idx={idx}/{len(path)} "
                      f"距终点 {math.hypot(path[-1][0] - x, path[-1][1] - y):.2f}", flush=True)

            if phase == "FWD" and switch_i and idx >= switch_i - 1:
                phase = "STOP1"
                print(f"[{now - state['t0']:5.1f}s] 到达换向点，滑行", flush=True)
            elif phase == "REV":
                d_end = math.hypot(path[-1][0] - x, path[-1][1] - y)
                if d_end < 0.30 and abs(abs(wrap_deg(ori - path[-1][2])) - 180) > 170:
                    phase = "HOLD"
                    print(f"[{now - state['t0']:5.1f}s] 到位 车心距 {d_end:.2f} m 航向差 "
                          f"{wrap_deg(ori - path[-1][2]):+.1f}", flush=True)
                elif d_end < 0.25:
                    phase = "HOLD"
                    print(f"[{now - state['t0']:5.1f}s] 到位 车心距 {d_end:.2f} m 航向差 "
                          f"{wrap_deg(ori - path[-1][2]):+.1f}", flush=True)
            continue

        if phase in ("STOP1", "HOLD"):
            if phase == "STOP1":
                send(GearMode.DRIVE, 0.0, 0.0, 0.0)
                if abs(v) < 0.06:
                    phase = "REV"
                    print(f"[{now - state['t0']:5.1f}s] 停稳，开始倒车", flush=True)
            else:
                send(GearMode.PARKING, 0.0, 0.6, 0.0)
                if now - state["t0"] > 0:
                    break
            continue

        if now - state["t0"] > 150:
            print("超时", flush=True)
            break

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
