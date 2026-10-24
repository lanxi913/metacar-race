# -*- coding: utf-8 -*-
"""异形车道泊车 v6：车道巡航 → 借道摆头 → 倒车入位。

与 v4/v5 的区别：
  1. 路径由 hybrid A* 现场生成，**车体四角**参与碰撞与越线约束（车宽 2.18）
  2. 不跟 static.route（route 在车位附近拐进停车区，v4 就是撞在那里）
  3. 倒车段不再跟踪路径，改用**车位坐标系级联控制**（离线扫参验证过）：
         yaw_des = slot_ori − 20·dv          # dv = 车位系横向偏差
         steer   = clamp(1.6·(yaw − yaw_des), ±1)
     终点目标 du=+0.22（比车位中心略微外停），dv≈0
     —— 自车 2.18 m 比邻车宽，停正中心会让左后角越过 SIDE_LINE(x=-10.95)
  4. 规划在后台线程跑（实测 2.8 s），不阻塞控制循环

坐标/符号约定（实测，不要改）：
      yaw 顺时针为正；前舵 >0 → yaw 增大；倒车 >0 → yaw 减小
      κ = 0.1834·steer（倒车），车心到后轴 1.45 m
"""
from __future__ import annotations

import heapq
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

# ---------- 规划参数 ----------
STEP = 0.70
XY_RES = 0.30
TH_RES = math.radians(9)
STEERS = (-1.0, -0.6, -0.25, 0.0, 0.25, 0.6, 1.0)
MARGIN = 0.10
SWITCH_COST = 15.0
X_CENTER_LINE = -0.05      # 对向车道中线 x=+0.02，车角留 7 cm 余量

# ---------- 车辆参数 ----------
H_RA = 1.45
K_STEER = 0.1834
EGO_MARGIN_X = 0.0

# ---------- 控制参数（速度层照抄 park_v8，垂直泊车 210 分那套）----------
STAGE_THROTTLE = 0.14      # 巡航恒定油门（开环，不闭环追速度）
SOFT_START_S = 3.0         # 起步前 2.5 s 油门封顶
SOFT_START_THR = 0.02
A_DEC_PLAN = 0.8           # 刹车规划用的减速度基准
V_REV = 0.55               # 倒车目标速度
STEER_SLEW = 0.12
THR_SLEW = 0.0008
BRK_SLEW = 0.0020
BRAKE_CAP = 0.06
THR_MAX = 0.03
BRK_MAX = 0.06

# 倒车级联
K_YAW = 1.6
K_DV = 20.0
S_DV = -1.0
DU_TARGET = 0.18
DU_TOL = 0.05
DV_TOL = 0.12
YAW_TOL = 5.0
HOLD_SECONDS = 5.0

STAGE_OFFSET = (3.58, 5.04, 90.0)   # 相对车位中心：车道侧 3.58 m、车位前 5.04 m
# 由实测车位 (-8.48,-2.04) 反推得到预置点 (-4.90, 3.00)，不写死绝对坐标


def wrap(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def heading_rad(yaw_deg: float) -> float:
    return math.radians(-yaw_deg)


def obb(x, y, ori, L, W):
    th = heading_rad(ori)
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
            a = [px * ax + py * ay for px, py in p]
            b = [px * ax + py * ay for px, py in q]
            best = min(best, min(max(a), max(b)) - max(min(a), min(b)))
    return -best


class Planner:
    def __init__(self, obstacles, ego_l, ego_w):
        self.obs = [obb(*o) for o in obstacles]
        self.L, self.W = ego_l, ego_w

    def free(self, x, y, ori):
        e = obb(x, y, ori, self.L, self.W)
        if max(c[0] for c in e) > X_CENTER_LINE:
            return False
        for o in self.obs:
            if sep(e, o) < MARGIN:
                return False
        return True

    def move(self, x, y, ori, steer, direction, ds):
        th = heading_rad(ori)
        ax = x - H_RA * math.cos(th)
        ay = y - H_RA * math.sin(th)
        s = direction * ds
        kappa = K_STEER * steer * direction
        if abs(kappa) < 1e-9:
            ax += s * math.cos(th)
            ay += s * math.sin(th)
        else:
            th2 = th + kappa * s
            R = 1 / kappa
            ax += R * (math.sin(th2) - math.sin(th))
            ay -= R * (math.cos(th2) - math.cos(th))
            th = th2
        return ax + H_RA * math.cos(th), ay + H_RA * math.sin(th), (-math.degrees(th)) % 360

    def plan(self, start, goal, max_expand=300000):
        def h(x, y, ori):
            return math.hypot(x - goal[0], y - goal[1]) + \
                math.radians(abs(wrap(ori - goal[2]))) * 1.2

        def key(x, y, ori):
            return (round(x / XY_RES), round(y / XY_RES), round(heading_rad(ori) / TH_RES))

        openq = [(h(*start), 0.0, 0, start[0], start[1], start[2], 0, None)]
        best = {key(*start): 0.0}
        came = {}
        nid = 1
        while openq and nid < max_expand:
            f, g, _, x, y, ori, dprev, parent = heapq.heappop(openq)
            if best.get(key(x, y, ori), 1e9) < g - 1e-6:
                continue
            if math.hypot(x - goal[0], y - goal[1]) <= 0.30 and abs(wrap(ori - goal[2])) <= 6:
                path = [(x, y, ori)]
                while parent is not None:
                    x, y, ori, parent = came[parent]
                    path.append((x, y, ori))
                path.reverse()
                return path
            for d in (1, -1):
                for s in STEERS:
                    nx, ny, no = self.move(x, y, ori, s, d, STEP)
                    if not self.free(nx, ny, no):
                        continue
                    ng = g + STEP * (1.3 if d < 0 else 1.0)
                    if dprev != 0 and d != dprev:
                        ng += SWITCH_COST
                    kk = key(nx, ny, no)
                    if ng < best.get(kk, 1e9):
                        best[kk] = ng
                        came[nid] = (x, y, ori, parent)
                        heapq.heappush(openq, (ng + h(nx, ny, no), ng, nid, nx, ny, no, d, nid))
                        nid += 1
        return None


class Pt:
    __slots__ = ("x", "y")

    def __init__(self, x, y):
        self.x, self.y = x, y


def pp_steer(x, y, yaw, path_pts, idx, lookahead, wheelbase=2.6, max_steer=0.6):
    """纯跟踪（与 metacar_race/control/tracking.py 完全一致，已验 110 分）。"""
    n = len(path_pts)
    lo = min(idx, n - 1)
    hi = min(n - 1, lo + 40)
    best_i, best_d = lo, 1e9
    for i in range(lo, hi + 1):
        d = math.hypot(path_pts[i].x - x, path_pts[i].y - y)
        if d < best_d:
            best_i, best_d = i, d
    idx = max(idx, best_i)
    tgt = path_pts[-1]
    for i in range(idx, n):
        if math.hypot(path_pts[i].x - x, path_pts[i].y - y) >= lookahead:
            tgt = path_pts[i]
            break
    dx, dy = tgt.x - x, tgt.y - y
    dist = max(math.hypot(dx, dy), 1e-3)
    th = heading_rad(yaw)
    lx = dx * math.cos(th) + dy * math.sin(th)
    ly = -dx * math.sin(th) + dy * math.cos(th)
    curv = 2.0 * ly / (dist * dist)
    delta = math.atan(curv * wheelbase)
    if lx < 0:
        delta = max_steer if ly > 0 else -max_steer
    return max(-1.0, min(1.0, -delta / max_steer)), idx


def speed_cmd(v_meas: float, v_target: float):
    """带死区的调速（park_v8 原样）：误差 0.06 以内不动，超速才轻刹。"""
    err = v_target - abs(v_meas)
    if abs(err) < 0.06:
        return 0.0, 0.0
    if err > 0:
        return min(THR_MAX, 0.004 + 0.020 * err), 0.0
    return 0.0, min(BRK_MAX, 0.08 * (-err))


def drive_to_point(v_meas: float, dist: float, thr_cruise: float = 0.02):
    """开环恒定油门 + 按剩余距离反推刹车（park_v8 的对位逻辑）。

    倒车/前进都靠这个：不闭环追速度，车速自己会平滑收敛到稳态；
    快到位时用 a_req = v²/(2d) 反推所需刹车量，减速是连续渐进的。
    """
    if dist < 20.0 and v_meas > 0.4:
        a_req = (v_meas * v_meas) / (2.0 * max(dist, 0.3))
        brk = max(0.0, min(BRAKE_CAP, (a_req - A_DEC_PLAN) / 25.0))
        if brk > 0.001:
            return 0.0, brk
    return thr_cruise, 0.0


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...", flush=True)
    api.connect()
    print("已连接。", flush=True)

    st = {"steer": 0.0, "thr": 0.0, "brk": 0.0, "t0": time.time()}
    plan_holder = {"path": None, "sw": None, "err": None, "done": False}
    slot = None
    geom = {}

    def send(gear, thr, brk, steer):
        # 转向、刹车做斜率限制；油门是 PWM 脉冲，直接下发（否则会被抹平）
        st["steer"] += max(-STEER_SLEW, min(STEER_SLEW, steer - st["steer"]))
        st["thr"] += max(-THR_SLEW, min(THR_SLEW, thr - st["thr"]))
        st["brk"] += max(-BRK_SLEW, min(BRK_SLEW, brk - st["brk"]))
        st["steer"] = max(-1.0, min(1.0, st["steer"]))
        st["thr"] = max(0.0, min(1.0, st["thr"]))
        st["brk"] = max(0.0, min(1.0, st["brk"]))
        # 互斥：油门和刹车绝不同时非零
        if st["brk"] > 0.002:
            st["thr"] = 0.0
        api.set_vehicle_control(VehicleControl(gear=gear, throttle=st["thr"],
                                               brake=st["brk"], steering=st["steer"]))

    phase = "SEEK"
    fwd_pts = []
    fwd_idx = 0
    hold_t = 0.0
    last_log = 0.0
    planner = None
    # 逐帧日志：用于事后算加速度尖峰
    log_dir = Path(__file__).resolve().parents[1] / "logs" / "angled"
    log_dir.mkdir(parents=True, exist_ok=True)
    csv_fh = open(log_dir / f"park6_{time.strftime('%Y%m%d-%H%M%S')}.csv",
                  "w", encoding="utf-8-sig", newline="")
    import csv as _csv
    csv_w = _csv.writer(csv_fh)
    csv_w.writerow(["frame", "t", "phase", "x", "y", "yaw", "v", "thr", "brk", "steer", "gear"])
    frame_no = 0
    t_prev = time.time()

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        car = msg.main_vehicle
        v = car.speed
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        now = time.time()
        dt = max(0.02, min(0.10, now - t_prev))
        t_prev = now
        frame_no += 1
        csv_w.writerow([frame_no, round(now - st["t0"], 3), phase, round(x, 3), round(y, 3),
                        round(yaw, 2), round(v, 3), round(st["thr"], 4), round(st["brk"], 4),
                        round(st["steer"], 3), getattr(car.gear, "name", "")])
        if frame_no % 30 == 0:
            csv_fh.flush()

        if slot is None:
            cars = []
            for ob in (msg.obstacles or []):
                nm = getattr(getattr(ob, "type", None), "name", str(getattr(ob, "type", "")))
                rec = (float(ob.pos_x), float(ob.pos_y), float(ob.ori_z),
                       float(ob.length), float(ob.width))
                if "PARK" in nm.upper():
                    slot = rec
                elif "CAR" in nm.upper():
                    cars.append(rec)
            if slot is None:
                send(GearMode.PARKING, 0.0, 0.35, 0.0)
                continue
            th = heading_rad(slot[2])
            geom = {"ux": math.cos(th), "uy": math.sin(th)}
            geom["vx"], geom["vy"] = -geom["uy"], geom["ux"]
            ego_l, ego_w = float(car.length), float(car.width)
            print(f"车位 ({slot[0]:.2f},{slot[1]:.2f}) ori={slot[2]:.1f} "
                  f"{slot[3]:.2f}x{slot[4]:.2f}  自车 {ego_l:.2f}x{ego_w:.2f}", flush=True)
            planner = Planner(cars, ego_l, ego_w)
            stg = (slot[0] + STAGE_OFFSET[0], slot[1] + STAGE_OFFSET[1], STAGE_OFFSET[2])
            print(f"预置点 ({stg[0]:.2f},{stg[1]:.2f})", flush=True)

            def _work():
                try:
                    t = time.time()
                    path = planner.plan(stg, (slot[0], slot[1], slot[2]))
                    if path:
                        sw = None
                        for i in range(4, len(path)):
                            if path[i][1] > path[i - 1][1] + 0.05:
                                sw = i
                                break
                        plan_holder["path"], plan_holder["sw"] = path, sw
                        print(f"[规划] {len(path)} 点，换向点 {sw}，{time.time() - t:.1f}s", flush=True)
                        for k in range(0, len(path), max(1, len(path) // 12)):
                            px, py, po = path[k]
                            print(f"   {k:3d} ({px:7.2f},{py:7.2f}) {po:6.1f}", flush=True)
                    else:
                        plan_holder["err"] = "无解"
                        print("[规划] 无解", flush=True)
                except Exception as e:  # noqa: BLE001
                    plan_holder["err"] = repr(e)
                    print(f"[规划] 异常 {e!r}", flush=True)
                plan_holder["done"] = True

            threading.Thread(target=_work, daemon=True).start()
            phase = "START"
            continue

        ux, uy, vx, vy = geom["ux"], geom["uy"], geom["vx"], geom["vy"]
        du = (x - slot[0]) * ux + (y - slot[1]) * uy
        dv = (x - slot[0]) * vx + (y - slot[1]) * vy

        # ---------------- 0) 起步：先驻车稳住，再用极小油门点起来 ----------------
        if phase == "START":
            if now - st["t0"] < 1.0:
                send(GearMode.PARKING, 0.0, 1.0, 0.0)
            elif abs(v) < 0.35:
                send(GearMode.DRIVE, 0.005, 0.0, 0.0)
            else:
                phase = "CRUISE"
                print(f"[{now - st['t0']:5.1f}s] 起步完成 v={v:.2f}", flush=True)
            continue

        # ---------------- 1) 车道巡航到预置点 ----------------
        if phase == "CRUISE":
            remaining = math.hypot(stg[0] - x, stg[1] - y)
            if remaining < 18.0:
                thr, brk = drive_to_point(v, remaining)
            else:
                thr, brk = speed_cmd(v, 1.2)
            # 直线跟踪：目标点 = 预置点
            dx, dy = stg[0] - x, stg[1] - y
            th = heading_rad(yaw)
            ly = -dx * math.sin(th) + dy * math.cos(th)
            steer = max(-0.6, min(0.6, -ly * 0.35))
            send(GearMode.DRIVE, thr, brk, steer)
            if now - last_log > 1.0:
                last_log = now
                print(f"[{now - st['t0']:5.1f}s] CRUISE ({x:.2f},{y:.2f}) yaw={yaw:.1f} "
                      f"v={v:.2f} 剩余{remaining:.1f} 规划={'✓' if plan_holder['done'] else '…'}",
                      flush=True)
            if remaining < 1.2 and plan_holder["done"] and plan_holder["path"]:
                # 规划已就绪就不停车，直接接摆头段，少一次起步瞬态
                path = plan_holder["path"]
                sw = plan_holder["sw"]
                fwd_pts = [Pt(px, py) for px, py, _ in path[:sw + 1]]
                fwd_idx = 0
                phase = "FWD"
                print(f"[{now - st['t0']:5.1f}s] 不停车直接接摆头（{len(fwd_pts)} 点）", flush=True)
            elif remaining < 0.35:
                phase = "WAITPLAN"
                print(f"[{now - st['t0']:5.1f}s] 到预置点，等规划", flush=True)
            continue

        # ---------------- 2) 等规划、停稳 ----------------
        if phase == "WAITPLAN":
            send(GearMode.DRIVE, 0.0, min(0.10, abs(v) * 2.0), 0.0)
            if plan_holder["done"] and abs(v) < 0.08:
                if plan_holder["path"] is None:
                    print("规划失败，驻车退出", flush=True)
                    send(GearMode.PARKING, 0.0, 0.6, 0.0)
                    break
                path = plan_holder["path"]
                sw = plan_holder["sw"]
                fwd_pts = [Pt(px, py) for px, py, _ in path[:sw + 1]]
                fwd_idx = 0

                phase = "FWD"
                print(f"[{now - st['t0']:5.1f}s] 开始摆头（{len(fwd_pts)} 点）", flush=True)
            continue

        # ---------------- 3) 前进摆头 ----------------
        if phase == "FWD":
            steer, fwd_idx = pp_steer(x, y, yaw, fwd_pts, fwd_idx, lookahead=1.8)
            remain_fwd = math.hypot(fwd_pts[-1].x - x, fwd_pts[-1].y - y)
            # 只用 PWM 给油，绝不额外加大油门；只有"靠近终点且速度偏高"时才刹车
            thr, brk = speed_cmd(v, 1.2)
            if remain_fwd < 5.0 and v > 0.4:
                _, brk2 = drive_to_point(v, remain_fwd, 0.0)
                if brk2 > 0.001:
                    thr, brk = 0.0, brk2
            send(GearMode.DRIVE, thr, brk, steer)
            if now - last_log > 0.8:
                last_log = now
                print(f"[{now - st['t0']:5.1f}s] FWD ({x:.2f},{y:.2f}) yaw={yaw:.1f} "
                      f"v={v:.2f} idx={fwd_idx}/{len(fwd_pts)}", flush=True)
            if fwd_idx >= len(fwd_pts) - 1:
                phase = "STOP"
                print(f"[{now - st['t0']:5.1f}s] 摆头完成，停稳换挡", flush=True)
            continue

        # ---------------- 4) 停稳 ----------------
        if phase == "STOP":
            send(GearMode.DRIVE, 0.0, min(0.12, abs(v) * 2.0), 0.0)
            if abs(v) < 0.05:
                st["thr"] = st["brk"] = 0.0
                phase = "REV"
                print(f"[{now - st['t0']:5.1f}s] 停稳，挂倒挡  du={du:+.2f} dv={dv:+.2f}", flush=True)
            continue

        # ---------------- 5) 倒车入位（车位坐标系级联） ----------------
        if phase == "REV":
            yaw_des = slot[2] + S_DV * K_DV * dv
            steer = max(-1.0, min(1.0, K_YAW * wrap(yaw - yaw_des)))
            thr, brk = speed_cmd(v, V_REV)
            send(GearMode.REVERSE, thr, brk, steer)
            if now - last_log > 0.6:
                last_log = now
                print(f"[{now - st['t0']:5.1f}s] REV ({x:.2f},{y:.2f}) yaw={yaw:.1f} "
                      f"v={v:.2f} du={du:+.2f} dv={dv:+.2f} steer={steer:+.2f}", flush=True)
            if (abs(du - DU_TARGET) < DU_TOL and abs(dv) < DV_TOL
                    and abs(wrap(yaw - slot[2])) < YAW_TOL):
                phase = "SETTLE"
                print(f"[{now - st['t0']:5.1f}s] 到位 du={du:+.2f} dv={dv:+.2f} "
                      f"航向差={wrap(yaw - slot[2]):+.1f}°，先刹停", flush=True)
            elif du < DU_TARGET - 0.9:
                phase = "SETTLE"
                print(f"[{now - st['t0']:5.1f}s] 冲过头 du={du:+.2f}，先刹停", flush=True)
            continue

        # ---------------- 5.5) 刹停后再挂 P（车还在动就切挡会让速度瞬跳） ----------------
        if phase == "SETTLE":
            if abs(v) > 0.05:
                send(GearMode.REVERSE, 0.0, BRAKE_CAP, 0.0)
            else:
                st["thr"] = st["brk"] = 0.0
                phase = "HOLD"
                hold_t = now
                print(f"[{now - st['t0']:5.1f}s] 已停稳，挂 P 保持", flush=True)
            continue

        # ---------------- 6) 驻车保持 ----------------
        if phase == "HOLD":
            send(GearMode.PARKING, 0.0, min(0.6, 0.20 + 0.3 * (now - hold_t)), 0.0)
            if now - hold_t > HOLD_SECONDS:
                print(f"[{now - st['t0']:5.1f}s] 保持 {HOLD_SECONDS:.0f}s 完成，"
                      f"du={du:+.2f} dv={dv:+.2f} 航向差={wrap(yaw - slot[2]):+.1f}°", flush=True)
                break
            continue

        if now - st["t0"] > 55:
            print("超时", flush=True)
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())



















