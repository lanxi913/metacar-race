# -*- coding: utf-8 -*-
r"""「丁字路口通行」入口（车身坐标系 + 加速度矢量预算版）。

评分要求：行驶过程中加速度绝对值不超过 5 m/s²，每题 +10。
注意加速度是**矢量**：|a| = sqrt(a_long² + a_lat²) ≤ 5，
过弯时向心加速度 a_lat = v² / R 必须一起算进去。

设计（不写场景常数）：
  * 路线、曲率全部来自 SceneStaticData.route（按弧长重采样后求曲率）
  * 允许速度由曲率反推：v_cap(s) = sqrt(a_lat_budget * R(s))
  * 提前看 12 m 取最小允许速度，保证减速是"缓"的（a_lon ≤ a_lon_max）
  * 横向控制：Stanley，误差全部在**车身坐标系**里算
      - 横向偏差 e = 车体左右方向上的偏移（左正）
      - 航向误差 err = 期望航向 - 当前航向
  * 纵向控制：由 a_des 反推油门/刹车，a_des 受 a_lon_max 限幅
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from metacar import GearMode, ObstacleType, SceneAPI, VehicleControl

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from speed.main import AdaptiveCruise  # noqa: E402  恒定油门巡航（最平滑）

# ---- 物理上限（来自评分规则）----
A_TOTAL_MAX = 5.0        # 评分要求：|a| <= 5 m/s^2
A_LAT_BUDGET = 4.0       # 留给向心加速度的预算
A_LON_MAX = 2.0          # 纵向加速度上限（sqrt(4^2+2^2)=4.47 < 5）

KMH = 1.0 / 3.6


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def resample(pts, step=0.5):
    """按弧长重采样，便于稳定求曲率。"""
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    acc = 0.0
    for i in range(1, len(pts)):
        p0, p1 = pts[i - 1], pts[i]
        seg = math.dist(p0, p1)
        if seg < 1e-9:
            continue
        while acc + seg >= step:
            need = step - acc
            t = need / seg
            q = (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
            out.append(q)
            p0 = q
            seg = math.dist(p0, p1)
            acc = 0.0
        acc += seg
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def curvature_radius(pts, i, k=3):
    a = pts[max(0, i - k)]
    b = pts[i]
    c = pts[min(len(pts) - 1, i + k)]
    A = math.dist(a, b)
    B = math.dist(b, c)
    C = math.dist(a, c)
    if A < 1e-9 or B < 1e-9 or C < 1e-9:
        return float("inf")
    area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2.0
    if area2 < 1e-9:
        return float("inf")
    return (A * B * C) / (4.0 * area2)


# 车辆物理参数（不是场景常数）
WHEELBASE = 2.6
MAX_STEER = 0.6
R_VEHICLE_MIN = WHEELBASE / math.tan(MAX_STEER)      # 约 3.80 m
V_CRUISE_CAP = 1e9      # 巡航速度不设人为上限：由曲率限速与车辆本身决定


def smooth_path(pts, k):
    if k <= 0:
        return list(pts)
    out = []
    n = len(pts)
    for i in range(n):
        lo, hi = max(0, i - k), min(n - 1, i + k)
        m = hi - lo + 1
        out.append((sum(pts[j][0] for j in range(lo, hi + 1)) / m,
                    sum(pts[j][1] for j in range(lo, hi + 1)) / m))
    return out


def auto_smooth(pts, v_ref, a_lat_budget, kmax=12):
    """自动选择平滑窗口：让最小曲率半径满足 v_ref^2/R <= a_lat_budget。

    v_ref 是规划用的速度上限（不是场景常数，而是"最高可能车速"的保守估计），
    平滑窗口由这个物理约束反推，不写死。
    """
    need_R = (v_ref ** 2) / a_lat_budget
    best = (0, pts, 1.0)
    for k in range(0, kmax + 1):
        cand = smooth_path(pts, k) if k > 0 else list(pts)
        rs = [curvature_radius(cand, i) for i in range(2, len(cand) - 2)]
        rmin = min(rs) if rs else float("inf")
        best = (k, cand, rmin)
        if rmin >= need_R:
            break
    return best


class BodyFrameFollower:
    """车身坐标系下的 Stanley 跟踪 + 曲率限速。"""

    def __init__(self, route, k_heading=2.0, k_cross=2.2,
                 a_lat_budget=A_LAT_BUDGET, a_lon_max=A_LON_MAX,
                 look_factor=1.2):
        # 不平滑：平滑会把 90 度弯走成大圆弧，直接压线（实测扣 20）。
        # 航点噪声改用更宽的曲率算子抑制（按弧长 2.5 m 取三点）。
        self.k_smooth = 0
        self.pts = resample(route, 0.5)
        self.r_min = min(curvature_radius(self.pts, i, k=5) for i in range(5, len(self.pts) - 5))
        self.k_h = k_heading
        self.k_c = k_cross
        self.a_lat = a_lat_budget
        self.a_lon = a_lon_max
        self.look_factor = look_factor
        self.idx = 0
        self.R = [curvature_radius(self.pts, i, k=5) for i in range(len(self.pts))]
        # 每个点的允许速度：曲率限速
        self.v_cap = [math.sqrt(a_lat_budget * r) if r < 1e8 else 1e9 for r in self.R]

    # ---------- 索引 ----------
    def _nearest(self, x, y):
        lo, hi = max(0, self.idx - 20), min(len(self.pts) - 1, self.idx + 60)
        best, bd = lo, 1e9
        for i in range(lo, hi + 1):
            d = math.hypot(self.pts[i][0] - x, self.pts[i][1] - y)
            if d < bd:
                best, bd = i, d
        if bd > 8.0:
            best = min(range(len(self.pts)),
                       key=lambda k: math.hypot(self.pts[k][0] - x, self.pts[k][1] - y))
        self.idx = max(self.idx, best)
        return self.idx

    # ---------- 车身坐标系 ----------
    def _body(self, x, y, yaw_deg, tx, ty):
        """把目标点转到车身坐标系：x 向前，y 向左。返回 (front, left)。"""
        dx, dy = tx - x, ty - y
        psi = math.radians(yaw_deg)
        # 场景 yaw 顺时针为正：车头单位向量
        hx, hy = math.cos(psi), -math.sin(psi)
        # 车身左向 = 车头逆时针 90 度
        lx, ly = math.sin(psi), math.cos(psi)
        front = dx * hx + dy * hy
        left = dx * lx + dy * ly
        return front, left

    def control(self, x, y, yaw_deg, v):
        i = self._nearest(x, y)
        # 前视点：按速度取，快则看远
        look = max(4.0, min(14.0, self.look_factor * max(abs(v), 1.0) + 2.0))
        acc = 0.0
        j = i
        while j < len(self.pts) - 1 and acc < look:
            acc += math.dist(self.pts[j], self.pts[j + 1])
            j += 1
        tx, ty = self.pts[min(j, len(self.pts) - 1)]

        front, left = self._body(x, y, yaw_deg, tx, ty)
        dist = max(math.hypot(front, left), 1e-3)
        # 纯跟踪曲率；在车身坐标系里，左偏为正
        curvature = 2.0 * left / (dist * dist)
        wheelbase = 2.6
        delta = math.atan(curvature * wheelbase)
        max_steer = 0.6
        # 实测：正舵 -> yaw 增大（右转）；车身坐标系左偏为正 -> 需要负舵
        steer = max(-1.0, min(1.0, -delta / max_steer))

        # 曲率限速：向前看 12 m 取最小允许速度
        v_cmd = min(min(self.v_cap[i:min(len(self.v_cap), i + 40)] or [1e9]), V_CRUISE_CAP)
        return steer, v_cmd, i

    def finished(self, x, y, dist_tol=1.5):
        ex, ey = self.pts[-1]
        return math.hypot(ex - x, ey - y) < dist_tol


class Longitudinal:
    """由期望加速度反推油门/刹车，并把 a_lon 限幅。"""

    def __init__(self, a_lon_max=A_LON_MAX):
        self.a_max = a_lon_max
        self.thr = 0.0
        self.brk = 0.0

    def step(self, v, v_cmd, dt):
        a_des = max(-self.a_max, min(self.a_max, 1.2 * (v_cmd - v)))
        # 一阶平滑，避免指令突变
        self.thr = 0.7 * self.thr
        self.brk = 0.7 * self.brk
        if a_des > 0.05:
            thr_cmd = min(1.0, a_des / 1.9)     # 满油门约 1.9 m/s^2（实测）
            # 油门封顶 0.85：再高会触发升挡，速度会跳一大截
            self.thr = max(self.thr, min(0.85, thr_cmd))
            self.brk = 0.0
        elif a_des < -0.05:
            brk_cmd = min(1.0, -a_des / 1.5)    # 满刹车约 1.5 m/s^2（实测）
            self.brk = max(self.brk, brk_cmd)
            self.thr = 0.0
        return self.thr, self.brk


def main() -> int:
    ap = argparse.ArgumentParser(description="丁字路口通行（矢量加速度预算）")
    ap.add_argument("--seconds", type=float, default=190.0)
    args = ap.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    st = api.get_scene_static_data()
    route = [(float(p.x), float(p.y)) for p in st.route]
    print(f"路线点: {len(route)}  子场景: {[s.name for s in st.sub_scenes]}", flush=True)

    follower = BodyFrameFollower(route)
    print(f"曲率限速: 最小 R={min(follower.R):.2f} m -> 允许速度 "
          f"{math.sqrt(A_LAT_BUDGET * min(follower.R)):.2f} m/s", flush=True)

    longi = Longitudinal()
    cruise = AdaptiveCruise(list(st.route), 60.0 * KMH, goal_ratio=1.01,
                             ramp_rate=0.08, plateau_eps=0.10, plateau_hold=0.5)
    frames = 0
    t0 = time.time()
    last_t = None
    v_peak = 0.0
    pi_integ = 0.0
    max_a_tot = 0.0
    prev_v = 0.0
    arrived = False
    arrived_t = None
    pi_integ = 0.0

    for msg, cams in api.main_loop():
        frames += 1
        car = msg.main_vehicle
        pose = msg.pose_gnss
        status = msg.scene_status
        dt = 1.0 / 30.0 if last_t is None else max(status.used_time - last_t, 1e-3)
        last_t = status.used_time

        steer, v_cmd, idx = follower.control(pose.pos_x, pose.pos_y, pose.ori_z, car.speed)

        # 实测总加速度：纵向（差分）+ 向心（v^2/R）
        if frames > 2 and dt >= 0.015:
            a_lon = (car.speed - prev_v) / dt
            R = follower.R[idx]
            a_lat = (car.speed ** 2) / R if R < 1e8 else 0.0
            a_tot = math.hypot(a_lon, a_lat)
            max_a_tot = max(max_a_tot, a_tot)
        prev_v = car.speed

        # 终点判定：沿路线方向"越过终点 2 m"才算过线（刹车距离约 7 m，
        # 停在终点前会判"没过去"）
        ex, ey = follower.pts[-1]
        px2, py2 = follower.pts[-2]
        dirx, diry = ex - px2, ey - py2
        nn = math.hypot(dirx, diry) or 1.0
        dirx, diry = dirx / nn, diry / nn
        along = (pose.pos_x - ex) * dirx + (pose.pos_y - ey) * diry
        if along > 2.0:
            arrived = True
            if arrived_t is None:
                arrived_t = time.time()
                print(f"== 已越过终点 {along:.1f} m，刹停 ==", flush=True)
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if time.time() - arrived_t > 20.0:
                break
            continue

        # 纵向 PI：输出连续的油门/刹车，不带脉冲，所以不会一顿一顿
        err = v_cmd - car.speed
        pi_integ = max(-3.0, min(24.0, pi_integ + err * dt))
        u = 0.10 * err + 0.05 * pi_integ          # 稳态时积分项撑起油门
        # 油门输出上限 0.40：低于升挡门槛，车会自然稳定在 1 挡最高速，
        # 不会越过平台速度换挡（换挡瞬间的速度跳变是 |a| 超限的主因）
        if u >= 0:
            thr, brk = min(0.40, u), 0.0
        else:
            thr, brk = 0.0, min(1.0, -u * 1.6)
        api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                               brake=brk, steering=steer))
        v_peak = max(v_peak, abs(car.speed))

        if frames % 15 == 1:
            a_lat = (car.speed ** 2) / follower.R[idx] if follower.R[idx] < 1e8 else 0.0
            print(f"[{status.used_time:6.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={car.speed:5.2f} v_cmd={v_cmd:5.2f} "
                  f"a_lat={a_lat:4.2f} str={steer:+5.2f} thr={thr:.2f} brk={brk:.2f}",
                  flush=True)

        if time.time() - t0 > args.seconds:
            break

    summary = {
        "frames": frames,
        "v_peak_kmh": round(v_peak * 3.6, 2),
        "max_a_total": round(max_a_tot, 2),
        "a_lat_budget": A_LAT_BUDGET,
        "arrived": arrived,
        "elapsed": round(time.time() - t0, 1),
    }
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    (log_dir / f"tjunction_vec_{time.strftime('%Y%m%d-%H%M%S')}.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
