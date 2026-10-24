# -*- coding: utf-8 -*-
r"""「十字路口通行」入口（场景元素 18，200 分）。

规则（表 18）：
  * 200 s 内从起点到终点
  * 压线 −20
  * 碰撞对手车 −5×v
  * **违反交通信号 −5×v**
  * **红灯时车头距停止线 10 m 进入检测区，必须在距停止线 0~3 m 内停下，否则 −20**

现场因素（全部运行时读取）：
  * 路线     <- SceneStaticData.route（69 m 直行，穿过十字路口）
  * 停止线   <- 由道路几何推出：与车头方向一致、终点在前方的道路端点
  * 信号灯   <- SimMsg.traffic_light_groups[].traffic_lights[].straight_state
  * 弯道限速 <- 曲率 R 反推 v=sqrt(a_lat*R)

控制：
  * 横向：车身坐标系纯跟踪（与丁字路口同源）
  * 纵向：PI 连续量，油门封顶 0.40（永不升挡，避免换挡速度跳变打断 |a|<=5）
  * 红灯：把 v_cmd 按"到停止线的剩余距离"收缩，平滑停在 0~3 m 内
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from metacar import GearMode, ObstacleType, SceneAPI, VehicleControl
from metacar.models import TrafficLightState

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

KMH = 1.0 / 3.6

# ---- 物理上限（评分规则 + 车辆参数，不是场景常数）----
A_LAT_BUDGET = 4.0
WHEELBASE = 2.6
MAX_STEER = 0.6
THR_CAP = 0.40          # 油门输出上限：低于升挡门槛
STOP_GAP_MIN = 0.6      # 红灯停车：距停止线下限（规则要求 0~3 m，留点余量）
STOP_GAP_MAX = 2.6      # 上限


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def resample(pts, step=0.5):
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    acc = 0.0
    for i in range(1, len(pts)):
        p0, p1 = pts[i - 1], pts[i]
        seg = math.dist(p0, p1)
        if seg < 1e-9:
            continue
        while acc + seg >= step:
            t = (step - acc) / seg
            q = (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
            out.append(q)
            p0 = q
            seg = math.dist(p0, p1)
            acc = 0.0
        acc += seg
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def curvature_radius(pts, i, k=5):
    a = pts[max(0, i - k)]
    b = pts[i]
    c = pts[min(len(pts) - 1, i + k)]
    A, B, C = math.dist(a, b), math.dist(b, c), math.dist(a, c)
    if A < 1e-9 or B < 1e-9 or C < 1e-9:
        return float("inf")
    area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2.0
    return float("inf") if area2 < 1e-9 else (A * B * C) / (4.0 * area2)


class BodyFrameFollower:
    def __init__(self, route, look_factor=1.2):
        self.pts = resample(route, 0.5)
        self.look_factor = look_factor
        self.idx = 0
        self.R = [curvature_radius(self.pts, i, 5) for i in range(len(self.pts))]
        self.v_cap = [math.sqrt(A_LAT_BUDGET * r) if r < 1e8 else 1e9 for r in self.R]

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

    def control(self, x, y, yaw_deg, v):
        i = self._nearest(x, y)
        look = max(4.0, min(14.0, self.look_factor * max(abs(v), 1.0) + 2.0))
        acc, j = 0.0, i
        while j < len(self.pts) - 1 and acc < look:
            acc += math.dist(self.pts[j], self.pts[j + 1])
            j += 1
        tx, ty = self.pts[min(j, len(self.pts) - 1)]

        dx, dy = tx - x, ty - y
        psi = math.radians(yaw_deg)
        hx, hy = math.cos(psi), -math.sin(psi)
        lx, ly = math.sin(psi), math.cos(psi)
        front = dx * hx + dy * hy
        left = dx * lx + dy * ly
        dist = max(math.hypot(front, left), 1e-3)
        curvature = 2.0 * left / (dist * dist)
        delta = math.atan(curvature * WHEELBASE)
        steer = max(-1.0, min(1.0, -delta / MAX_STEER))

        v_cmd = min(self.v_cap[i:min(len(self.v_cap), i + 40)] or [1e9])
        return steer, v_cmd, i

    def end_dir(self):
        a, b = self.pts[-2], self.pts[-1]
        n = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
        return (b[0] - a[0]) / n, (b[1] - a[1]) / n


class Longitudinal:
    """PI 纵向控制，输出连续的油门/刹车。"""

    def __init__(self):
        self.integ = 0.0

    def step(self, v, v_cmd, dt):
        err = v_cmd - v
        self.integ = max(-3.0, min(24.0, self.integ + err * dt))
        u = 0.10 * err + 0.05 * self.integ
        if u >= 0:
            return min(THR_CAP, u), 0.0
        return 0.0, min(1.0, -u * 1.6)


def pick_stop_line(roads, x, y, yaw_deg, max_dist=80.0):
    """直接从道路数据里的 stopLine 取停止线（SDK 已提供）。

    选"在车正前方、且最近"的那条，返回其中点。
    """
    psi = math.radians(yaw_deg)
    hx, hy = math.cos(psi), -math.sin(psi)
    best = None
    for rd in roads:
        pts = getattr(rd, "stop_line", None) or []
        if not pts:
            continue
        mx = sum(p.x for p in pts) / len(pts)
        my = sum(p.y for p in pts) / len(pts)
        dx, dy = mx - x, my - y
        dist = math.hypot(dx, dy)
        if not (0.5 < dist < max_dist):
            continue
        ahead = (dx * hx + dy * hy) / dist
        lateral = abs(dx * hy - dy * hx)
        if ahead > 0.8 and lateral < 10.0:
            if best is None or dist < best[0]:
                best = (dist, (mx, my), rd)
    return best


def pick_light(groups, x, y, yaw_deg, max_dist=60.0):
    """挑出本车方向上、前方最近的一组信号灯，返回直行状态。"""
    psi = math.radians(yaw_deg)
    hx, hy = math.cos(psi), -math.sin(psi)
    best = None
    for g in groups:
        for tl in g.traffic_lights:
            dx, dy = tl.position.x - x, tl.position.y - y
            dist = math.hypot(dx, dy)
            if dist > max_dist:
                continue
            ahead = (dx * hx + dy * hy) / max(dist, 1e-3)
            if ahead > 0.3 and (best is None or dist < best[0]):
                best = (dist, tl)
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description="十字路口通行")
    ap.add_argument("--seconds", type=float, default=190.0)
    args = ap.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    st = api.get_scene_static_data()
    route = [(float(p.x), float(p.y)) for p in st.route]
    print(f"路线点: {len(route)}  子场景: {[s.name for s in st.sub_scenes]}", flush=True)
    print(f"道路数: {len(st.roads)}", flush=True)

    follower = BodyFrameFollower(route)
    longi = Longitudinal()
    print(f"曲率: 最小 R={min(follower.R):.2f} m -> 允许速度 "
          f"{math.sqrt(A_LAT_BUDGET * min(follower.R)):.2f} m/s", flush=True)

    frames = 0
    t0 = time.time()
    last_t = None
    v_peak = 0.0
    max_a = 0.0
    prev_v = 0.0
    stop_line = None
    light_seen = False
    stopped_for_red = False
    arrived = False
    passed = False

    for msg, cams in api.main_loop():
        frames += 1
        car = msg.main_vehicle
        pose = msg.pose_gnss
        status = msg.scene_status
        dt = 1.0 / 30.0 if last_t is None else max(status.used_time - last_t, 1e-3)
        last_t = status.used_time

        steer, v_cmd, idx = follower.control(pose.pos_x, pose.pos_y, pose.ori_z, car.speed)

        # ---- 停止线（只找一次）----
        if stop_line is None:
            got = pick_stop_line(st.roads, pose.pos_x, pose.pos_y, pose.ori_z)
            if got:
                stop_line = got[1]
                print(f"== 停止线 ({stop_line[0]:.2f}, {stop_line[1]:.2f}) "
                      f"距车 {got[0]:.1f} m ==", flush=True)
            elif frames > 60:
                print("!! 没找到停止线，本场景可能没有信号灯，直接通行", flush=True)

        # ---- 信号灯 ----
        light = pick_light(msg.traffic_light_groups, pose.pos_x, pose.pos_y, pose.ori_z)
        if light and not light_seen:
            light_seen = True
            tl = light[1]
            print(f"== 找到信号灯 id={tl.id} 距 {light[0]:.1f} m "
                  f"直行={tl.straight_state} 剩余={tl.straight_remaining_time:.1f}s ==", flush=True)

        if stop_line is not None:
            dist_sl = math.hypot(stop_line[0] - pose.pos_x, stop_line[1] - pose.pos_y)
            red = light is not None and light[1].straight_state == TrafficLightState.RED
            if red and 0.0 < dist_sl < 30.0 and not passed:
                # 用剩余距离收缩目标速度，等价于"匀减速到停"：
                # v_allow = sqrt(2*a*d)，a 取 1.2 m/s^2
                gap = max(dist_sl - STOP_GAP_MAX, 0.0)
                v_allow = math.sqrt(2.0 * 1.2 * gap)
                v_cmd = min(v_cmd, v_allow)
                if dist_sl <= STOP_GAP_MAX + 0.3:
                    v_cmd = 0.0
                    stopped_for_red = True
            if stopped_for_red and light is not None and light[1].straight_state == TrafficLightState.GREEN:
                stopped_for_red = False
                print("== 绿灯，继续通行 ==", flush=True)
            if dist_sl < 0.8:
                passed = True
                print("== 已通过停止线 ==", flush=True)

        # ---- 纵向 ----
        thr, brk = longi.step(car.speed, v_cmd, dt)
        api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                               brake=brk, steering=steer))
        v_peak = max(v_peak, abs(car.speed))

        if frames > 2 and dt >= 0.015:
            a_lon = (car.speed - prev_v) / dt
            R = follower.R[idx]
            a_lat = (car.speed ** 2) / R if R < 1e8 else 0.0
            max_a = max(max_a, math.hypot(a_lon, a_lat))
        prev_v = car.speed

        # ---- 终点：越过终点 2 m 再刹停 ----
        ex, ey = follower.pts[-1]
        dirx, diry = follower.end_dir()
        along = (pose.pos_x - ex) * dirx + (pose.pos_y - ey) * diry
        if along > 2.0:
            arrived = True
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if frames % 15 == 1:
                print("== 已越过终点，刹停 ==", flush=True)
            if time.time() - t0 > 40.0:
                break

        if frames % 15 == 1:
            state = "-"
            if light:
                state = light[1].straight_state.name
            print(f"[{status.used_time:6.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={car.speed:5.2f} v_cmd={v_cmd:5.2f} "
                  f"light={state:5s} thr={thr:.2f} brk={brk:.2f} idx={idx}", flush=True)

        if time.time() - t0 > args.seconds:
            print("到达脚本时长上限，退出", flush=True)
            break

    summary = {
        "frames": frames,
        "v_peak_kmh": round(v_peak * 3.6, 2),
        "max_a_total": round(max_a, 2),
        "arrived": arrived,
        "stop_line": stop_line,
        "light_seen": light_seen,
        "elapsed": round(time.time() - t0, 1),
    }
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
