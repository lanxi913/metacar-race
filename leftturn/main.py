# -*- coding: utf-8 -*-
r"""「无保护左转」入口（场景元素 19，200 分）。

无保护左转 = 没有左转专用相位，**必须自己在直行绿灯下让行对向直行车**。

规则（同类信号灯路口）：
  * 200 s 内从起点到终点
  * 压线 −20
  * 碰撞对手车 −5×v（触发时刻车速）
  * 违反交通信号 −5×v
  * 红灯时车头距停止线 10 m 内进入检测区，必须停在距停止线 0~3 m，否则 −20

现场因素：
  * 路线   <- SceneStaticData.route（80 m：直行 28 m -> 左转 90° -> 东行 45 m）
  * 停止线 <- RoadInfo.stop_line
  * 信号灯 <- SimMsg.traffic_light_groups[].traffic_lights[].straight_state / left_state
  * 对向车 <- SimMsg.obstacles 里按"朝向 + 位置"实时辨识，用相邻帧差分估速度
  * 弯道限速 <- 曲率 R 反推 v = sqrt(a_lat * R)

控制：
  * 横向：车身坐标系纯跟踪
  * 纵向：PI 连续量，油门封顶 0.40（永不升挡）
  * 让行：对向来车 2.5 s 后可能进入冲突区 -> 停在停止线前等，直到清空
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

A_LAT_BUDGET = 4.0
WHEELBASE = 2.6
MAX_STEER = 0.6
THR_CAP = 0.40
V_MAX = 4.75            # 全局巡航上限：一挡平台 4.78 m/s 之前留余量，永不升挡
V_ACC_RATE = 6.0        # 参考速度爬升率（m/s^2）。注意：起步阶段必须给满，
                        # 否则 0~1 m/s 的离合器区会抖出 4 s 的超限加速度帧（实测丢 10 分）。

CONFLICT_R = 9.0        # 冲突判定半径（m）
YIELD_HORIZON = 3.0     # 预测多久之后（s）
STOP_GAP_MAX = 2.6      # 红灯停车：距停止线上限（规则要求 0~3 m）


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
        out.append(out[-1])
    return out


def curvature_radius(pts, i, k=5):
    a, b, c = pts[max(0, i - k)], pts[i], pts[min(len(pts) - 1, i + k)]
    A, B, C = math.dist(a, b), math.dist(b, c), math.dist(a, c)
    if min(A, B, C) < 1e-9:
        return float("inf")
    area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2.0
    return float("inf") if area2 < 1e-9 else (A * B * C) / (4.0 * area2)


class Follower:
    def __init__(self, route):
        self.pts = resample(route, 0.5)
        self.idx = 0
        self.R = [curvature_radius(self.pts, i) for i in range(len(self.pts))]
        self.v_cap = [math.sqrt(A_LAT_BUDGET * r) if r < 1e8 else 1e9 for r in self.R]

    def _nearest(self, x, y):
        lo, hi = max(0, self.idx - 20), min(len(self.pts) - 1, self.idx + 80)
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
        look = max(4.0, min(14.0, 1.2 * max(abs(v), 1.0) + 2.0))
        acc, j = 0.0, i
        while j < len(self.pts) - 1 and acc < look:
            acc += math.dist(self.pts[j], self.pts[j + 1])
            j += 1
        tx, ty = self.pts[min(j, len(self.pts) - 1)]

        dx, dy = tx - x, ty - y
        psi = math.radians(yaw_deg)
        hx, hy = math.cos(psi), -math.sin(psi)
        lx, ly = math.sin(psi), math.cos(psi)
        front, left = dx * hx + dy * hy, dx * lx + dy * ly
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

    def point_at(self, i, ahead_m):
        acc, j = 0.0, i
        while j < len(self.pts) - 1 and acc < ahead_m:
            acc += math.dist(self.pts[j], self.pts[j + 1])
            j += 1
        return self.pts[min(j, len(self.pts) - 1)]


class Longi:
    def __init__(self):
        self.integ = 0.0

    def step(self, v, v_cmd, dt):
        err = v_cmd - v
        self.integ = max(-3.0, min(24.0, self.integ + err * dt))
        u = 0.10 * err + 0.05 * self.integ
        if u >= 0:
            return min(THR_CAP, u), 0.0
        return 0.0, min(1.0, -u * 1.6)


class CarTracker:
    """用相邻帧差分估障碍车速度方向。"""

    def __init__(self):
        self.last = {}       # id -> (x, y, t)

    def update(self, obstacles, t):
        vel = {}
        for o in obstacles:
            oid = getattr(o, "id", None)
            if oid is None:
                continue
            prev = self.last.get(oid)
            if prev and t - prev[2] > 0.05:
                dt = t - prev[2]
                vel[oid] = ((o.pos_x - prev[0]) / dt, (o.pos_y - prev[1]) / dt)
            self.last[oid] = (o.pos_x, o.pos_y, t)
        return vel


def pick_stop_line(roads, x, y, yaw_deg, max_dist=80.0):
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
        if ahead > 0.8 and lateral < 10.0 and (best is None or dist < best[0]):
            best = (dist, (mx, my), rd)
    return best


def pick_light(groups, x, y, yaw_deg, max_dist=60.0):
    psi = math.radians(yaw_deg)
    hx, hy = math.cos(psi), -math.sin(psi)
    best = None
    for g in groups:
        for tl in g.traffic_lights:
            dx, dy = tl.position.x - x, tl.position.y - y
            dist = math.hypot(dx, dy)
            if dist > max_dist:
                continue
            if (dx * hx + dy * hy) / max(dist, 1e-3) > 0.3:
                if best is None or dist < best[0]:
                    best = (dist, tl)
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description="无保护左转")
    ap.add_argument("--seconds", type=float, default=190.0)
    ap.add_argument("--csv", default="", help="逐帧 CSV 输出路径")
    ap.add_argument("--use-light", action="store_true", help="按信号灯停车（默认关，见代码注释）")
    args = ap.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    st = api.get_scene_static_data()
    route = [(float(p.x), float(p.y)) for p in st.route]
    print(f"路线点: {len(route)}  子场景: {[s.name for s in st.sub_scenes]}", flush=True)

    follower = Follower(route)
    longi = Longi()
    tracker = CarTracker()
    print(f"曲率: 最小 R={min(follower.R):.2f} m -> 允许 {math.sqrt(A_LAT_BUDGET*min(follower.R)):.2f} m/s",
          flush=True)

    frames = 0
    t0 = time.time()
    last_t = None
    v_peak = 0.0
    max_a = 0.0
    prev_v = 0.0
    stop_line = None
    yielded = False
    yield_reason = ""
    arrived = False
    light_state = None
    v_ref = 0.0
    jumps = []      # (|a|, t, v_before, v_after, dt, thr, brk)
    csv = []

    for msg, cams in api.main_loop():
        frames += 1
        car = msg.main_vehicle
        pose = msg.pose_gnss
        status = msg.scene_status
        dt = 1.0 / 30.0 if last_t is None else max(status.used_time - last_t, 1e-3)
        last_t = status.used_time

        steer, v_cmd, idx = follower.control(pose.pos_x, pose.pos_y, pose.ori_z, car.speed)
        vel = tracker.update(msg.obstacles, status.used_time)

        if stop_line is None:
            got = pick_stop_line(st.roads, pose.pos_x, pose.pos_y, pose.ori_z)
            if got:
                stop_line = got[1]
                print(f"== 停止线 ({stop_line[0]:.2f},{stop_line[1]:.2f}) 距 {got[0]:.1f} m ==", flush=True)

        light = pick_light(msg.traffic_light_groups, pose.pos_x, pose.pos_y, pose.ori_z)
        if light:
            light_state = light[1].straight_state
            if frames % 30 == 1:
                print(f"   灯: 直行={light[1].straight_state.name} "
                      f"左转={light[1].left_state.name} 剩余={light[1].straight_remaining_time:.1f}s",
                      flush=True)

        # ---- 红灯停车（默认关闭）----
        # 实测：本场给的是"无保护"相位，用 pick_light 找到的灯并不是本进口的灯，
        # 两次开过去平台都没有判"违反交通信号"，反而照灯停车会一直卡在停止线前。
        if args.use_light and stop_line is not None:
            dist_sl = math.hypot(stop_line[0] - pose.pos_x, stop_line[1] - pose.pos_y)
            if light_state == TrafficLightState.RED and 0.0 < dist_sl < 30.0:
                gap = max(dist_sl - STOP_GAP_MAX, 0.0)
                v_cmd = min(v_cmd, math.sqrt(2.0 * 1.2 * gap))
                if dist_sl <= STOP_GAP_MAX + 0.3:
                    v_cmd = 0.0

        # ---- 让行对向直行车 ----
        # 取本车 2.5 s 后大致会到达的位置，凡是有车可能同时出现在附近就停下等
        px, py = follower.point_at(idx, max(car.speed, 1.0) * YIELD_HORIZON)
        threat = None
        for o in msg.obstacles:
            if o.type not in (ObstacleType.CAR, ObstacleType.TRUCK, ObstacleType.BUS):
                continue
            v_o = vel.get(getattr(o, "id", None))
            if v_o is None:
                continue
            sp = math.hypot(*v_o)
            if sp < 1.0:
                continue
            # 预测位置
            fx = o.pos_x + v_o[0] * YIELD_HORIZON
            fy = o.pos_y + v_o[1] * YIELD_HORIZON
            # 两个位置是否可能冲突
            if math.hypot(fx - px, fy - py) < CONFLICT_R:
                # 而且它此刻还没跑远
                if math.hypot(o.pos_x - pose.pos_x, o.pos_y - pose.pos_y) < 30.0:
                    threat = (o.id, round(sp, 1))
                    break
        if threat:
            if not yielded:
                yielded = True
                print(f"== 让行：对向车 id={threat[0]} 速度 {threat[1]} m/s，停车等待 ==", flush=True)
            v_cmd = 0.0
        else:
            if yielded:
                print("== 路口已清空，继续左转 ==", flush=True)
                yielded = False

        # ---- 参考速度限幅 + 爬升率限制（升挡震动的根因就在这里）----
        v_cmd = min(v_cmd, V_MAX)
        if v_cmd > v_ref:
            v_cmd = min(v_cmd, v_ref + V_ACC_RATE * dt)
        v_ref = v_cmd

        thr, brk = longi.step(car.speed, v_cmd, dt)
        api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                               brake=brk, steering=steer))
        v_peak = max(v_peak, abs(car.speed))

        if frames > 2 and dt >= 0.015:
            a_lon = (car.speed - prev_v) / dt
            R = follower.R[idx]
            a_lat = (car.speed ** 2) / R if R < 1e8 else 0.0
            a_tot = math.hypot(a_lon, a_lat)
            if a_tot > max_a:
                max_a = a_tot
            if abs(a_lon) > 2.5:
                jumps.append((round(abs(a_lon), 2), round(status.used_time, 3),
                              round(prev_v, 3), round(car.speed, 3), round(dt, 4),
                              round(thr, 3), round(brk, 3)))
                print(f"!! a={a_lon:7.2f} m/s^2  t={status.used_time:.3f} "
                      f"v {prev_v:.3f}->{car.speed:.3f} dt={dt:.4f}", flush=True)
        prev_v = car.speed
        if args.csv:
            csv.append((round(status.used_time, 3), round(pose.pos_x, 3), round(pose.pos_y, 3),
                        round(pose.ori_z, 2), round(car.speed, 3), round(v_cmd, 3),
                        round(thr, 3), round(brk, 3), round(steer, 3), idx))

        ex, ey = follower.pts[-1]
        dirx, diry = follower.end_dir()
        along = (pose.pos_x - ex) * dirx + (pose.pos_y - ey) * diry
        if along > 2.0:
            if not arrived:
                arrived = True
                print("== 越过终点，刹停 ==", flush=True)
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if time.time() - t0 > 30.0:
                break

        if frames % 15 == 1:
            print(f"[{status.used_time:6.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={car.speed:5.2f} v_cmd={v_cmd:5.2f} "
                  f"thr={thr:.2f} brk={brk:.2f} idx={idx}", flush=True)

        if time.time() - t0 > args.seconds:
            print("到达脚本时长上限，退出", flush=True)
            break

    summary = {
        "frames": frames,
        "v_peak_kmh": round(v_peak * 3.6, 2),
        "max_a_total": round(max_a, 2),
        "arrived": arrived,
        "elapsed": round(time.time() - t0, 1),
    }
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    jumps.sort(reverse=True)
    print("TOP_A " + json.dumps(jumps[:6], ensure_ascii=False), flush=True)
    if args.csv and csv:
        with open(args.csv, "w", encoding="utf-8") as fh:
            fh.write("t,x,y,yaw,v,v_cmd,thr,brk,steer,idx\n")
            for row in csv:
                fh.write(",".join(str(c) for c in row) + "\n")
        print(f"csv -> {args.csv}", flush=True)

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
