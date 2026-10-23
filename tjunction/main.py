# -*- coding: utf-8 -*-
r"""「丁字路口通行」入口（场景元素 17，200 分）。

规则：
  * 200 s 内从起点到终点
  * 压线 -20
  * 碰撞对手车 -5*v（v 为碰撞瞬间车速）

现场因素（全部运行时读取）：
  * 路线   <- SceneStaticData.route（53 m：北向直行 -> 右转 90 度 -> 东向）
  * 交通车 <- SimMsg.obstacles 实时列表（有一辆沿横路东行，约 10 m/s）
  * 弯道    <- 由路线曲率算，限速 v = sqrt(a_lat * R)

控制：
  * 横向 Stanley（航向 + 横向偏差）
  * 纵向：目标速度 = min(巡航, 弯道限速, 前方障碍限速)，用 P 控制油门/刹车
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

from speed.main import AdaptiveCruise  # noqa: E402  纵向：自标定恒定油门

KMH = 1.0 / 3.6


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def smooth_path(pts, k=2):
    out = []
    for i in range(len(pts)):
        lo, hi = max(0, i - k), min(len(pts) - 1, i + k)
        n = hi - lo + 1
        out.append((sum(pts[j][0] for j in range(lo, hi + 1)) / n,
                    sum(pts[j][1] for j in range(lo, hi + 1)) / n))
    return out


def radius_at(pts, i, step=3):
    a = pts[max(0, i - step)]
    b = pts[i]
    c = pts[min(len(pts) - 1, i + step)]
    A = math.dist(a, b)
    B = math.dist(b, c)
    C = math.dist(a, c)
    area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2.0
    if area2 < 1e-9:
        return 1e9
    return (A * B * C) / (4.0 * area2)


class TDriver:
    def __init__(self, route, k_heading=2.0, k_cross=2.2,
                 a_lat=1.2, v_cruise=2.0, look_speed_gain=1.4):
        self.pts = route
        self.k_h = k_heading
        self.k_c = k_cross
        self.a_lat = a_lat
        self.v_cruise = v_cruise
        self.look_gain = look_speed_gain
        self.idx = 0
        self._curv = [radius_at(route, i) for i in range(len(route))]

    # ---------- 横向 ----------
    def nearest(self, x, y):
        lo, hi = max(0, self.idx - 10), min(len(self.pts) - 1, self.idx + 40)
        best, bd = lo, 1e9
        for i in range(lo, hi + 1):
            d = math.hypot(self.pts[i][0] - x, self.pts[i][1] - y)
            if d < bd:
                best, bd = i, d
        if bd > 10.0:
            best = min(range(len(self.pts)),
                       key=lambda k: math.hypot(self.pts[k][0] - x, self.pts[k][1] - y))
        self.idx = max(self.idx, best)
        return self.idx

    def steering(self, x, y, yaw_deg, v):
        i = self.nearest(x, y)
        a, b = max(0, i - 3), min(len(self.pts) - 1, i + 3)
        tx = self.pts[b][0] - self.pts[a][0]
        ty = self.pts[b][1] - self.pts[a][1]
        norm = math.hypot(tx, ty) or 1.0
        tx, ty = tx / norm, ty / norm
        yaw_ref = -math.degrees(math.atan2(ty, tx))
        px, py = x - self.pts[i][0], y - self.pts[i][1]
        cross = tx * py - ty * px          # >0 车在左侧
        corr = math.degrees(math.atan2(self.k_c * cross, max(abs(v), 0.8)))
        desired = yaw_ref + corr
        err = math.radians(wrap_deg(desired - yaw_deg))
        return max(-1.0, min(1.0, self.k_h * err)), i, cross

    # ---------- 纵向 ----------
    def speed_target(self, i, x, y, yaw_deg, obstacles):
        # 弯道限速：向前看 10 m 取最小允许速度，提前刹车
        v = self.v_cruise
        acc = 0.0
        j = i
        while j < len(self._curv) - 1 and acc < 10.0:
            R = self._curv[j]
            vc = math.sqrt(self.a_lat * R) if R < 1e5 else self.v_cruise
            v = min(v, vc)
            acc += math.dist(self.pts[j], self.pts[j + 1])
            j += 1

        # 前方障碍：只看车头方向上、横向偏移小的
        for o in obstacles:
            if o.type not in (ObstacleType.CAR, ObstacleType.TRUCK, ObstacleType.BUS,
                              ObstacleType.DYNAMIC, ObstacleType.BICYCLE):
                continue
            dx, dy = o.pos_x - x, o.pos_y - y
            dist = math.hypot(dx, dy)
            if dist > 20.0:
                continue
            # 车头方向单位向量（场景 yaw 顺时针）
            hx = math.cos(math.radians(yaw_deg))
            hy = -math.sin(math.radians(yaw_deg))
            fwd = dx * hx + dy * hy          # 前方投影
            lat = abs(dx * hy - dy * hx)     # 横向偏移
            if fwd > 0 and lat < 2.6:
                v_allow = max(0.0, (fwd - 6.0) * 0.5)
                v = min(v, v_allow)
        return v

    @staticmethod
    def longitudinal(v, v_target):
        if v > v_target + 0.4:
            return 0.0, min(1.0, 0.5 + 0.3 * (v - v_target))
        if v > v_target:
            return 0.0, 0.0
        return 0.30, 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description="丁字路口通行")
    ap.add_argument("--seconds", type=float, default=190.0)
    ap.add_argument("--a-lat", type=float, default=1.2)
    args = ap.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    st = api.get_scene_static_data()
    route = [(float(p.x), float(p.y)) for p in st.route]
    print(f"路线点: {len(route)}  子场景: {[s.name for s in st.sub_scenes]}", flush=True)
    total = sum(math.dist(route[i], route[i + 1]) for i in range(len(route) - 1)) if len(route) > 1 else 0
    print(f"  起点 {route[0]} -> 终点 {route[-1]}，全长 {total:.1f} m", flush=True)

    # 拐角太急（实测半径 < 车的最小转弯半径），轻度平滑
    route = smooth_path(route, k=2)
    driver = TDriver(route, a_lat=args.a_lat)
    cruise = AdaptiveCruise(list(st.route), 60.0 * KMH, goal_ratio=0.92,
                             ramp_rate=0.06, plateau_eps=0.15, plateau_hold=0.6)
    prev_state = None

    frames = 0
    settle_until = None
    t0 = time.time()
    last_t = None
    v_peak = 0.0
    max_e = 0.0
    arrived = False
    arrived_t = None
    v_prev = 0.0
    spikes = []
    vc_last_thr = 0.0
    vc_last_brk = 0.0
    pwm_acc = 0.0
    stuck_t = 0.0
    vc_last_thr = 0.0
    vc_last_brk = 0.0
    pwm_acc = 0.0

    for msg, cams in api.main_loop():
        frames += 1
        car = msg.main_vehicle
        pose = msg.pose_gnss
        status = msg.scene_status
        dt = 1.0 / 30.0 if last_t is None else max(status.used_time - last_t, 1e-3)
        last_t = status.used_time

        # 逐帧加速度诊断：记录 |a| 最大的若干帧
        if frames > 3 and dt >= 0.015:
            a_inst = (car.speed - v_prev) / dt
            if abs(a_inst) > 4.0:
                spikes.append((round(status.used_time, 2), round(a_inst, 2),
                               round(car.speed, 2), round(vc_last_thr, 2), round(vc_last_brk, 2)))
        v_prev = car.speed

        steer, idx, e_ct = driver.steering(pose.pos_x, pose.pos_y, pose.ori_z, car.speed)
        max_e = max(max_e, abs(e_ct))

        end_pt = driver.pts[-1]
        dist_end = math.hypot(end_pt[0] - pose.pos_x, end_pt[1] - pose.pos_y)
        if dist_end < 1.5 or idx >= len(driver.pts) - 3:
            if arrived_t is None:
                arrived_t = time.time()
                print(f"== 接近终点（余 {dist_end:.2f} m），低速滚过终点线 ==", flush=True)
            arrived = True
            if time.time() - arrived_t < 3.0:
                api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=0.10,
                                                       brake=0.0, steering=steer))
                continue
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if time.time() - arrived_t > 25.0:
                break
            continue

        # 场景刚起来时先空转 1.2 s：变速箱在这个阶段给的加速度最猛，
        # 等它稳定再给油，才能把 |a| 压到 5 以内
        if settle_until is None:
            settle_until = time.time() + 1.2
        if time.time() < settle_until:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0,
                                                   steering=steer))
            continue

        v_target = driver.speed_target(idx, pose.pos_x, pose.pos_y, pose.ori_z, msg.obstacles)
        # 实测：只要踩刹车就会降档，降档瞬间 car.speed 会跳变 1~2 m/s
        # （诊断里出现过 ±19 m/s²，而当时油门是 0，物理上不可能）。
        # 所以这里全程用 PWM 维持低速、不踩刹车；拐角本身也只需要约 1.7 m/s。
        a_des = max(-1.0, min(1.2, 0.9 * (v_target - car.speed)))
        if a_des > 0:
            pwm_acc += min(1.0, a_des / 1.9)
            if pwm_acc >= 1.0:
                pwm_acc -= 1.0
                thr, brk = 0.12, 0.0
            else:
                thr, brk = 0.0, 0.0
        else:
            pwm_acc = 0.0
            thr, brk = 0.0, 0.0
        vc_last_thr, vc_last_brk = thr, brk
        api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                               brake=brk, steering=steer))
        v_peak = max(v_peak, abs(car.speed))

        if abs(car.speed) < 0.05 and thr > 0.05:
            stuck_t += dt
        else:
            stuck_t = 0.0
        if stuck_t > 15.0:
            print(f"!! 卡住 {stuck_t:.0f}s，退出", flush=True)
            break

        if frames % 15 == 1:
            print(f"[{status.used_time:6.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={car.speed:5.2f} v_t={v_target:5.2f} "
                  f"e={e_ct:+5.2f} str={steer:+5.2f} thr={thr:.2f} brk={brk:.2f} idx={idx}",
                  flush=True)

        if time.time() - t0 > args.seconds:
            print("到达脚本时长上限，退出", flush=True)
            break

    summary = {
        "frames": frames,
        "v_peak_kmh": round(v_peak * 3.6, 2),
        "max_cross_track_m": round(max_e, 2),
        "arrived": arrived,
        "route_len_m": round(total, 1),
        "accel_spikes": sorted(spikes, key=lambda x: -abs(x[1]))[:8],
        "elapsed": round(time.time() - t0, 1),
    }
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    (log_dir / f"tjunction_{time.strftime('%Y%m%d-%H%M%S')}.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
