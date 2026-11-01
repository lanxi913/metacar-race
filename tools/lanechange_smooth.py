# -*- coding: utf-8 -*-
r"""「车辆变道」稳压版（不改 lanechange/main.py，另开入口）。

与 lanechange/main.py 的差别只有纵向：
  * 原版用 AdaptiveCruise 自标定，实测爬到 thr=0.81 → 升挡（峰值 22.65 km/h）→ 加速度违规，100 分；
  * 这里改成**恒定油门 0.30 封顶**（1 挡平台 4.78 m/s），全程不升挡；
  * 终点前按 v = sqrt(2*0.8*d) 缓减，刹车上限 0.10。

横向沿用原版思路：本车道 → 余弦 S 弯 → 目标车道 + Stanley。

用法：先跑脚本，再在客户端点「车辆变道」。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

KMH = 1.0 / 3.6
THR_CAP = 0.30
BRAKE_CAP = 0.10


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def lane_x_at(lane_path, y):
    pts = sorted(lane_path, key=lambda p: p[1])
    if y <= pts[0][1]:
        return pts[0][0]
    if y >= pts[-1][1]:
        return pts[-1][0]
    for i in range(1, len(pts)):
        y0, y1 = pts[i - 1][1], pts[i][1]
        if y0 <= y <= y1 and y1 - y0 > 1e-6:
            t = (y - y0) / (y1 - y0)
            return pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0])
    return pts[-1][0]


class LaneChangePath:
    def __init__(self, y_start, y_end, x_from, x_to, s_start_offset=2.0):
        self.x_from, self.x_to = x_from, x_to
        self.y_start, self.y_end = y_start, y_end
        self.y_s0 = y_start - s_start_offset
        d = abs(x_to - x_from)
        self.L = max(12.0, 5.5 * d)

    def ref(self, y):
        if y >= self.y_s0:
            return self.x_from, 90.0
        s = self.y_s0 - y
        if s <= self.L:
            t = s / self.L
            x = self.x_from + (self.x_to - self.x_from) * 0.5 * (1 - math.cos(math.pi * t))
            dxdy = -((self.x_to - self.x_from) * 0.5 * math.pi * math.sin(math.pi * t) / self.L)
            yaw = -math.degrees(math.atan2(-1.0, dxdy))
            return x, yaw
        return self.x_to, 90.0


def main() -> int:
    ap = argparse.ArgumentParser(description="车辆变道（稳压版）")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--k-cross", type=float, default=1.6)
    ap.add_argument("--k-heading", type=float, default=2.0)
    args = ap.parse_args()

    print("waiting for scene ...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    route = list(static.route)
    lanes = []
    for rd in static.roads:
        for ln in rd.lanes:
            if len(ln.path_points) >= 2:
                lanes.append([(float(p.x), float(p.y)) for p in ln.path_points])
    print(f"route {len(route)} pts, lanes {len(lanes)}", flush=True)
    if len(route) < 2 or not lanes:
        print("no route/lanes", flush=True)
        return 1

    end_x, end_y = float(route[-1].x), float(route[-1].y)
    path = None
    driver_ready = False
    t0 = time.time()
    last_t = None
    frames = 0
    v_peak = 0.0
    thr_cur = 0.0

    try:
        for msg, _ in api.main_loop():
            frames += 1
            car = msg.main_vehicle
            pose = msg.pose_gnss
            st = msg.scene_status
            x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
            speed = float(car.speed)
            dt = 1 / 30 if last_t is None else max(st.used_time - last_t, 1e-3)
            last_t = st.used_time
            v_peak = max(v_peak, abs(speed))

            if not driver_ready:
                cur = min(lanes, key=lambda lp: abs(lane_x_at(lp, y) - x))
                tgt = min(lanes, key=lambda lp: abs(lane_x_at(lp, end_y) - end_x))
                path = LaneChangePath(y, end_y, lane_x_at(cur, y), lane_x_at(tgt, end_y))
                driver_ready = True
                print(f"lane x_from={path.x_from:.2f} x_to={path.x_to:.2f} shift={path.x_to - path.x_from:+.2f} "
                      f"S={path.L:.1f}m", flush=True)

            x_ref, yaw_ref = path.ref(y)
            e = x - x_ref
            corr = math.degrees(math.atan2(args.k_cross * e, max(abs(speed), 0.8)))
            desired = yaw_ref + corr
            steer = max(-1.0, min(1.0, args.k_heading * math.radians(wrap_deg(desired - yaw))))

            rem_y = y - end_y
            thr_t = THR_CAP
            brk_t = 0.0
            # 起步压一下：先驻车 1.5 s，车一动就滑行，压掉挂挡首帧的速度跳变
            if st.used_time < 1.5:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.10))
                continue
            if st.used_time < 3.0 and abs(speed) > 0.15:
                api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, steering=float(steer)))
                continue
            if rem_y < 12.0:
                v_allow = max(0.6, math.sqrt(2.0 * 0.8 * max(rem_y, 0.0)))
                if speed > v_allow + 0.2:
                    thr_t, brk_t = 0.0, min(BRAKE_CAP, 0.02 + 0.08 * (speed - v_allow))
                elif speed > v_allow:
                    thr_t = 0.0
            if speed > 4.80:
                thr_t = 0.0
            thr_cur = min(thr_t, thr_cur + 0.010) if thr_t > thr_cur else max(thr_t, thr_cur - 0.05)

            api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=float(thr_cur),
                                                   brake=float(brk_t), steering=float(steer)))

            if frames % 10 == 1:
                print(f"[{st.used_time:6.1f}s] pos=({x:7.2f},{y:7.2f}) yaw={yaw:6.1f} v={speed:5.2f} "
                      f"ref_x={x_ref:7.2f} e={e:+5.2f} str={steer:+5.2f} thr={thr_cur:4.2f} brk={brk_t:4.2f}", flush=True)

            if rem_y < -0.5 or st.used_time > args.seconds:
                break
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:
            pass

    print("SUMMARY " + json.dumps({
        "frames": frames,
        "v_peak_kmh": round(v_peak * 3.6, 2),
        "elapsed": round(time.time() - t0, 1),
    }, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
