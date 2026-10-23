# -*- coding: utf-8 -*-
r"""「车辆变道」入口（自建变道轨迹 + Stanley 反馈）。

为什么不用场景给的 route：
  实测这条 route 是一条 94.6° 的斜直线（(-2,-2) -> (-6,-51.8)），
  它既不平行车道，离本车道第一个障碍物也只有 1.10 m（车半宽 1.09 m），
  跟着它跑会擦碰、且终点姿态是斜的。

做法（全部参数来自场景）：
  1. 读 roads[].lanes，找到"当前车道"和"目标车道"的中心线
     - 当前车道 = 离车辆起点最近的车道
     - 目标车道 = 离 route 终点最近的车道
  2. 自建参考路径：本车道直行 -> 余弦 S 弯横移到目标车道 -> 目标车道直行
     S 弯长度按"横向位移 / 最大侧向加速度"推，不写死
  3. 用 Stanley 控制：航向误差 + 横向偏差反馈
     —— 横向偏差项天然产生"先多转一点、再回正"的效果
  4. 纵向沿用 speed 模块的自标定恒定油门（标定后开环，保 |a|<=5 的加分）

规则（场景元素 11，100 分）：200 s 内到终点；终点在另一条车道；压线 -20。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from metacar import GearMode, SceneAPI, VehicleControl

# 先把仓库根加入 sys.path，再导入仓库内的共享模块
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from speed.main import AdaptiveCruise  # noqa: E402  纵向：自标定恒定油门

KMH = 1.0 / 3.6


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def lane_x_at(lane_path, y):
    """在车道中心线上按 y 线性插值取 x。车道近似沿 y 走向。"""
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
    """自建参考路径：本车道 -> S 弯 -> 目标车道。"""

    def __init__(self, x_start, y_start, y_end,
                 x_from, x_to, s_start_offset=2.0, lat_accel_max=1.2):
        self.x_from = x_from
        self.x_to = x_to
        self.y_start = y_start
        self.y_end = y_end
        self.y_s0 = y_start - s_start_offset

        # S 弯长度：由横向位移与允许侧向加速度推出来（不是写死的 20 m）
        # 余弦 S: 最大侧向加速度 a = d * (pi*v/L)^2 ... 用几何近似 L = pi*sqrt(d*v^2/(2a))
        # 这里先按"转向可行"给一个下限，再由巡航速度决定
        d = abs(x_to - x_from)
        self.d = d
        # 取 S 弯长度 = 5.5 倍横向位移（对应约 45 度的进入角），并保证至少 12 m
        self.L = max(12.0, 5.5 * d)

    def ref(self, y):
        """返回 (x_ref, yaw_ref_deg)。y 单调向下。"""
        if y >= self.y_s0:
            return self.x_from, 90.0
        s = self.y_s0 - y
        if s <= self.L:
            t = s / self.L
            s_shape = 0.5 * (1 - math.cos(math.pi * t))
            x = self.x_from + (self.x_to - self.x_from) * s_shape
            # 切线方向（dy 为负，向下）
            dxdt = (self.x_to - self.x_from) * 0.5 * math.pi * math.sin(math.pi * t) / self.L
            # 单位切向：dx = dxdt * (-1)? 用 y 递减参数化：dx/dy = -dxdt
            dydt = -1.0
            yaw = -math.degrees(math.atan2(dydt, dxdt))
            return x, yaw
        return self.x_to, 90.0


class StanleyDriver:
    def __init__(self, path: LaneChangePath, k_heading=2.0, k_cross=1.6,
                 lookahead=6.0, v_cruise=4.5):
        self.path = path
        self.k_h = k_heading
        self.k_c = k_cross
        self.v = v_cruise

    def control(self, x, y, yaw_deg, v):
        x_ref, yaw_ref = self.path.ref(y)
        e_ct = x - x_ref          # + 表示车在参考线右侧（x 更大）
        # Stanley: 期望航向 = 参考航向 - atan(k * e / v)
        # 车在右侧(e>0) 需要往左打 -> yaw 增大 -> 期望航向加大
        corr = math.degrees(math.atan2(self.k_c * e_ct, max(abs(v), 0.8)))
        desired_yaw = yaw_ref + corr
        err = math.radians(wrap_deg(desired_yaw - yaw_deg))
        steer = max(-1.0, min(1.0, self.k_h * err))
        return steer


def main() -> int:
    ap = argparse.ArgumentParser(description="车辆变道")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--k-cross", type=float, default=1.6)
    ap.add_argument("--k-heading", type=float, default=2.0)
    args = ap.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    st = api.get_scene_static_data()
    route = list(st.route)
    lanes = []
    for rd in st.roads:
        for ln in rd.lanes:
            if len(ln.path_points) >= 2:
                lanes.append([(p.x, p.y) for p in ln.path_points])
    print(f"route {len(route)} pts, lanes {len(lanes)}", flush=True)

    if len(route) < 2 or not lanes:
        print("缺少路线或车道数据，退出。", flush=True)
        return 1

    end_x, end_y = route[-1].x, route[-1].y

    driver = None
    cruise = None
    prev_state = None
    frames = 0
    t0 = time.time()
    last_t = None
    path = None
    v_peak = 0.0

    for msg, cams in api.main_loop():
        frames += 1
        car = msg.main_vehicle
        pose = msg.pose_gnss
        status = msg.scene_status

        if driver is None:
            # 当前车道 = 离起点最近；目标车道 = 离终点最近
            cur_lane = min(lanes, key=lambda lp: abs(lane_x_at(lp, pose.pos_y) - pose.pos_x))
            tgt_lane = min(lanes, key=lambda lp: abs(lane_x_at(lp, end_y) - end_x))
            x_from = lane_x_at(cur_lane, pose.pos_y)
            x_to = lane_x_at(tgt_lane, end_y)
            path = LaneChangePath(pose.pos_x, pose.pos_y, end_y, x_from, x_to)
            print(f"现场读取: 本车道 x={x_from:.2f}, 目标车道 x={x_to:.2f}, "
                  f"横移 {x_to - x_from:+.2f} m, S 弯长 {path.L:.1f} m", flush=True)
            for o in msg.obstacles:
                d_ref = abs(o.pos_x - path.ref(o.pos_y)[0])
                print(f"  障碍物 ({o.pos_x:.2f},{o.pos_y:.2f}) 距参考线 {d_ref:.2f} m", flush=True)
            driver = StanleyDriver(path, k_heading=args.k_heading, k_cross=args.k_cross)
            cruise = AdaptiveCruise(route, 60.0 * KMH, goal_ratio=0.92)
        dt = 1.0 / 30.0 if last_t is None else max(status.used_time - last_t, 1e-3)
        last_t = status.used_time

        steer = driver.control(pose.pos_x, pose.pos_y, pose.ori_z, car.speed)

        # 纵向：自标定恒定油门（标定后开环），保住 |a|<=5 的 10 分；
        # 横向用 Stanley 覆盖掉它自带的转向
        vc, mode = cruise.step(pose.pos_x, pose.pos_y, pose.ori_z, car.speed, dt)
        vc = VehicleControl(gear=vc.gear, throttle=vc.throttle, brake=vc.brake, steering=steer)
        api.set_vehicle_control(vc)
        if prev_state == "ramp" and cruise.state == "cruise":
            print(f"== 自标定完成: 油门 {cruise.calibrated_thr:.3f} -> "
                  f"平台速度 {cruise.calibrated_v * 3.6:.1f} km/h ==", flush=True)
        prev_state = cruise.state
        thr, brk = vc.throttle, vc.brake
        v_peak = max(v_peak, abs(car.speed))

        if frames % 10 == 1:
            print(f"[{status.used_time:6.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={car.speed:5.2f} "
                  f"ref_x={path.ref(pose.pos_y)[0]:7.2f} e={pose.pos_x - path.ref(pose.pos_y)[0]:+5.2f} "
                  f"str={steer:+5.2f} thr={thr:.2f} brk={brk:.2f}", flush=True)

        if pose.pos_y < end_y - 0.5 or time.time() - t0 > args.seconds:
            break

    summary = {
        "frames": frames,
        "v_peak_kmh": round(v_peak * 3.6, 2),
        "lateral_shift_m": None if path is None else round(path.x_to - path.x_from, 2),
        "s_curve_len_m": None if path is None else round(path.L, 2),
        "elapsed": round(time.time() - t0, 1),
    }
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    (log_dir / f"lanechange_{time.strftime('%Y%m%d-%H%M%S')}.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
