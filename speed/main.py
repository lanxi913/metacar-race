# -*- coding: utf-8 -*-
r"""「限速行驶」入口（现场自标定版）。

设计原则：不写死经验常数，参数来自场景或在线自标定。
  * 限速值      <- SPEED_LIMIT_SIGN 障碍物的 extra_info("SpeedLimit|20")
  * 目标速度    <- 限速 × 系数（唯一保留的工程裕量，不是标定值）
  * 巡航油门    <- 在线自标定：缓慢加油门，等车速不再上升（低挡位平台）
                   就锁住当前油门；平台速度是实测的，不是写死的 0.45
  * 硬刹车阈值  <- 限速 × 0.97，只在真可能超速时才介入

为什么要"锁住油门"：直行/限速两题实测，恒定油门（不加闭环、不点刹）能拿
110 分（含 |a|≤5 的 10 分）；一旦闭环来回补油门/点刹，加速度抖动，只剩 100 分。

用法：
  1) .venv\Scripts\python.exe -u speed\main.py
  2) 客户端点「启动场景」（限速行驶）
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

KMH = 1.0 / 3.6


class AdaptiveCruise:
    """先自标定平台速度，再恒定油门巡航。

    状态：ramp（油门慢爬并测速）-> cruise（锁住油门不动）
    """

    def __init__(self, route, v_limit, lookahead=8.0, kp=1.2,
                 goal_ratio=0.92, brake_ratio=0.97,
                 ramp_rate=0.30, plateau_eps=0.06, plateau_hold=0.8):
        self.route = [(float(p.x), float(p.y)) for p in route]
        self.v_limit = float(v_limit)
        self.v_goal = self.v_limit * goal_ratio
        self.v_brake = self.v_limit * brake_ratio
        self.lookahead = float(lookahead)
        self.kp = float(kp)
        self.ramp_rate = float(ramp_rate)
        self.plateau_eps = float(plateau_eps)
        self.plateau_hold = float(plateau_hold)

        self.idx = 0
        self.thr = 0.0
        self.state = "ramp"
        self._v_hist = []
        self._flat_t = 0.0
        self.calibrated_thr = None
        self.calibrated_v = None

    # ---- 路径 ----
    def _nearest(self, x, y):
        lo = max(0, self.idx - 5)
        hi = min(len(self.route) - 1, self.idx + 30)
        best, bd = lo, 1e9
        for i in range(lo, hi + 1):
            px, py = self.route[i]
            d = math.hypot(px - x, py - y)
            if d < bd:
                best, bd = i, d
        self.idx = max(self.idx, best)
        return best

    def _target(self, x, y):
        i = self._nearest(x, y)
        for j in range(i, len(self.route)):
            px, py = self.route[j]
            if math.hypot(px - x, py - y) >= self.lookahead:
                return px, py
        return self.route[-1]

    def steering(self, x, y, yaw_deg):
        tx, ty = self._target(x, y)
        # yaw = -atan2(dy, dx)：不能写 atan2(-dy, -dx)，那只在 dx≈0 时相等，
        # 有横向分量时会反向（变道题实测会往反方向打方向）
        desired_yaw = -math.degrees(math.atan2(ty - y, tx - x))
        err = math.radians((desired_yaw - yaw_deg + 180.0) % 360.0 - 180.0)
        return max(-1.0, min(1.0, self.kp * err))

    # ---- 纵向 ----
    def step(self, x, y, yaw_deg, v, dt):
        dt = max(dt, 1e-3)
        steer = self.steering(x, y, yaw_deg)

        if v > self.v_brake:
            return VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.8,
                                  steering=steer), "brake"

        if self.state == "ramp":
            # 接近限速就停止加油，避免爬坡阶段顶过限速（超速是唯一重罚项）
            if v < 0.88 * self.v_limit:
                self.thr = min(1.0, self.thr + self.ramp_rate * dt)
            self._v_hist.append(v)
            if len(self._v_hist) > 30:
                self._v_hist.pop(0)
            # 平台检测：最近一秒速度基本不变，且已经动起来
            if len(self._v_hist) >= 12 and v > 1.0:
                span = max(self._v_hist) - min(self._v_hist)
                if span < self.plateau_eps:
                    self._flat_t += dt
                else:
                    self._flat_t = 0.0
            if self._flat_t >= self.plateau_hold:
                self.calibrated_thr = self.thr
                self.calibrated_v = v
                self.state = "cruise"
            return VehicleControl(gear=GearMode.DRIVE, throttle=self.thr, brake=0.0,
                                  steering=steer), "ramp"

        # 油门锁死不再动；但速度若明显高于自标定的平台值（可能触发升挡），
        # 立刻松油并轻刹，绝不让它冲过限速
        if self.calibrated_v is not None and v > self.calibrated_v + 0.6:
            return VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.5,
                                  steering=steer), "guard"
        return VehicleControl(gear=GearMode.DRIVE, throttle=self.calibrated_thr, brake=0.0,
                              steering=steer), "cruise"


def main() -> int:
    ap = argparse.ArgumentParser(description="限速行驶（现场自标定）")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--goal-ratio", type=float, default=0.92)
    ap.add_argument("--brake-ratio", type=float, default=0.97)
    args = ap.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    st = api.get_scene_static_data()
    route = list(st.route)
    print(f"路线点: {len(route)}  子场景: {[s.name for s in st.sub_scenes]}", flush=True)

    driver = None
    limit_kmh = None
    sign_pos = None
    frames = 0
    v_peak = 0.0
    t0 = time.time()
    last_t = None

    for msg, cams in api.main_loop():
        frames += 1
        car = msg.main_vehicle
        pose = msg.pose_gnss
        status = msg.scene_status

        if driver is None:
            signs = [o for o in msg.obstacles if o.type == ObstacleType.SPEED_LIMIT_SIGN]
            for s in signs:
                extra = s.extra_info or ""
                if "|" in extra:
                    try:
                        limit_kmh = float(extra.split("|")[-1])
                        sign_pos = (round(s.pos_x, 2), round(s.pos_y, 2))
                    except ValueError:
                        pass
            if limit_kmh is None:
                limit_kmh = 60.0
                print("!! 场景里没有 SPEED_LIMIT_SIGN，按场景元素默认 60 km/h 处理", flush=True)
            driver = AdaptiveCruise(route, limit_kmh * KMH, goal_ratio=args.goal_ratio)
            print(f"现场读取: 限速 {limit_kmh:.0f} km/h @ {sign_pos}", flush=True)
            print(f"  目标 {driver.v_goal * 3.6:.1f} km/h, 刹车阈值 {driver.v_brake * 3.6:.1f} km/h",
                  flush=True)

        dt = 1.0 / 30.0 if last_t is None else max(status.used_time - last_t, 1e-3)
        last_t = status.used_time

        prev_state = driver.state
        vc, mode = driver.step(pose.pos_x, pose.pos_y, pose.ori_z, car.speed, dt)
        api.set_vehicle_control(vc)
        v_peak = max(v_peak, abs(car.speed))

        if prev_state == "ramp" and driver.state == "cruise":
            print(f"== 自标定完成: 油门 {driver.calibrated_thr:.3f} -> "
                  f"平台速度 {driver.calibrated_v * 3.6:.1f} km/h ==", flush=True)

        if frames % 10 == 1:
            print(f"[{status.used_time:6.1f}s] {mode:6s} "
                  f"pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) yaw={pose.ori_z:6.1f} "
                  f"v={car.speed:5.2f}m/s({car.speed * 3.6:5.1f}km/h) "
                  f"thr={vc.throttle:5.3f} brk={vc.brake:4.2f}", flush=True)

        if time.time() - t0 > args.seconds:
            print("到达运行时长，退出。", flush=True)
            break

    summary = {
        "frames": frames,
        "limit_kmh": limit_kmh,
        "sign_pos": sign_pos,
        "v_peak_kmh": round(v_peak * 3.6, 2),
        "over_limit_kmh": round(max(0.0, v_peak * 3.6 - (limit_kmh or 0)), 2),
        "calibrated_thr": None if driver is None else driver.calibrated_thr,
        "calibrated_v_kmh": None if driver is None or driver.calibrated_v is None
                            else round(driver.calibrated_v * 3.6, 2),
        "elapsed": round(time.time() - t0, 1),
    }
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    (log_dir / f"speed_{time.strftime('%Y%m%d-%H%M%S')}.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
