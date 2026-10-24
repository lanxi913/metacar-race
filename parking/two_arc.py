# -*- coding: utf-8 -*-
"""标准侧方停车：两段定转角倒车（不用路径追踪）。

流程（全部在车道内完成，符合人手开车的直觉）：
  1) 前进沿车道开过车位，停在后车旁边
  2) 倒车 + 方向打向车位一侧，直到车头转过 phi
  3) 倒车 + 反打方向，直到车身回正
  4) 刹停保持

依据实测标定：
  前进 steering=+0.5 -> yaw +38.2 deg / 4s
  倒车 steering=+0.5 -> yaw -33.8 deg / 4s（方向相反）
  yaw 顺时针为正，车头朝 -y 时 yaw=90

几何：两段等角圆弧，横向位移 d = 2R(1-cos(phi))，纵向 = 2R sin(phi)。
"""
from __future__ import annotations

import math

from metacar import GearMode, VehicleControl

# 满打方向时的等效转弯半径（由标定反推：0.5 转向约 R=7.5 m，满舵按比例 ~3.8 m）
R_FULL = 3.9


class TwoArcParker:
    def __init__(self, slot_xy, lane_x, v_forward=1.2, v_reverse=0.7,
                 hold_seconds=6.0, steer=1.0):
        self.sx, self.sy = slot_xy
        self.goal_xy = (self.sx, self.sy)   # 供遥测打印
        self.lane_x = lane_x
        self.v_fwd = v_forward
        self.v_rev = v_reverse
        self.hold_seconds = hold_seconds
        self.steer_amp = steer

        # 由几何算起始点与转角
        d = abs(self.sx - lane_x)
        r = R_FULL
        if d >= 2 * r:
            d = 2 * r * 0.95
        self.phi = math.acos(max(-1.0, min(1.0, 1.0 - d / (2 * r))))
        self.longi = 2 * r * math.sin(self.phi)
        self.y_start = self.sy - self.longi          # 前进段的停车位置（车中心）
        self.yaw_mid = 90.0 - math.degrees(self.phi)  # 中间航向
        self.yaw_end = 90.0                           # 最终航向

        self.state = "forward"
        self.t = 0.0
        self.hold_t = 0.0
        self.stage_t = 0.0
        self._pwm = 0.0
        self.notes = []

    # ---------- 纵向：脉冲油门 ----------
    def _drive(self, v, v_target, dt):
        a_des = max(-1.5, min(1.5, 1.0 * (v_target - abs(v))))
        if a_des <= 0:
            self._pwm = 0.0
            return 0.0, min(1.0, -a_des / 1.5)
        self._pwm += min(1.0, a_des / 1.9)
        if self._pwm >= 1.0:
            self._pwm -= 1.0
            return 0.12, 0.0
        return 0.0, 0.0

    def params(self):
        return {
            "phi_deg": round(math.degrees(self.phi), 1),
            "longi_m": round(self.longi, 2),
            "y_start": round(self.y_start, 2),
            "yaw_mid": round(self.yaw_mid, 1),
        }

    def step(self, x, y, yaw_deg, v, dt):
        dt = max(dt, 1e-3)
        self.t += dt
        self.stage_t += dt

        if self.state == "forward":
            thr, brk = self._drive(v, self.v_fwd, dt)
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=0.0)
            if y <= self.y_start and abs(v) < 1.6:
                self.state = "arc1"
                self.stage_t = 0.0
                self.notes.append(f"forward done ({x:.2f},{y:.2f}) yaw={yaw_deg:.1f}")
                return VehicleControl(gear=GearMode.PARKING, brake=1.0, steering=0.0), self.state
            return vc, self.state

        if self.state == "arc1":
            # 倒车 + 正舵：yaw 减小（车头摆向 +x，车尾进车位）
            if yaw_deg <= self.yaw_mid:
                self.state = "arc2"
                self.stage_t = 0.0
                self.notes.append(f"arc1 done ({x:.2f},{y:.2f}) yaw={yaw_deg:.1f}")
                return VehicleControl(gear=GearMode.REVERSE, brake=1.0, steering=0.0), self.state
            if abs(v) < 0.05:
                thr, brk = self._drive(v, self.v_rev, dt)
                return VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk,
                                      steering=+self.steer_amp), self.state
            thr, brk = self._drive(v, self.v_rev, dt)
            return VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk,
                                  steering=+self.steer_amp), self.state

        if self.state == "arc2":
            if yaw_deg >= self.yaw_end - 1.0 or self.stage_t > 30.0:
                self.state = "hold"
                self.stage_t = 0.0
                err = math.hypot(self.sx - x, self.sy - y)
                self.notes.append(f"arc2 done ({x:.2f},{y:.2f}) yaw={yaw_deg:.1f} pos_err={err:.2f}")
                return VehicleControl(gear=GearMode.PARKING, brake=1.0, steering=0.0), self.state
            thr, brk = self._drive(v, self.v_rev * 0.85, dt)
            return VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk,
                                  steering=-self.steer_amp), self.state

        if self.state == "hold":
            self.hold_t += dt
            if self.hold_t >= self.hold_seconds:
                self.state = "done"
            return VehicleControl(gear=GearMode.PARKING, brake=1.0, steering=0.0), self.state

        return VehicleControl(gear=GearMode.PARKING, brake=1.0, steering=0.0), self.state
