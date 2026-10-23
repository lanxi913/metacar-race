# -*- coding: utf-8 -*-
"""平行泊车控制器：车道行驶 → 循迹入位 → 刹停保持。

这个场景的油门几乎是开关式的（实测给 0.05 也会一路加速到 4.78 m/s 的低挡位平台速度），
所以低速蠕行必须用 PWM：按占空比每 N 帧给一帧油门，其余帧滑行。
单帧油门约带来 0.1 m/s 的速度增量，冲击约 3.6 m/s²，仍在 |a| ≤ 5 以内。

转向符号（实测）：steering < 0 时 theta 增大（数学角逆时针）。
    theta_dot = -(v / L) * tan(steering * MAX_STEER)
前进纠偏 steering = -kp * err_theta；倒车时符号相反。
"""
from __future__ import annotations

import math

from metacar import GearMode, VehicleControl

WHEELBASE = 2.6
MAX_STEER = 0.6


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class PathFollower:
    """沿位姿序列走，支持前进/倒车混排。

    方向必须"锁存"：相邻点的 d 会抖动，若每帧按最近点的 d 切挡，挡位会在
    DRIVE/REVERSE 之间反复横跳，控制器一直踩刹车，车就原地不动了。
    """

    def __init__(self, path, lookahead=1.0):
        self.path = path
        self.lookahead = lookahead
        self.cum = [0.0]
        for i in range(1, len(path)):
            self.cum.append(self.cum[-1] + math.hypot(path[i].x - path[i - 1].x,
                                                      path[i].y - path[i - 1].y))
        self.total = self.cum[-1]
        self.idx = 0
        # 把路径切成若干"同向段"
        self.segments = []
        start = 0
        for i in range(1, len(path)):
            if (path[i].d > 0) != (path[start].d > 0):
                self.segments.append((start, i - 1, path[start].d))
                start = i
        self.segments.append((start, len(path) - 1, path[start].d))
        self.seg = 0

    def _advance_segment(self):
        while self.seg < len(self.segments) - 1 and self.idx >= self.segments[self.seg][1]:
            self.seg += 1

    def _nearest(self, x, y):
        lo = max(0, self.idx - 6)
        hi = min(len(self.path) - 1, self.idx + 20)
        best, bd = lo, 1e9
        for i in range(lo, hi + 1):
            p = self.path[i]
            d = math.hypot(p.x - x, p.y - y)
            if d < bd:
                best, bd = i, d
        self.idx = max(self.idx, best)
        return self.idx

    def _point_at(self, s):
        s = max(0.0, min(self.total, s))
        for i in range(1, len(self.cum)):
            if self.cum[i] >= s:
                s0, s1 = self.cum[i - 1], self.cum[i]
                t = 0.0 if s1 - s0 < 1e-9 else (s - s0) / (s1 - s0)
                a, b = self.path[i - 1], self.path[i]
                return a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.d
        p = self.path[-1]
        return p.x, p.y, p.d

    def target(self, x, y):
        i = self._nearest(x, y)
        self._advance_segment()
        s0, s1, d = self.segments[self.seg]
        s = self.cum[i] + self.lookahead * (1.0 if d > 0 else -1.0)
        tx, ty, dd = self._point_at(s)
        return tx, ty, d

    def finished(self, x, y, yaw_deg, goal_theta, pos_tol=0.35, yaw_tol_deg=12.0):
        p = self.path[-1]
        pos_ok = math.hypot(p.x - x, p.y - y) < pos_tol
        yaw_ok = abs(wrap(goal_theta - math.radians(-yaw_deg))) < math.radians(yaw_tol_deg)
        return pos_ok and yaw_ok


class ParkingDriver:
    def __init__(self, path, lane_x, handoff_y, goal_xy, goal_theta,
                 lane_speed=1.8, path_speed=0.6, hold_seconds=6.0, kp_heading=2.2):
        self.follower = PathFollower(path)
        self.lane_x = lane_x
        self.handoff_y = handoff_y
        self.goal_xy = goal_xy
        self.goal_theta = goal_theta
        self.lane_speed = lane_speed
        self.path_speed = path_speed
        self.hold_seconds = hold_seconds
        self.kp = kp_heading
        self.state = "lane"
        self.hold_t = 0.0
        self.state_t = 0.0
        self.gear = GearMode.DRIVE
        self.gear_wait = 0.0
        self._pwm = 0.0
        self.notes = []

    # ---------- 纵向：PWM 油门 + 刹车 ----------
    def _longitudinal(self, v, v_target, dt):
        """返回 (throttle, brake)。低速靠脉冲，刹车负责收敛。"""
        err = v_target - v
        # 实测：满占空比约 1.9 m/s² 加速，刹车满量约 1.5 m/s² 减速
        a_des = max(-1.5, min(1.5, 0.9 * err))
        if a_des >= 0.0:
            duty = min(1.0, a_des / 1.9)
            self._pwm += duty
            if self._pwm >= 1.0:
                self._pwm -= 1.0
                return 0.12, 0.0
            return 0.0, 0.0
        self._pwm = 0.0
        return 0.0, min(1.0, -a_des / 1.5)

    def _command(self, x, y, yaw_deg, v, dt, v_target, want_reverse):
        vc = VehicleControl(steering=0.0)
        want_gear = GearMode.REVERSE if want_reverse else GearMode.DRIVE

        # 换挡前先停稳，避免挡位冲击
        if want_gear != self.gear:
            if abs(v) > 0.15:
                return VehicleControl(gear=self.gear, throttle=0.0, brake=0.6, steering=0.0)
            self.gear = want_gear
            self.gear_wait = 0.35
            return VehicleControl(gear=self.gear, brake=1.0, steering=0.0)
        if self.gear_wait > 0.0:
            self.gear_wait -= dt
            return VehicleControl(gear=self.gear, brake=0.25, steering=0.0)

        thrott, brake = self._longitudinal(v, v_target, dt)
        return VehicleControl(gear=self.gear, throttle=thrott, brake=brake, steering=vc.steering)

    # ---------- 主循环 ----------
    def step(self, x, y, yaw_deg, v, dt):
        dt = max(dt, 1e-3)
        self.state_t += dt
        theta = math.radians(-yaw_deg)

        if self.state == "lane":
            err_y = self.lane_x - x
            steer = max(-1.0, min(1.0, -1.5 * err_y))
            dist = y - self.handoff_y
            v_t = self.lane_speed if dist > 8.0 else max(0.35, 0.22 * dist)
            if dist <= 0.35 or self.state_t > 45.0:
                self.state = "path"
                self.state_t = 0.0
                self.notes.append(f"handoff ({x:.2f},{y:.2f}) v={v:.2f}")
                return VehicleControl(gear=GearMode.PARKING, brake=1.0, steering=0.0), self.state
            base = self._command(x, y, yaw_deg, v, dt, v_t, want_reverse=False)
            base.steering = steer
            return base, self.state

        if self.state == "path":
            tx, ty, d = self.follower.target(x, y)
            if d > 0:
                desired = math.atan2(ty - y, tx - x)
                err = wrap(desired - theta)
                steer = max(-1.0, min(1.0, -self.kp * err))
            else:
                desired = math.atan2(y - ty, x - tx)
                err = wrap(desired - theta)
                steer = max(-1.0, min(1.0, self.kp * err))

            gx, gy = self.goal_xy
            near_goal = math.hypot(gx - x, gy - y) < 1.2
            v_t = self.path_speed * (0.6 if near_goal else 1.0)
            base = self._command(x, y, yaw_deg, v, dt, v_t, want_reverse=(d < 0))
            base.steering = steer

            if self.follower.finished(x, y, yaw_deg, self.goal_theta) or self.state_t > 110.0:
                self.state = "hold"
                self.state_t = 0.0
                self.notes.append(
                    f"path done ({x:.2f},{y:.2f}) yaw={yaw_deg:.1f} "
                    f"pos_err={math.hypot(gx - x, gy - y):.2f}m "
                    f"yaw_err={math.degrees(wrap(self.goal_theta - theta)):.1f}deg"
                )
            return base, self.state

        if self.state == "hold":
            self.hold_t += dt
            if self.hold_t >= self.hold_seconds:
                self.state = "done"
            return VehicleControl(gear=GearMode.PARKING, brake=1.0, steering=0.0), self.state

        return VehicleControl(gear=GearMode.PARKING, brake=1.0, steering=0.0), self.state
