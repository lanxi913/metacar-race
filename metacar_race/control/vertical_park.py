# -*- coding: utf-8 -*-
"""垂直泊车控制器（把 tools/park_v4.py 的已验证动作封装成类）

来源：`tools/park_v4.py`（元素场景「垂直车道泊车」实测 210 分）。
本文件只做"脚本 → 可组合控制器"的封装，**算法与判据保持不变**，
组合理由：连续场景的最后一个元素就是垂直泊车，直接复用已验证动作，
不在连续场景里重新调一套参数。

动作序列（与 park_v4 一致）：
    STAGE   车道内开到圆弧起点 (lane_x, slot_y − R + 补偿)
    ARC     倒挡满舵，直到车头与车位轴线平行
    STRAIGHT 倒挡直线补齐，边退边修航向
    HOLD    斜坡加压刹停保持 3 s

R（满舵倒车半径）来自 park_v4 的实测值 5.26 m，属"已标定的车辆参数"。
"""
from __future__ import annotations

import math

from metacar import GearMode, VehicleControl

from .geo_follower import A_LONG_MAX, V_RULE

PARKING_SLOT_TYPE = 30

ARC_RADIUS = 5.26          # m，park_v4 实测满舵倒车半径
ARC_END_Y_FIX = 0.30       # m，圆弧结束时的横向欠量补偿（park_v4 实测）
STAGE_SPEED = 0.14
REVERSE_SPEED = 0.55
PARK_DEPTH_TOL = 0.35
HEADING_TOL_DEG = 8.0
HOLD_SECONDS = 3.0
WHEELBASE_M = 2.6                        # m，乘用车标准轴距（与 geo_follower 兜底一致）

# 车控协议量程
STEER_FULL_SCALE = 0.6
WHEELBASE_M = 2.6                        # m，乘用车标准轴距（与 geo_follower 兜底一致）
MIN_TURN_RADIUS = WHEELBASE_M / math.tan(STEER_FULL_SCALE)   # m，满舵最小转弯半径


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class VerticalPark:
    """垂直泊车。`update()` 每帧调用，返回 (VehicleControl, phase)。"""

    def __init__(self):
        self.phase = "STAGE"
        self.slot = None          # (x, y, ori)
        self.stage = None
        self.hold_t = 0.0
        self.done = False
        self.t = 0.0
        self._lim_thr = 0.0
        self._lim_brk = 0.0

    # ---- 与 park_v4 相同的速度闭环（加了静止时的破静摩擦下限） ----
    @staticmethod
    def _speed_cmd(v: float, v_target: float):
        err = v_target - abs(v)
        if abs(err) < 0.06 and abs(v) > 0.05:
            return 0.0, 0.0
        if err > 0 or abs(v) < 0.05:
            # 满舵倒车静止时会顶死，给一个最小油门
            return min(0.10, max(0.05, 0.01 + 0.04 * max(err, 0.0))), 0.0
        return 0.0, min(0.10, -0.06 * err)

    def _smooth(self, vc: VehicleControl) -> VehicleControl:
        """油门限斜率（保证起步平滑）；刹车直通（低速进场必须刹得住）。"""
        self._lim_thr += max(-0.0015, min(0.0015, vc.throttle - self._lim_thr))
        self._lim_thr = max(0.0, min(1.0, self._lim_thr))
        self._lim_brk = max(0.0, min(1.0, vc.brake))
        vc.throttle, vc.brake = self._lim_thr, self._lim_brk
        return vc

    def update(self, pose, v: float, obstacles, dt: float):
        self.t += dt
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)

        if self.slot is None:
            for o in obstacles:
                if getattr(getattr(o, "type", None), "value", None) == PARKING_SLOT_TYPE:
                    self.slot = (float(o.pos_x), float(o.pos_y), float(o.ori_z))
                    # 起点以**车位**为基准（不取车当前 x）：连续场景里车可能在车位几百米外
                    self.stage = (self.slot[0], self.slot[1] - ARC_RADIUS + ARC_END_Y_FIX)
                    break
            if self.slot is None:
                return VehicleControl(gear=GearMode.PARKING, brake=1.0), "SEARCH"

        sx, sy, sori = self.slot
        d_center = math.hypot(sx - x, sy - y)
        dyaw = abs(math.degrees(_wrap_pi(math.radians(-yaw) - math.radians(-sori))))

        if self.phase == "STAGE":
            # 近距离（≤10 m）时直接"瞄准车位中心"开进去——连续场景里车位就在路线尽头，
            # 车位排是标准垂直车位，车头正对车位口即可进库
            if False and d_center <= 10.0:   # SEEK 模式实测会顶到邻车，停用
                if d_center <= 0.35 and abs(v) < 0.25:
                    self.phase = "HOLD"
                    self.hold_t = 0.0
                    return VehicleControl(gear=GearMode.PARKING, brake=0.25), "HOLD"
                ld = max(2.0 * MIN_TURN_RADIUS, min(d_center, 8.0))
                bearing = -math.degrees(math.atan2(sy - y, sx - x))
                alpha = _wrap_pi(math.radians(bearing - yaw))
                delta = math.atan2(2 * WHEELBASE_M * math.sin(alpha), ld)
                steer = max(-1.0, min(1.0, delta / STEER_FULL_SCALE))
                v_t = min(1.0, math.sqrt(max(2 * A_LONG_MAX * (d_center - 0.30), 0.0)))
                v_t = max(0.12, v_t)
                if v > v_t + 0.05:
                    vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, steering=steer,
                                        brake=min(0.20, max(0.03, (v - v_t) / 0.4)))
                else:
                    vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.06, steering=steer)
                return self._smooth(vc), "SEEK"
            dsx, dsy = self.stage[0] - x, self.stage[1] - y
            d_stage = math.hypot(dsx, dsy)
            # 到点且停稳 → 切倒车圆弧
            if d_stage < 0.45 and abs(v) < 0.30:
                self.phase = "ARC"
                return VehicleControl(gear=GearMode.PARKING, brake=0.20), "ARC"
            # 速度剖面：按规则允许的减速度反解；上限 2 m/s（泊车进场工况），下限 0.14
            v_stage = min(1.2, math.sqrt(max(2 * A_LONG_MAX * (d_stage - 0.30), 0.0)))
            v_stage = max(0.14, v_stage)
            # 纯跟踪到 stage 点：前视不小于 2 倍最小转弯半径，避免绕着点打转
            Ld = max(2.0 * MIN_TURN_RADIUS, min(d_stage, 8.0))
            bearing = -math.degrees(math.atan2(dsy, dsx))     # 仿真 yaw 顺时针
            alpha = _wrap_pi(math.radians(bearing - yaw))
            delta = math.atan2(2 * WHEELBASE_M * math.sin(alpha), Ld)
            steer = max(-1.0, min(1.0, delta / STEER_FULL_SCALE))
            if v > v_stage + 0.05:
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, steering=steer,
                                    brake=min(0.10, max(0.02, (v - v_stage) / 0.5)))
            else:
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.10, steering=steer)
            return self._smooth(vc), self.phase

        if self.phase == "ARC":
            if dyaw < HEADING_TOL_DEG:
                self.phase = "STRAIGHT"
                return VehicleControl(gear=GearMode.REVERSE, throttle=0.0, steering=0.0), "STRAIGHT"
            thr, brk = self._speed_cmd(v, REVERSE_SPEED)
            vc = VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=1.0)
            return self._smooth(vc), self.phase

        if self.phase == "STRAIGHT":
            dx, dy = x - sx, y - sy
            if abs(dx) < 0.06 or dx < -0.35:
                self.phase = "HOLD"
                self.hold_t = 0.0
                return VehicleControl(gear=GearMode.PARKING, brake=0.20), "HOLD"
            err = _wrap_pi(math.radians(-sori) - math.radians(-yaw))
            steer = max(-0.4, min(0.4, 1.2 * err))
            vt = max(0.15, min(0.80, 1.1 * max(abs(dx) - 0.10, 0.0) + 0.22))
            thr, brk = self._speed_cmd(v, vt)
            vc = VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=steer)
            return self._smooth(vc), self.phase

        # HOLD
        self.hold_t += dt
        ramp = min(1.0, 0.22 + 0.55 * self.hold_t)
        if self.hold_t > HOLD_SECONDS:
            self.done = True
        return VehicleControl(gear=GearMode.PARKING, brake=ramp), "HOLD"







