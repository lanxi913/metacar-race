# -*- coding: utf-8 -*-
"""垂直泊车控制（倒车入库，实测 210 分 / 误差 0.10 m）

适用：场景元素「垂直车道泊车」（车位与车道垂直，两侧有车）。

设计要点（都是 2026-10-23 实车标定踩出来的，改之前先看 docs/parking-calibration.md）：

1. 车位坐标**不写死**：从 ``SimCarMsg.obstacles`` 里 type=PARKING_SLOT(30) 的对象读
   ``pos_x / pos_y / ori_z / length / width``。
2. 入位用**倒车单圆弧**：后轴走半径 = 车道到车位中心横向距离 的 90° 圆弧，
   再直线倒到车位中心。前进一把进不去（转弯半径不够，车头顶车位边线）。
3. **staging 用位置反馈**，不用"刹车距离估算"：``v_des = sqrt(2·a·剩余距离)``。
   靠踩一脚刹车停车，落点会飘 ±1 m：早了倒不进，晚了圆弧扫到车位外。
4. **油门必须 PWM**：这台车的油门指令最小有效值就是 0.01，连续给会在挂挡瞬间
   窜到 7 m/s²（丢加速度分）；用 30 Hz 占空比等效出 0~0.005 的细油门。
5. **收尾必须滑行**：``brake=0.10`` 在 0.23 m/s 时会一帧把速度打到 0（6 m/s² 尖峰），
   所以先松油门滑到 0.06 m/s 以下再刹。
6. 目标：全程 |a| ≤ 5 m/s²（拿满 200 + 10 加速度分 = 210）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from metacar import GearMode, VehicleControl

from metacar_race.config import (
    PARKING_APPROACH_ACCEL,
    PARKING_APPROACH_MAX_MS,
    PARKING_ARC_STEER,
    PARKING_ARC_STEER_RAMP_S,
    PARKING_BACK_ACCEL,
    PARKING_BACK_STOP_GAP_M,
    PARKING_BRAKE_CMD,
    PARKING_PWM_LAUNCH_DUTY,
    PARKING_PWM_PERIOD,
    PARKING_REVERSE_MAX_MS,
    PARKING_SETTLE_HOLD_S,
    PARKING_SETTLE_SPEED_MS,
    PARKING_SLOT_TYPE,
    PARKING_STAGING_EXTRA_M,
    PARKING_STOP_GAP_M,
    PARKING_THROTTLE_MAX,
    PARKING_THROTTLE_START_MS,
    PARKING_YAW_TOL_DEG,
)


def wrap180(a: float) -> float:
    """把角度折到 (-180, 180]。"""
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def pwm_throttle(duty: float, frame: int, period: int = PARKING_PWM_PERIOD) -> float:
    """把 0~1 占空比变成 0.01/0 的脉冲，等效出 0~0.01 的连续油门。"""
    on = int(round(max(0.0, min(1.0, duty)) * period))
    return PARKING_THROTTLE_MAX if (frame % period) < on else 0.0


@dataclass(frozen=True)
class ParkingSlot:
    """车位（世界坐标，单位 m / 度）。"""

    x: float
    y: float
    yaw_deg: float
    length: float
    width: float


@dataclass(frozen=True)
class ParkingInfo:
    phase: str
    lateral_error: float     # 横向偏差（车位中心 - 自车，米）
    dist_to_center: float
    yaw_error: float


def find_slot(obstacles) -> ParkingSlot | None:
    """从障碍物里找车位。场景给的 type 是枚举，取 value == 30。"""
    for obj in obstacles:
        t = getattr(obj, "type", None)
        value = getattr(t, "value", t)
        if value == PARKING_SLOT_TYPE:
            return ParkingSlot(
                x=float(obj.pos_x),
                y=float(obj.pos_y),
                yaw_deg=float(obj.ori_z),
                length=float(getattr(obj, "length", 5.5)),
                width=float(getattr(obj, "width", 2.5)),
            )
    return None


class ParkingController:
    """垂直车位倒车入库状态机。

    用法（在 main_loop 里逐帧调用）::

        ctrl = ParkingController()
        vc, info = ctrl.update(pose, speed, obstacles, dt)
        api.set_vehicle_control(vc)
    """

    def __init__(self) -> None:
        self.phase = "APPROACH"
        self.slot: ParkingSlot | None = None
        self.staging_point: tuple[float, float] | None = None
        self.elapsed = 0.0
        self.arc_elapsed = 0.0
        self.hold_seconds = 0.0
        self.frame = 0
        # 以自车初始朝向作为"车道方向"：车位是垂直的，所以横向距离 = 投影差
        self.lane_yaw: float | None = None

    # ---- 内部工具 ----
    def _lateral(self, x: float, y: float) -> float:
        """自车到车位中心的**横向**距离（垂直于车道方向的分量）。"""
        assert self.slot is not None
        h = self._heading()
        dx, dy = self.slot.x - x, self.slot.y - y
        # 叉积模长 = 垂直分量
        return abs(dx * h[1] - dy * h[0])

    def _heading(self) -> tuple[float, float]:
        """车道方向单位向量：场景 yaw 顺时针为正，数学航向 θ = -yaw。"""
        yaw = self.lane_yaw if self.lane_yaw is not None else 90.0
        t = -math.radians(yaw)
        return math.cos(t), math.sin(t)

    def _yaw_error(self, yaw: float) -> float:
        """相对车位朝向的角度误差（车位 0 或 180 都算正）。"""
        assert self.slot is not None
        target = self.slot.yaw_deg
        return min(abs(wrap180(yaw - target)), abs(wrap180(yaw - target - 180.0)))

    def _ensure_slot(self, obstacles) -> bool:
        if self.slot is None:
            self.slot = find_slot(obstacles)
        return self.slot is not None

    # ---- 主循环 ----
    def update(self, pose, speed: float, obstacles, dt: float) -> tuple[VehicleControl, ParkingInfo]:
        self.frame += 1
        self.elapsed += dt
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)

        if not self._ensure_slot(obstacles):
            # 还没看到车位：低速直行，等障碍物数据
            return (
                VehicleControl(gear=GearMode.DRIVE, throttle=pwm_throttle(0.3, self.frame)),
                ParkingInfo("SEEK_SLOT", math.inf, math.inf, math.inf),
            )

        assert self.slot is not None
        if self.lane_yaw is None:
            self.lane_yaw = yaw
            # staging 点：从车位中心沿车道方向"往回"退 (横向距离 + 余量)，
            # 这样倒车走一个 90° 圆弧正好落到车位中心。
            h = self._heading()
            lateral = self._lateral(x, y)
            # 沿车道方向：从自车到车位的纵向距离 + (横向距离 + 余量)
            along = (self.slot.x - x) * h[0] + (self.slot.y - y) * h[1]
            d = along + lateral + PARKING_STAGING_EXTRA_M
            self.staging_point = (x + h[0] * d, y + h[1] * d)

        steer, thr, brk = 0.0, 0.0, 0.0
        gear = GearMode.DRIVE
        lateral = self._lateral(x, y)

        if self.phase == "APPROACH":
            assert self.staging_point is not None
            rem = math.hypot(self.staging_point[0] - x, self.staging_point[1] - y)
            v_des = min(PARKING_APPROACH_MAX_MS,
                        math.sqrt(max(2 * PARKING_APPROACH_ACCEL * max(rem, 0.0), 0.0)))
            if rem <= PARKING_STOP_GAP_M:
                self.phase = "STOP"
            else:
                err = v_des - speed
                if err < -0.2:
                    brk = PARKING_BRAKE_CMD
                else:
                    # 占空比上限 0.5：连续 0.01 会在挂挡瞬间窜到 7 m/s²
                    thr = pwm_throttle(min(err / 0.6, PARKING_PWM_LAUNCH_DUTY), self.frame)

        elif self.phase == "STOP":
            gear, brk = GearMode.DRIVE, PARKING_BRAKE_CMD
            if abs(speed) <= 0.10:
                self.phase = "ARC"
                self.arc_elapsed = 0.0

        elif self.phase == "ARC":
            gear = GearMode.REVERSE
            self.arc_elapsed += dt
            # 舵角斜坡：满舵 + 倒车起步会让车轮顶住静摩擦，v 直接掉到 0
            k = min(1.0, self.arc_elapsed / PARKING_ARC_STEER_RAMP_S)
            steer = PARKING_ARC_STEER * k
            if abs(speed) < PARKING_THROTTLE_START_MS:
                thr = PARKING_THROTTLE_MAX          # 先给连续油门破静摩擦
            elif abs(speed) < PARKING_REVERSE_MAX_MS:
                thr = pwm_throttle(0.5, self.frame)
            if self._yaw_error(yaw) <= PARKING_YAW_TOL_DEG:
                self.phase = "BACK"

        elif self.phase == "BACK":
            gear = GearMode.REVERSE
            rem = abs(x - self.slot.x)
            v_des = min(PARKING_REVERSE_MAX_MS,
                        math.sqrt(max(2 * PARKING_BACK_ACCEL * max(rem - PARKING_BACK_STOP_GAP_M, 0.0), 0.0)))
            if rem <= PARKING_BACK_STOP_GAP_M:
                self.phase = "SETTLE"
            else:
                err = v_des - abs(speed)
                if err < -0.15:
                    brk = PARKING_BRAKE_CMD
                elif abs(speed) < 0.25:
                    thr = PARKING_THROTTLE_MAX      # 破静摩擦，否则会卡住不动
                else:
                    thr = pwm_throttle(err / 0.4, self.frame)

        elif self.phase == "SETTLE":
            # 滑行到 0.06 m/s 以下再刹，避免 brake 一帧打停的 6 m/s² 尖峰
            if abs(speed) > PARKING_SETTLE_SPEED_MS:
                gear, brk = GearMode.DRIVE, 0.0
            else:
                gear, brk = GearMode.PARKING, PARKING_BRAKE_CMD
                self.hold_seconds += dt
            if self.hold_seconds >= PARKING_SETTLE_HOLD_S:
                self.phase = "DONE"

        else:  # DONE
            gear, brk = GearMode.PARKING, 1.0

        vc = VehicleControl(gear=gear, throttle=float(thr), brake=float(brk), steering=float(steer))
        return vc, ParkingInfo(self.phase, lateral, self._lateral(x, y), self._yaw_error(yaw))

    def done(self) -> bool:
        return self.phase == "DONE"


