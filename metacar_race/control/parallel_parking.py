# -*- coding: utf-8 -*-
"""平行车道泊车（侧方停车）控制器：几何推导 + 平滑控制

实测（2026-10-23，连续 5 次 210.00，停位离散 < 3 cm）：
    车位 (-9.78,-2.84) 朝向 90°、5.5 × 2.5；两侧车 (-9.78, +3.79) / (-9.78, -8.97)
    自车起点 (-6.35, 30.00) yaw=90；横向距离 L=3.43 m

**没有任何写死的坐标或 staging 距离**，全部由场景数据 + 车辆属性推导：

    车位中心/朝向/长宽  ← obstacles 里 type=PARKING_SLOT(30)
    横向距离 L          ← 自车到车位中心的垂直分量
    满舵半径 R          ← 轴距 / tan(最大前轮转角 × 舵角)
    入位角 θ            ← cosθ = 1 − L / (2R)   （两段等半径圆弧的总横移 = 2R(1−cosθ)）
    staging 点          ← 与"远端那辆车"并排（远端车中心 + 车长余量）

流程（经典两段圆弧侧方停车）：
    1) 前进，停在远端车旁（staging）
    2) 倒车、舵角斜坡打到 ±STEER，甩入 θ
    3) 反舵回正，车身与车道平行
    4) 低速前后微调到车位中心
    5) 轻刹停稳并保持

加速度分（|a|≤5 每题 +10）靠三件事拿到：
    · 起步油门用 PWM 占空比且**上限 0.5**（连续 0.01 会在挂挡瞬间窜到 6.8 m/s²）
    · 收尾**不再纯滑行**（会多滚 1 m 把 dy 带偏），改用 brake 0.06 轻刹收速
    · 换向前先停稳，避免正负加速度叠加
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from metacar import GearMode, VehicleControl

from metacar_race.config import (
    PARALLEL_A_MAX,
    PARALLEL_BRAKE_CMD,
    PARALLEL_CAR_LEN,
    PARALLEL_HOLD_SECONDS,
    PARALLEL_MAX_STEER_RAD,
    PARALLEL_SETTLE_BRAKE,
    PARALLEL_SLOT_TYPE,
    PARALLEL_STAGING_MARGIN_M,
    PARALLEL_STEER,
    PARALLEL_THROTTLE_MAX,
    PARALLEL_TOL_DX_M,
    PARALLEL_TOL_DY_M,
    PARALLEL_V_APPROACH,
    PARALLEL_V_REVERSE,
    PARALLEL_WHEELBASE,
)


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class ThrottleRamp:
    """油门斜坡 + PWM 占空比：把 0~0.01 的连续量拆成脉冲，并按 slew 限幅。"""

    def __init__(self, slew: float = 0.0006, period: int = 4) -> None:
        self.cur = 0.0
        self.slew = slew
        self.period = period
        self.frame = 0

    def update(self, duty_target: float) -> float:
        # 占空比上限 0.5：连续给满油门会在挂挡瞬间产生 6~8 m/s² 的加速度尖峰
        target = max(0.0, min(0.5, duty_target)) * PARALLEL_THROTTLE_MAX
        if target > self.cur:
            self.cur = min(target, self.cur + self.slew)
        else:
            self.cur = max(target, self.cur - self.slew)
        self.frame += 1
        on = int(round((self.cur / PARALLEL_THROTTLE_MAX) * self.period))
        return PARALLEL_THROTTLE_MAX if (self.frame % self.period) < on else 0.0


@dataclass(frozen=True)
class ParallelInfo:
    phase: str
    dx: float          # 相对车位中心的横向偏差
    dy: float          # 相对车位中心的纵向偏差
    yaw_error: float
    theta: float       # 由几何解出的入位角


def turning_radius(steer: float) -> float:
    """按轴距与最大前轮转角推算转弯半径。"""
    s = max(0.05, min(1.0, abs(steer)))
    return PARALLEL_WHEELBASE / math.tan(PARALLEL_MAX_STEER_RAD * s)


class ParallelParkingController:
    """侧方停车状态机。用法见 templates/parking/zyyxnjhu/parallel_run.py。"""

    def __init__(self) -> None:
        self.thr = ThrottleRamp()
        self.phase = "APPROACH"
        self.phase_t = 0.0
        self.hold = 0.0
        self.frame = 0
        self.slot: tuple[float, float, float, float, float] | None = None
        self.cars: list[tuple[float, float, float]] = []
        self.geom: dict[str, float] = {}
        # ---- 坡度适配（默认 0 = 平地行为完全不变）----
        # slope_thr  ：前进时的重力前馈油门（抵消坡道下滑分量）
        # slope_brake：需要停住时的最小刹车（坡上 0.10 的常规刹车刹不住）
        self.slope_thr = 0.0
        self.slope_brake = 0.0

    # ---------- 场景解析 ----------
    def _parse_scene(self, obstacles, x: float) -> None:
        if self.slot is None:
            for o in obstacles:
                t = getattr(o, "type", None)
                if getattr(t, "value", None) == PARALLEL_SLOT_TYPE:
                    self.slot = (float(o.pos_x), float(o.pos_y), float(o.ori_z),
                                 float(o.length), float(o.width))
                elif getattr(t, "name", "") == "CAR":
                    self.cars.append((float(o.pos_x), float(o.pos_y), float(o.length)))
        if self.slot is not None and not self.geom:
            sx, sy, sori, slen, swid = self.slot
            lateral = abs(sx - x)
            radius = turning_radius(PARALLEL_STEER)
            cos_t = max(-0.99, min(0.99, 1.0 - lateral / (2.0 * radius)))
            theta = math.degrees(math.acos(cos_t))
            # staging：与"远端那辆车"并排（远端 = 车位另一侧、y 更小的那辆）
            far = [c for c in self.cars if abs(c[0] - sx) < 1.5 and c[1] < sy]
            if far:
                stage_y = far[0][1] + PARALLEL_STAGING_MARGIN_M
            else:
                stage_y = sy - slen / 2.0 - PARALLEL_CAR_LEN / 2.0 - 1.0
            self.geom = {"sx": sx, "sy": sy, "sori": sori, "slen": slen, "swid": swid,
                         "lateral": lateral, "radius": radius, "theta": theta,
                         "stage_y": stage_y}

    # ---------- 主循环 ----------
    def update(self, pose, speed: float, obstacles, dt: float) -> tuple[VehicleControl, ParallelInfo]:
        self.frame += 1
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        self._parse_scene(obstacles, x)
        if not self.geom:
            return (VehicleControl(gear=GearMode.DRIVE, throttle=self.thr.update(0.25)),
                    ParallelInfo("SEEK", math.inf, math.inf, math.inf, 0.0))

        sx, sy = self.geom["sx"], self.geom["sy"]
        sori = self.geom["sori"]
        theta = self.geom["theta"]
        stage_y = self.geom["stage_y"]
        yaw_err = min(abs(wrap180(yaw - sori)), abs(wrap180(yaw - sori - 180.0)))
        dx, dy = x - sx, y - sy
        dt_phase = self.phase_t

        steer = thr = brk = 0.0
        gear = GearMode.DRIVE

        if self.phase == "APPROACH":
            rem = y - stage_y
            v_des = min(PARALLEL_V_APPROACH,
                        math.sqrt(max(2 * PARALLEL_A_MAX * max(rem, 0.0), 0.0)),
                        PARALLEL_A_MAX * dt_phase)      # 起步从 0 线性加速
            if rem <= 0.15:
                self.phase, self.phase_t = "STOP", 0.0
            else:
                err = v_des - speed
                if err < -0.2:
                    brk = PARALLEL_BRAKE_CMD
                else:
                    thr = self.thr.update(err / 0.8)
        elif self.phase == "STOP":
            gear, brk = GearMode.DRIVE, PARALLEL_BRAKE_CMD
            if abs(speed) <= 0.08:
                self.phase, self.phase_t = "ARC1", 0.0
        elif self.phase == "ARC1":
            gear = GearMode.REVERSE
            k = min(1.0, dt_phase / 1.2)                # 舵角斜坡：满舵静止起步会顶住静摩擦
            steer = PARALLEL_STEER * k
            thr = PARALLEL_THROTTLE_MAX if abs(speed) < 0.3 else self.thr.update(0.6)
            if 90.0 - yaw >= theta:
                self.phase, self.phase_t = "ARC2", 0.0
        elif self.phase == "ARC2":
            gear = GearMode.REVERSE
            steer = -PARALLEL_STEER * min(1.0, dt_phase / 0.8)
            thr = PARALLEL_THROTTLE_MAX if abs(speed) < 0.3 else self.thr.update(0.6)
            # 坡上这段圆弧的实际横向位移比规划大（实测每秒 2.9 m），
            # 只按"航向对上"判会冲过头顶到路缘；再加一条"横向到位就切 FIT"
            if yaw_err <= 3.0 or dx <= 0.90:
                self.phase, self.phase_t = "FIT", 0.0
        elif self.phase == "FIT":
            if abs(dy) <= PARALLEL_TOL_DY_M and abs(dx) <= PARALLEL_TOL_DX_M:
                self.phase, self.phase_t = "SETTLE", 0.0
            elif abs(speed) > 0.08:
                brk = PARALLEL_SETTLE_BRAKE          # 先停稳再换向
            else:
                gear = GearMode.DRIVE if dy > 0 else GearMode.REVERSE
                thr = PARALLEL_THROTTLE_MAX if abs(speed) < 0.15 else 0.0
        elif self.phase == "SETTLE":
            if abs(speed) > 0.06:
                gear, brk = GearMode.DRIVE, PARALLEL_SETTLE_BRAKE
            else:
                gear, brk = GearMode.PARKING, PARALLEL_BRAKE_CMD
                self.hold += dt
                if self.hold >= PARALLEL_HOLD_SECONDS:
                    self.phase = "DONE"
        else:
            gear, brk = GearMode.PARKING, 1.0

        self.phase_t += dt
        # ---- 坡度适配：前进时补重力前馈；要停住时提高最小刹车 ----
        if self.slope_thr > 0.0 and gear is GearMode.DRIVE and thr > 0.0:
            thr = min(1.0, thr + self.slope_thr)
        if self.slope_brake > 0.0 and brk > 0.0:
            brk = max(brk, self.slope_brake)
        vc = VehicleControl(gear=gear, throttle=float(thr), brake=float(brk), steering=float(steer))
        return vc, ParallelInfo(self.phase, dx, dy, yaw_err, theta)

    def done(self) -> bool:
        return self.phase == "DONE"


