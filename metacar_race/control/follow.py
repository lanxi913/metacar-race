# -*- coding: utf-8 -*-
"""车辆跟随的前车选择与纵向控制。"""
from __future__ import annotations

import math
from dataclasses import dataclass

from metacar import ObstacleType

from metacar_race.config import (
    FOLLOW_BRAKE_SLEW_PER_FRAME,
    FOLLOW_CANDIDATE_MAX_HEADING_DEG,
    FOLLOW_CANDIDATE_MAX_LATERAL_M,
    FOLLOW_END_DECEL_MS2,
    FOLLOW_END_TOL_M,
    FOLLOW_GAP_GAIN,
    FOLLOW_HEADWAY_SECONDS,
    FOLLOW_LEADER_LOST_SPEED_MS,
    FOLLOW_MAX_SPEED_MS,
    FOLLOW_MIN_GAP_M,
    FOLLOW_SAFE_DECEL_MS2,
    FOLLOW_SPEED_DEADBAND_MS,
    FOLLOW_STANDSTILL_GAP_M,
    FOLLOW_THROTTLE_MAX,
    FOLLOW_THROTTLE_SLEW_PER_FRAME,
)


VEHICLE_TYPES = {
    ObstacleType.CAR,
    ObstacleType.TRUCK,
    ObstacleType.BUS,
    ObstacleType.SPECIAL_VEHICLE,
    ObstacleType.DYNAMIC,
}


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class LeaderInfo:
    obstacle_id: int
    obstacle_type: str
    forward_m: float
    lateral_m: float
    gap_m: float
    speed_ms: float
    heading_diff_deg: float


def select_leader(obstacles, pose, ego_length: float) -> LeaderInfo | None:
    """选择当前车道中净距最近的前方车辆。"""
    heading = -math.radians(float(pose.ori_z))
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    candidates = []
    for obstacle in obstacles:
        if obstacle.type not in VEHICLE_TYPES:
            continue

        dx = float(obstacle.pos_x) - float(pose.pos_x)
        dy = float(obstacle.pos_y) - float(pose.pos_y)
        forward = dx * cos_h + dy * sin_h
        lateral = -dx * sin_h + dy * cos_h
        heading_diff = wrap_deg(
            float(obstacle.ori_z) - float(pose.ori_z)
        )
        if forward <= 0.0:
            continue
        if abs(lateral) > FOLLOW_CANDIDATE_MAX_LATERAL_M:
            continue
        if abs(heading_diff) > FOLLOW_CANDIDATE_MAX_HEADING_DEG:
            continue

        radians = math.radians(heading_diff)
        lead_half_extent = (
            abs(math.cos(radians)) * float(obstacle.length) * 0.5
            + abs(math.sin(radians)) * float(obstacle.width) * 0.5
        )
        gap = forward - float(ego_length) * 0.5 - lead_half_extent
        candidates.append(
            LeaderInfo(
                obstacle_id=int(obstacle.id),
                obstacle_type=str(
                    getattr(obstacle.type, "name", obstacle.type)
                ),
                forward_m=forward,
                lateral_m=lateral,
                gap_m=gap,
                speed_ms=max(0.0, float(obstacle.vel_x)),
                heading_diff_deg=heading_diff,
            )
        )

    if not candidates:
        return None
    return min(candidates, key=lambda candidate: candidate.gap_m)


def compute_target_speed(
    leader: LeaderInfo | None,
    remaining_to_end: float | None,
) -> tuple[float, float, float]:
    """返回 (目标速度, 期望净距, 安全速度上限)。"""
    if leader is None:
        target = FOLLOW_LEADER_LOST_SPEED_MS
        desired_gap = FOLLOW_STANDSTILL_GAP_M
        safe_speed = target
    else:
        desired_gap = (
            FOLLOW_STANDSTILL_GAP_M
            + FOLLOW_HEADWAY_SECONDS * leader.speed_ms
        )
        gap_error = leader.gap_m - desired_gap
        gap_speed = leader.speed_ms + FOLLOW_GAP_GAIN * gap_error
        safe_speed = math.sqrt(
            2.0
            * FOLLOW_SAFE_DECEL_MS2
            * max(0.0, leader.gap_m - FOLLOW_MIN_GAP_M)
        )
        target = min(
            FOLLOW_MAX_SPEED_MS,
            max(0.0, gap_speed),
            safe_speed,
        )
        if leader.gap_m <= FOLLOW_MIN_GAP_M:
            target = 0.0

    if remaining_to_end is not None:
        end_speed = math.sqrt(
            2.0
            * FOLLOW_END_DECEL_MS2
            * max(0.0, remaining_to_end - FOLLOW_END_TOL_M)
        )
        target = min(target, end_speed)

    return target, desired_gap, safe_speed


@dataclass
class LongitudinalController:
    """带斜率限制和加速度反馈的纵向执行器。"""

    filtered_speed: float = 0.0
    acceleration: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    stopped: bool = False

    def reset(self):
        self.filtered_speed = 0.0
        self.acceleration = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.stopped = False

    def update(
        self,
        speed: float,
        target_speed: float,
        gap_m: float | None,
        dt: float,
    ) -> tuple[float, float]:
        dt = max(dt, 1.0 / 60.0)
        raw_speed = abs(float(speed))
        error = target_speed - raw_speed
        raw_accel = (raw_speed - self.filtered_speed) / dt
        self.filtered_speed += 0.20 * (raw_speed - self.filtered_speed)
        self.acceleration += 0.10 * (
            clamp(raw_accel, -10.0, 10.0) - self.acceleration
        )

        if gap_m is not None and gap_m <= FOLLOW_MIN_GAP_M:
            throttle_target = 0.0
            brake_target = 0.20
        elif error > FOLLOW_SPEED_DEADBAND_MS:
            throttle_target = min(
                FOLLOW_THROTTLE_MAX,
                0.04 + 0.06 * error,
            )
            brake_target = 0.0
        elif error < -FOLLOW_SPEED_DEADBAND_MS:
            throttle_target = 0.0
            brake_target = min(0.15, 0.03 + 0.04 * (-error))
        else:
            throttle_target = 0.0
            brake_target = 0.0

        if self.acceleration > 3.6:
            throttle_target = max(0.0, throttle_target - 0.02)
        elif self.acceleration < -3.6:
            brake_target = max(0.0, brake_target - 0.04)

        self.throttle = clamp(
            self.throttle
            + clamp(
                throttle_target - self.throttle,
                -0.02,
                FOLLOW_THROTTLE_SLEW_PER_FRAME,
            ),
            0.0,
            1.0,
        )
        self.brake = clamp(
            self.brake
            + clamp(
                brake_target - self.brake,
                -FOLLOW_BRAKE_SLEW_PER_FRAME,
                FOLLOW_BRAKE_SLEW_PER_FRAME,
            ),
            0.0,
            1.0,
        )
        if self.throttle > 0.005:
            self.brake = 0.0
        elif self.brake > 0.005:
            self.throttle = 0.0

        self.stopped = (
            target_speed <= 0.02
            and abs(speed) <= 0.10
            and self.throttle <= 0.005
        )
        return self.throttle, self.brake
