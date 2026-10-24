# -*- coding: utf-8 -*-
"""车辆停止控制与标定。

车辆停止题不能只复用循迹入口：循迹负责沿路线走，停车还需要读取
``SceneStatus.end_point``，提前按制动距离减速，并在到达后锁存制动。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from metacar import GearMode, VehicleControl

from metacar_race.config import (
    LOOKAHEAD_M,
    STOP_BRAKE_COMMAND,
    STOP_BRAKE_DECEL_MS2,
    STOP_BRAKE_LEAD_M,
    STOP_CAPTURE_RADIUS_M,
    STOP_HOLD_SECONDS,
    STOP_SETTLE_SECONDS,
    STOP_STEER_LOCK_DISTANCE_M,
    STOP_STILL_SPEED_MS,
    STOP_THROTTLE_COMMAND,
)
from metacar_race.control.tracking import PurePursuit


@dataclass(frozen=True)
class StopInfo:
    distance_to_end: float
    along_track: float
    target_speed: float
    stopped: bool
    held_seconds: float


class StopController:
    """循迹到终点后停车，并保持制动 5 s。"""

    def __init__(
        self,
        route,
        lookahead: float = LOOKAHEAD_M,
        throttle_command: float = STOP_THROTTLE_COMMAND,
        steer_lock_distance: float = STOP_STEER_LOCK_DISTANCE_M,
    ):
        self.pursuit = PurePursuit(route, lookahead=lookahead)
        self.throttle_command = float(throttle_command)
        self.steer_lock_distance = float(steer_lock_distance)
        self.stopped = False
        self.held_seconds = 0.0
        self.braking = False
        self._low_speed_seconds = 0.0

    def reset(self):
        self.pursuit.reset()
        self.stopped = False
        self.held_seconds = 0.0
        self.braking = False
        self._low_speed_seconds = 0.0

    @staticmethod
    def _pose_heading(yaw_deg: float) -> float:
        # 场景 yaw 顺时针为正，数学坐标中的航向要取反。
        return -math.radians(float(yaw_deg))

    def control(
        self,
        x: float,
        y: float,
        yaw_deg: float,
        speed: float,
        dt: float,
        end_point,
    ) -> tuple[VehicleControl, StopInfo]:
        if end_point is None:
            return (
                VehicleControl(gear=GearMode.PARKING, brake=1.0),
                StopInfo(math.inf, math.inf, 0.0, True, self.held_seconds),
            )

        dx = float(end_point.x) - x
        dy = float(end_point.y) - y
        distance = math.hypot(dx, dy)
        yaw = self._pose_heading(yaw_deg)
        along_track = dx * math.cos(yaw) + dy * math.sin(yaw)

        if self.stopped:
            self.held_seconds += max(0.0, dt)
            return (
                VehicleControl(gear=GearMode.PARKING, brake=1.0),
                StopInfo(distance, along_track, 0.0, True, self.held_seconds),
            )

        dt = max(0.0, dt)

        # 用标定后的固定刹车距离决定何时开始一次连续制动，不再根据抖动的
        # car.speed 在油门和刹车之间逐帧来回切。
        stopping_distance = (
            speed * max(0.0, speed) / (2.0 * STOP_BRAKE_DECEL_MS2)
            + STOP_BRAKE_LEAD_M
        )
        if not self.braking and along_track <= stopping_distance:
            self.braking = True

        if self.braking:
            throttle = 0.0
            brake = STOP_BRAKE_COMMAND
            if abs(speed) <= STOP_STILL_SPEED_MS:
                self._low_speed_seconds += dt
            else:
                self._low_speed_seconds = 0.0
            if (
                self._low_speed_seconds >= STOP_SETTLE_SECONDS
                and distance <= STOP_CAPTURE_RADIUS_M + 0.20
            ):
                self.stopped = True
                self.held_seconds = 0.0
                return (
                    VehicleControl(gear=GearMode.PARKING, brake=1.0),
                    StopInfo(distance, along_track, 0.0, True, 0.0),
                )
        else:
            throttle = self.throttle_command
            brake = 0.0
            self._low_speed_seconds = 0.0

        if (
            self.steer_lock_distance > 0.0
            and along_track <= self.steer_lock_distance
        ):
            steering = 0.0
        else:
            steering = self.pursuit.steering(x, y, yaw_deg)

        return (
            VehicleControl(
                gear=GearMode.DRIVE,
                throttle=float(throttle),
                brake=float(brake),
                steering=float(steering),
            ),
            StopInfo(
                distance,
                along_track,
                0.0 if self.braking else self.throttle_command,
                self.stopped,
                self.held_seconds,
            ),
        )


def hold_complete(info: StopInfo) -> bool:
    return info.stopped and info.held_seconds >= STOP_HOLD_SECONDS
