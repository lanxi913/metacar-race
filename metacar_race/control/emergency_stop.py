# -*- coding: utf-8 -*-
"""车辆紧急停止的终点坐标系和停车速度规划。"""
from __future__ import annotations

import math
from dataclasses import dataclass

from metacar_race.config import (
    ACCEL_BONUS_LIMIT,
    EMERGENCY_STOP_CLEARANCE_RATIO,
    EMERGENCY_STOP_DECEL_RATIO,
)


@dataclass(frozen=True)
class RouteStopFrame:
    x: float
    y: float
    forward_x: float
    forward_y: float


@dataclass(frozen=True)
class StopCoordinates:
    along_m: float
    lateral_m: float


def make_stop_frame(end_point, route) -> RouteStopFrame:
    """以路线终点和最后一段路线方向建立停车坐标系。"""
    if len(route) < 2:
        raise ValueError("route 至少需要两个点")
    previous = route[-2]
    dx = float(end_point.x) - float(previous.x)
    dy = float(end_point.y) - float(previous.y)
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        dx = float(end_point.x) - float(route[0].x)
        dy = float(end_point.y) - float(route[0].y)
        length = math.hypot(dx, dy)
    if length <= 1e-9:
        raise ValueError("无法从路线推导终点方向")
    return RouteStopFrame(
        x=float(end_point.x),
        y=float(end_point.y),
        forward_x=dx / length,
        forward_y=dy / length,
    )


def stop_coordinates(pose, frame: RouteStopFrame) -> StopCoordinates:
    """返回主车相对终点的前向剩余距离和左向偏差。"""
    dx = frame.x - float(pose.pos_x)
    dy = frame.y - float(pose.pos_y)
    along = dx * frame.forward_x + dy * frame.forward_y
    lateral = -dx * frame.forward_y + dy * frame.forward_x
    return StopCoordinates(along_m=along, lateral_m=lateral)


def braking_speed_limit(along_m: float) -> float:
    decel = max(
        0.5,
        EMERGENCY_STOP_DECEL_RATIO * ACCEL_BONUS_LIMIT,
    )
    return math.sqrt(2.0 * decel * max(0.0, along_m))


def minimum_endpoint_clearance(ego_length: float) -> float:
    return max(0.10, EMERGENCY_STOP_CLEARANCE_RATIO * ego_length)


def endpoint_target_speed(
    along_m: float,
    speed_mps: float,
    max_speed_mps: float,
    response_time_s: float,
    clearance_m: float,
) -> float:
    """在停车制动区内不允许因距离误差要求再次加速。"""
    speed = max(0.0, speed_mps)
    decel = max(
        0.5,
        EMERGENCY_STOP_DECEL_RATIO * ACCEL_BONUS_LIMIT,
    )
    response_distance = response_time_s * speed + clearance_m
    profile = min(
        max_speed_mps,
        braking_speed_limit(max(0.0, along_m - response_distance)),
    )
    stopping_distance = (
        response_time_s * speed
        + speed * speed / (2.0 * decel)
        + clearance_m
    )
    if along_m <= stopping_distance:
        return min(profile, speed)
    return profile


def safe_stop_margin(leader_clearance_at_end: float | None, min_clearance: float) -> float:
    """在前车终点余量中保留一半作为制动超调缓冲。"""
    if leader_clearance_at_end is None:
        return 0.0
    available = max(0.0, leader_clearance_at_end - min_clearance)
    return 0.5 * available
