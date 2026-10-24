# -*- coding: utf-8 -*-
"""垂直车道泊车路径生成。

推荐路线只用于驶近泊车区域。最后一段根据动态读取到的 PARKING_SLOT
几何重新生成，保证车辆沿车位长轴驶入并在车位中心附近停车。
"""
from __future__ import annotations

import math

from metacar import Vector3

from metacar_race.config import MAX_STEER_RAD, WHEELBASE_M


def nearest_parallel_yaw(current_yaw: float, slot_yaw: float) -> float:
    """返回与车位长轴平行且离当前车头最近的目标航向。"""
    first = float(slot_yaw)
    second = first + 180.0

    def distance(angle: float) -> float:
        return abs((angle - current_yaw + 180.0) % 360.0 - 180.0)

    return first if distance(first) <= distance(second) else second


def yaw_error(current_yaw: float, target_yaw: float) -> float:
    """目标航向减当前航向，范围 [-180, 180] 度。"""
    return (target_yaw - current_yaw + 180.0) % 360.0 - 180.0


def slot_travel_direction(
    start_x: float,
    start_y: float,
    slot,
) -> tuple[float, float]:
    """车辆沿车位长轴驶入时的单位方向。"""
    slot_math_yaw = -math.radians(float(slot.ori_z))
    axis = (math.cos(slot_math_yaw), math.sin(slot_math_yaw))
    to_vehicle = (
        float(start_x) - float(slot.pos_x),
        float(start_y) - float(slot.pos_y),
    )
    sign = -1.0 if axis[0] * to_vehicle[0] + axis[1] * to_vehicle[1] > 0.0 else 1.0
    return sign * axis[0], sign * axis[1]


def _bezier(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    samples: int = 60,
) -> list[Vector3]:
    points = []
    for index in range(1, samples + 1):
        t = index / samples
        u = 1.0 - t
        x = (
            u * u * u * p0[0]
            + 3.0 * u * u * t * p1[0]
            + 3.0 * u * t * t * p2[0]
            + t * t * t * p3[0]
        )
        y = (
            u * u * u * p0[1]
            + 3.0 * u * u * t * p1[1]
            + 3.0 * u * t * t * p2[1]
            + t * t * t * p3[1]
        )
        points.append(Vector3(x=x, y=y, z=0.0))
    return points


def build_vertical_parking_path(
    route,
    slot,
    approach_distance: float,
    align_extension: float,
) -> list[Vector3]:
    """根据推荐路线和停车位动态生成最后一段泊车路径。"""
    if not route or slot is None:
        return list(route)

    cumulative = [0.0]
    for previous, current in zip(route, route[1:]):
        cumulative.append(
            cumulative[-1]
            + math.hypot(current.x - previous.x, current.y - previous.y)
        )
    total = cumulative[-1]

    approach_index = max(0, len(route) - 2)
    for index, distance in enumerate(cumulative):
        if total - distance <= approach_distance:
            approach_index = index
            break

    lane_x = float(route[0].x)
    path = [
        Vector3(x=lane_x, y=float(point.y), z=0.0)
        for point in route[: approach_index + 1]
    ]
    if not path:
        return list(route)

    # 车位 ori_z=0 时长轴沿 x 轴；从车辆所在的一侧驶入车位。
    direction_x, direction_y = slot_travel_direction(
        float(route[0].x),
        float(route[0].y),
        slot,
    )
    entry_x = float(slot.pos_x) - direction_x * float(slot.length) / 2.0
    entry_y = float(slot.pos_y) - direction_y * float(slot.length) / 2.0

    start = (float(path[-1].x), float(path[-1].y))
    target = (float(slot.pos_x), float(slot.pos_y))
    control_1 = (start[0], start[1] - 2.5)
    control_2 = (entry_x, entry_y)
    path.extend(_bezier(start, control_1, control_2, target))

    # 车位中心之后继续延伸少量引导点，让前视目标仍在车位轴向。
    # 停车控制器使用车位中心作为独立终点，因此车辆不会实际开过中心。
    extension = max(0.0, float(align_extension))
    for index in range(1, 16):
        distance = extension * index / 15.0
        path.append(
            Vector3(
                x=target[0] + direction_x * distance,
                y=target[1] + direction_y * distance,
                z=0.0,
            )
        )
    return path


def build_alignment_path(
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    slot,
    control_distance: float,
    samples: int = 40,
) -> list[Vector3]:
    """倒车后从当前位姿平滑前进到车位中心并回正。"""
    start = (float(start_x), float(start_y))
    target = (float(slot.pos_x), float(slot.pos_y))
    start_math_yaw = -math.radians(float(start_yaw_deg))
    start_direction = (
        math.cos(start_math_yaw),
        math.sin(start_math_yaw),
    )

    target_direction = slot_travel_direction(start[0], start[1], slot)

    control = max(0.1, float(control_distance))
    p0 = start
    p1 = (
        start[0] + start_direction[0] * control,
        start[1] + start_direction[1] * control,
    )
    p2 = (
        target[0] - target_direction[0] * control,
        target[1] - target_direction[1] * control,
    )
    p3 = target
    path = [Vector3(x=p0[0], y=p0[1], z=0.0)] + _bezier(
        p0, p1, p2, p3, samples=samples
    )
    for index in range(1, 16):
        extension = control * index / 15.0
        path.append(
            Vector3(
                x=target[0] + target_direction[0] * extension,
                y=target[1] + target_direction[1] * extension,
                z=0.0,
            )
        )
    return path


def build_entrance_parking_paths(
    route,
    slot,
) -> tuple[list[Vector3], list[Vector3], Vector3]:
    """生成入口对齐路径和入口到车位中心的直线路径。"""
    start_x = float(route[0].x)
    start_y = float(route[0].y)
    direction = slot_travel_direction(start_x, start_y, slot)
    normal = (-direction[1], direction[0])
    to_start = (start_x - float(slot.pos_x), start_y - float(slot.pos_y))
    if normal[0] * to_start[0] + normal[1] * to_start[1] < 0.0:
        normal = (-normal[0], -normal[1])

    # 垂直车位从靠近车辆的一侧进入；这里使用车位长度建立入口前准备点。
    prep_distance = float(slot.length) + 0.5
    turning_span = WHEELBASE_M / math.tan(MAX_STEER_RAD)
    entry = Vector3(
        x=float(slot.pos_x) - direction[0] * float(slot.length) / 2.0,
        y=float(slot.pos_y) - direction[1] * float(slot.length) / 2.0,
        z=0.0,
    )
    prep_x = entry.x - direction[0] * turning_span
    prep_y = (
        float(slot.pos_y)
        + normal[1] * prep_distance
    )
    approach_delta_y = max(1e-6, start_y - prep_y)
    lane_path = []
    for point in route:
        point_y = float(point.y)
        if point_y < prep_y:
            continue
        blend = max(0.0, min(1.0, (start_y - point_y) / approach_delta_y))
        lane_path.append(
            Vector3(
                x=start_x + (prep_x - start_x) * blend,
                y=point_y,
                z=0.0,
            )
        )
    lane_path.append(Vector3(x=prep_x, y=prep_y, z=0.0))

    center = Vector3(
        x=float(slot.pos_x),
        y=float(slot.pos_y),
        z=float(slot.pos_z),
    )

    approach_y_sign = -1.0 if start_y > prep_y else 1.0
    p0 = (prep_x, prep_y)
    p1 = (prep_x, prep_y + approach_y_sign * turning_span)
    p2 = (
        entry.x - direction[0] * turning_span / 2.0,
        entry.y - direction[1] * turning_span / 2.0,
    )
    p3 = (entry.x, entry.y)
    lane_path.extend(_bezier(p0, p1, p2, p3, samples=60))

    entry_line = [entry]
    entry_line.extend(
        _bezier(
            p3,
            (
                entry.x + direction[0] * 1.0,
                entry.y + direction[1] * 1.0,
            ),
            (
                center.x - direction[0] * 1.0,
                center.y - direction[1] * 1.0,
            ),
            (center.x, center.y),
            samples=20,
        )
    )
    return lane_path, entry_line, entry
