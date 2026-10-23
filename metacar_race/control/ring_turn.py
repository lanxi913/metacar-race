# -*- coding: utf-8 -*-
"""环形转弯的自适应循迹、曲率限速和纵向控制。"""
from __future__ import annotations

import math
from dataclasses import dataclass

from metacar_race.config import (
    LIMIT_SCENE_ELEMENT_KMH,
    MAX_STEER_RAD,
    RING_TURN_CROSS_TRACK_BAND_RATIO,
    RING_TURN_CROSS_TRACK_MIN_SPEED_RATIO,
    RING_TURN_CURVATURE_SAMPLE_M,
    RING_TURN_LATERAL_ACCEL_MS2,
    RING_TURN_LOOKAHEAD_CAR_LENGTHS,
    RING_TURN_LOOKAHEAD_LANE_MAX_RATIO,
    RING_TURN_LOOKAHEAD_LANE_MIN_RATIO,
    RING_TURN_LOOKAHEAD_RESPONSE_S,
    RING_TURN_MAX_SPEED_RATIO,
    RING_TURN_MIN_SPEED_MS,
    RING_TURN_PLAN_DECEL_MS2,
    RING_TURN_REACTION_TIME_S,
    RING_TURN_SHIFT_URGENCY_RATIO,
    RING_TURN_SHIFT_COOLDOWN_SECONDS,
    RING_TURN_SHIFT_CURVATURE_MARGIN_MS,
    RING_TURN_SHIFT_EXIT_SPEED_MS,
    RING_TURN_SHIFT_MAX_SECONDS,
    RING_TURN_SHIFT_STABLE_SECONDS,
    RING_TURN_SHIFT_THROTTLE_RAMP_PER_S,
    RING_TURN_SPEED_MARGIN_RATIO,
    RING_TURN_TARGET_SPEED_SLEW_MS2,
    RING_TURN_TIME_BUDGET_RATIO,
    RING_TURN_THROTTLE_RELEASE_PER_SECOND,
    RING_TURN_THROTTLE_SLEW_PER_SECOND,
    RING_TURN_BRAKE_RELEASE_PER_SECOND,
    RING_TURN_BRAKE_SLEW_PER_SECOND,
    RING_TURN_BRAKE_START_ERROR_MS,
    RING_TURN_CRUISE_THROTTLE_MAX,
    RING_TURN_POST_SHIFT_SETTLE_SECONDS,
    RING_TURN_SHIFT_THROTTLE_RELEASE_PER_S,
    RING_TURN_START_EXIT_SPEED_MS,
    RING_TURN_START_RAMP_SECONDS,
    RING_TURN_START_THROTTLE_MAX,
    UPSHIFT_SPEED_THRESHOLD,
    WHEELBASE_M,
)
from metacar_race.control.tracking import SpeedController


def wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def points_to_xy(points) -> list[tuple[float, float]]:
    result = []
    for point in points:
        if isinstance(point, (tuple, list)):
            result.append((float(point[0]), float(point[1])))
        else:
            result.append((float(point.x), float(point.y)))
    return result


def nearest_segment(points_xy, x: float, y: float):
    """返回折线最近投影的距离、线段索引和累计弧长。"""
    if not points_xy:
        return None
    if len(points_xy) == 1:
        px, py = points_xy[0]
        return math.hypot(px - x, py - y), 0, 0.0

    best = None
    arc = 0.0
    for i in range(len(points_xy) - 1):
        ax, ay = points_xy[i]
        bx, by = points_xy[i + 1]
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        length = math.sqrt(length_sq)
        if length <= 1e-9:
            continue
        t = clamp(((x - ax) * dx + (y - ay) * dy) / length_sq, 0.0, 1.0)
        px, py = ax + t * dx, ay + t * dy
        candidate = (math.hypot(px - x, py - y), i, arc + t * length)
        if best is None or candidate[0] < best[0]:
            best = candidate
        arc += length
    return best


def estimate_lane_width(roads, x: float, y: float, fallback: float) -> float:
    """从运行时道路数据选择最近车道宽度，不依赖车道 ID。"""
    widths = []
    for road in roads:
        for lane in road.lanes:
            match = nearest_segment(points_to_xy(lane.path_points), x, y)
            if match is not None:
                widths.append((match[0], float(lane.width)))
    if not widths:
        return fallback
    return min(widths, key=lambda item: item[0])[1]


def pursuit_steering(
    points_xy,
    x: float,
    y: float,
    yaw_deg: float,
    lookahead_m: float,
) -> tuple[float, int, float]:
    """纯跟踪转向，返回 steering、目标索引和到路径的横向误差。"""
    if len(points_xy) < 2:
        return 0.0, 0, 0.0

    nearest_index = 0
    nearest_distance = float("inf")
    for i, (px, py) in enumerate(points_xy):
        distance = math.hypot(px - x, py - y)
        if distance < nearest_distance:
            nearest_index = i
            nearest_distance = distance

    target_index = len(points_xy) - 1
    for i in range(nearest_index, len(points_xy)):
        px, py = points_xy[i]
        if math.hypot(px - x, py - y) >= lookahead_m:
            target_index = i
            break

    tx, ty = points_xy[target_index]
    dx, dy = tx - x, ty - y
    distance = max(math.hypot(dx, dy), 1e-3)
    yaw = math.radians(-yaw_deg)
    local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
    local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

    if local_x < 0.0:
        delta = MAX_STEER_RAD if local_y > 0.0 else -MAX_STEER_RAD
    else:
        curvature = 2.0 * local_y / (distance * distance)
        delta = math.atan(curvature * WHEELBASE_M)

    steering = clamp(-delta / MAX_STEER_RAD, -1.0, 1.0)
    return steering, target_index, nearest_distance


@dataclass(frozen=True)
class RouteProgress:
    index: int
    cross_track_m: float
    remaining_m: float
    curvature_per_m: float


class RouteGeometry:
    """预计算静态路线的累计弧长和局部曲率。"""

    def __init__(self, points):
        self.points_xy = points_to_xy(points)
        self.cumulative = [0.0]
        for i in range(1, len(self.points_xy)):
            ax, ay = self.points_xy[i - 1]
            bx, by = self.points_xy[i]
            self.cumulative.append(
                self.cumulative[-1] + math.hypot(bx - ax, by - ay)
            )
        self.total_m = self.cumulative[-1] if self.cumulative else 0.0
        self.sample_span = self._sample_span()
        self.curvature = self._curvature()

    def _sample_span(self) -> int:
        if len(self.points_xy) < 3 or self.total_m <= 0.0:
            return 1
        spacing = self.total_m / max(1, len(self.points_xy) - 1)
        return max(2, int(round(RING_TURN_CURVATURE_SAMPLE_M / spacing)))

    def _curvature(self) -> list[float]:
        curvature = [0.0] * len(self.points_xy)
        for i in range(1, len(self.points_xy) - 1):
            j0 = max(0, i - self.sample_span)
            j1 = min(len(self.points_xy) - 1, i + self.sample_span)
            if j1 <= i or i <= j0:
                continue
            x0, y0 = self.points_xy[j0]
            x1, y1 = self.points_xy[i]
            x2, y2 = self.points_xy[j1]
            inbound = math.atan2(y1 - y0, x1 - x0)
            outbound = math.atan2(y2 - y1, x2 - x1)
            arc = self.cumulative[j1] - self.cumulative[j0]
            curvature[i] = abs(wrap(outbound - inbound)) / max(arc, 1e-3)
        return curvature

    def nearest(self, x: float, y: float, start_index: int = 0) -> RouteProgress:
        if not self.points_xy:
            return RouteProgress(0, 0.0, 0.0, 0.0)
        lo = max(0, start_index - 4)
        hi = min(len(self.points_xy), start_index + 80)
        segment = nearest_segment(self.points_xy[lo:hi], x, y)
        if segment is None:
            return RouteProgress(lo, 0.0, self.total_m, 0.0)
        distance, local_index, arc = segment
        index = min(len(self.points_xy) - 1, lo + local_index)
        absolute_cursor = self.cumulative[lo] + arc
        return RouteProgress(
            index=index,
            cross_track_m=distance,
            remaining_m=max(0.0, self.total_m - absolute_cursor),
            curvature_per_m=self.curvature[index],
        )

    def point_ahead(self, index: int, distance_m: float) -> tuple[float, float]:
        if not self.points_xy:
            raise ValueError("路线不能为空")
        target_arc = self.cumulative[index] + max(0.0, distance_m)
        j = index
        while (
            j < len(self.points_xy) - 1
            and self.cumulative[j + 1] < target_arc
        ):
            j += 1
        if j >= len(self.points_xy) - 1:
            return self.points_xy[-1]

        segment_length = self.cumulative[j + 1] - self.cumulative[j]
        if segment_length <= 1e-9:
            return self.points_xy[j]
        t = clamp(
            (target_arc - self.cumulative[j]) / segment_length,
            0.0,
            1.0,
        )
        ax, ay = self.points_xy[j]
        bx, by = self.points_xy[j + 1]
        return ax + t * (bx - ax), ay + t * (by - ay)

    def curvature_speed_limit(self, index: int, speed_ms: float) -> float:
        if not self.curvature:
            return LIMIT_SCENE_ELEMENT_KMH / 3.6
        preview = (
            max(
                RING_TURN_CURVATURE_SAMPLE_M * 2.0,
                speed_ms * RING_TURN_REACTION_TIME_S,
            )
            + speed_ms * speed_ms / (2.0 * RING_TURN_PLAN_DECEL_MS2)
        )
        limit = LIMIT_SCENE_ELEMENT_KMH / 3.6
        cursor = self.cumulative[index]
        for j in range(index, len(self.points_xy)):
            ahead = self.cumulative[j] - cursor
            if ahead > preview:
                break
            curvature = self.curvature[j]
            if curvature <= 1e-6:
                continue
            corner_speed = math.sqrt(
                RING_TURN_LATERAL_ACCEL_MS2 / curvature
            )
            allowed_now = math.sqrt(
                corner_speed * corner_speed
                + 2.0 * RING_TURN_PLAN_DECEL_MS2 * max(0.0, ahead)
            )
            limit = min(limit, allowed_now)
        return limit

    def curvature_speed_capacity(self, index: int, distance_m: float) -> float:
        """未来一段路线上最急弯允许的速度。"""
        if not self.curvature:
            return LIMIT_SCENE_ELEMENT_KMH / 3.6
        limit = LIMIT_SCENE_ELEMENT_KMH / 3.6
        cursor = self.cumulative[index]
        for j in range(index, len(self.points_xy)):
            if self.cumulative[j] - cursor > distance_m:
                break
            if self.curvature[j] > 1e-6:
                limit = min(
                    limit,
                    math.sqrt(
                        RING_TURN_LATERAL_ACCEL_MS2 / self.curvature[j]
                    ),
                )
        return limit


@dataclass(frozen=True)
class RingTurnResult:
    steering: float
    throttle: float
    brake: float
    target_speed_ms: float
    required_speed_ms: float
    route_index: int
    cross_track_m: float
    remaining_m: float
    curvature_per_m: float
    lookahead_m: float
    path_source: str
    finish_mode: bool
    time_budget_left_s: float
    shift_assist_allowed: bool
    shift_assist_active: bool
    target_speed_filtered_ms: float
    startup_active: bool


class RingTurnController:
    def __init__(
        self,
        route,
        car_length: float,
        car_width: float,
        lane_width: float,
    ):
        self.geometry = RouteGeometry(route)
        self.car_length = max(float(car_length), 1.0)
        self.car_width = max(float(car_width), 0.5)
        self.lane_width = max(float(lane_width), self.car_width * 1.4)
        self.index = 0
        self.previous_time = None
        max_speed = LIMIT_SCENE_ELEMENT_KMH / 3.6 * RING_TURN_MAX_SPEED_RATIO
        self.speed_controller = SpeedController(v_target=max_speed)
        self.speed_controller.reset(0.0)
        self.target_speed_cmd = RING_TURN_MIN_SPEED_MS
        self.throttle_cmd = 0.0
        self.brake_cmd = 0.0
        self.shift_stable_time = 0.0
        self.shift_active_time = 0.0
        self.shift_cooldown_until = 0.0
        self.shift_complete = False
        self.post_shift_settle_until = 0.0
        self.previous_speed = 0.0
        self.startup_complete = False

    def _lookahead(self, speed_ms: float) -> float:
        raw = (
            max(0.0, speed_ms) * RING_TURN_LOOKAHEAD_RESPONSE_S
            + RING_TURN_LOOKAHEAD_CAR_LENGTHS * self.car_length
        )
        return clamp(
            raw,
            self.lane_width * RING_TURN_LOOKAHEAD_LANE_MIN_RATIO,
            self.lane_width * RING_TURN_LOOKAHEAD_LANE_MAX_RATIO,
        )

    def _smooth_target(self, target_speed_ms: float, dt: float) -> float:
        max_step = RING_TURN_TARGET_SPEED_SLEW_MS2 * dt
        self.target_speed_cmd += clamp(
            target_speed_ms - self.target_speed_cmd,
            -max_step,
            max_step,
        )
        return self.target_speed_cmd

    def _smooth_actuators(
        self,
        throttle_target: float,
        brake_target: float,
        dt: float,
    ) -> tuple[float, float]:
        if throttle_target > self.throttle_cmd:
            throttle_step = RING_TURN_THROTTLE_SLEW_PER_SECOND * dt
        else:
            throttle_step = RING_TURN_THROTTLE_RELEASE_PER_SECOND * dt
        self.throttle_cmd += clamp(
            throttle_target - self.throttle_cmd,
            -throttle_step,
            throttle_step,
        )
        self.throttle_cmd = clamp(self.throttle_cmd, 0.0, 1.0)

        if brake_target > self.brake_cmd:
            brake_step = RING_TURN_BRAKE_SLEW_PER_SECOND * dt
        else:
            brake_step = RING_TURN_BRAKE_RELEASE_PER_SECOND * dt
        self.brake_cmd += clamp(
            brake_target - self.brake_cmd,
            -brake_step,
            brake_step,
        )
        self.brake_cmd = clamp(self.brake_cmd, 0.0, 1.0)
        return self.throttle_cmd, self.brake_cmd

    def compute(
        self,
        x: float,
        y: float,
        yaw_deg: float,
        speed_ms: float,
        sim_time: float,
        time_limit: float,
        dynamic_path=None,
        a_now: float = 0.0,
    ) -> RingTurnResult:
        if self.previous_time is None:
            dt = 1.0 / 30.0
        else:
            dt = clamp(sim_time - self.previous_time, 0.01, 0.10)
        self.previous_time = sim_time

        progress = self.geometry.nearest(x, y, self.index)
        self.index = max(self.index, progress.index)
        lookahead = self._lookahead(speed_ms)
        dynamic_xy = points_to_xy(dynamic_path or [])
        if len(dynamic_xy) >= 2:
            steering, _, dynamic_cross = pursuit_steering(
                dynamic_xy,
                x,
                y,
                yaw_deg,
                lookahead,
            )
            cross_track = min(progress.cross_track_m, dynamic_cross)
            path_source = "trajectory"
        else:
            target = self.geometry.point_ahead(self.index, lookahead)
            static_xy = [(x, y), target]
            steering, _, _ = pursuit_steering(
                static_xy,
                x,
                y,
                yaw_deg,
                lookahead,
            )
            cross_track = progress.cross_track_m
            path_source = "static_route"

        speed_limit = (
            LIMIT_SCENE_ELEMENT_KMH
            / 3.6
            * RING_TURN_MAX_SPEED_RATIO
        )
        planned_end = float(time_limit) * RING_TURN_TIME_BUDGET_RATIO
        time_budget_left = max(planned_end - float(sim_time), 0.1)
        required_speed = progress.remaining_m / time_budget_left
        raw_target_speed = clamp(
            required_speed * RING_TURN_SPEED_MARGIN_RATIO,
            RING_TURN_MIN_SPEED_MS,
            speed_limit,
        )
        raw_target_speed = min(
            raw_target_speed,
            self.geometry.curvature_speed_limit(self.index, speed_ms),
        )

        cross_band = self.lane_width * RING_TURN_CROSS_TRACK_BAND_RATIO
        excess = max(0.0, cross_track - cross_band)
        speed_scale = clamp(
            1.0 - excess / max(self.lane_width, 1.0),
            RING_TURN_CROSS_TRACK_MIN_SPEED_RATIO,
            1.0,
        )
        raw_target_speed *= speed_scale

        finish_mode = progress.remaining_m <= self.car_length * 0.75
        if finish_mode:
            steering = 0.0
            raw_target_speed = min(
                raw_target_speed,
                RING_TURN_MIN_SPEED_MS,
            )

        target_speed = self._smooth_target(raw_target_speed, dt)
        eta_s = progress.remaining_m / max(speed_ms, 0.5)
        speed_is_stable = abs(speed_ms - self.previous_speed) < 0.12
        if (
            UPSHIFT_SPEED_THRESHOLD - 0.55
            <= speed_ms
            < UPSHIFT_SPEED_THRESHOLD
            and speed_is_stable
        ):
            self.shift_stable_time += dt
        else:
            self.shift_stable_time = 0.0

        shift_preview = max(
            self.car_length * 4.0,
            speed_ms * (RING_TURN_REACTION_TIME_S + 2.0),
        )
        shift_capacity = self.geometry.curvature_speed_capacity(
            self.index,
            shift_preview,
        )
        shift_geometry_safe = (
            shift_capacity
            >= target_speed + RING_TURN_SHIFT_CURVATURE_MARGIN_MS
        )
        urgency = eta_s > RING_TURN_SHIFT_URGENCY_RATIO * time_budget_left
        shift_allowed = (
            self.startup_complete
            and not finish_mode
            and not self.shift_complete
            and sim_time >= self.shift_cooldown_until
            and target_speed > UPSHIFT_SPEED_THRESHOLD + 0.75
            and urgency
            and shift_geometry_safe
            and cross_track < self.lane_width * 0.5
            and self.shift_stable_time >= RING_TURN_SHIFT_STABLE_SECONDS
        )
        if shift_allowed and self.shift_active_time <= 0.0:
            self.shift_active_time = 1e-6

        shift_active = self.shift_active_time > 0.0
        if shift_active:
            self.shift_active_time += dt
            if (
                speed_ms >= RING_TURN_SHIFT_EXIT_SPEED_MS
                or self.shift_active_time >= RING_TURN_SHIFT_MAX_SECONDS
            ):
                self.shift_complete = (
                    speed_ms >= RING_TURN_SHIFT_EXIT_SPEED_MS
                )
                self.shift_active_time = 0.0
                self.shift_cooldown_until = (
                    sim_time + RING_TURN_SHIFT_COOLDOWN_SECONDS
                )
                self.post_shift_settle_until = (
                    sim_time + RING_TURN_POST_SHIFT_SETTLE_SECONDS
                )
                shift_active = False

        startup_active = not self.startup_complete
        if startup_active:
            ramp = clamp(sim_time / RING_TURN_START_RAMP_SECONDS, 0.0, 1.0)
            startup_throttle = RING_TURN_START_THROTTLE_MAX * ramp
            startup_rate = (
                RING_TURN_START_THROTTLE_MAX
                / max(RING_TURN_START_RAMP_SECONDS, 0.1)
            )
            self.throttle_cmd = min(
                startup_throttle,
                self.throttle_cmd + startup_rate * dt,
            )
            self.brake_cmd = max(
                0.0,
                self.brake_cmd - RING_TURN_BRAKE_RELEASE_PER_SECOND * dt,
            )
            throttle, brake = self.throttle_cmd, self.brake_cmd
            if speed_ms >= RING_TURN_START_EXIT_SPEED_MS:
                self.startup_complete = True
                self.speed_controller.reset(speed_ms)
        elif shift_active:
            shift_step = RING_TURN_SHIFT_THROTTLE_RAMP_PER_S * dt
            self.throttle_cmd = min(1.0, self.throttle_cmd + shift_step)
            self.brake_cmd = max(
                0.0,
                self.brake_cmd - RING_TURN_BRAKE_RELEASE_PER_SECOND * dt,
            )
            throttle, brake = self.throttle_cmd, self.brake_cmd
        else:
            if sim_time < self.post_shift_settle_until:
                self.throttle_cmd = max(
                    0.0,
                    self.throttle_cmd
                    - RING_TURN_SHIFT_THROTTLE_RELEASE_PER_S * dt,
                )
                self.brake_cmd = max(
                    0.0,
                    self.brake_cmd
                    - RING_TURN_BRAKE_RELEASE_PER_SECOND * dt,
                )
                throttle, brake = self.throttle_cmd, self.brake_cmd
            else:
                raw_throttle, raw_brake = self.speed_controller.update(
                    speed_ms,
                    dt,
                    v_ceiling=target_speed,
                    a_now=a_now,
                    allow_kick=False,
                )
                raw_throttle = min(
                    raw_throttle,
                    RING_TURN_CRUISE_THROTTLE_MAX,
                )
                if speed_ms <= target_speed + RING_TURN_BRAKE_START_ERROR_MS:
                    raw_brake = 0.0
                if raw_brake > 0.0:
                    raw_throttle = 0.0
                else:
                    raw_brake = 0.0
                throttle, brake = self._smooth_actuators(
                    raw_throttle,
                    raw_brake,
                    dt,
                )
                if brake > 0.0:
                    throttle = 0.0
        self.previous_speed = speed_ms
        return RingTurnResult(
            steering=steering,
            throttle=throttle,
            brake=brake,
            target_speed_ms=raw_target_speed,
            required_speed_ms=required_speed,
            route_index=self.index,
            cross_track_m=cross_track,
            remaining_m=progress.remaining_m,
            curvature_per_m=progress.curvature_per_m,
            lookahead_m=lookahead,
            path_source=path_source,
            finish_mode=finish_mode,
            time_budget_left_s=time_budget_left,
            shift_assist_allowed=shift_allowed,
            shift_assist_active=shift_active,
            target_speed_filtered_ms=self.target_speed_cmd,
            startup_active=startup_active,
        )
