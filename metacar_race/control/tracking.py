# -*- coding: utf-8 -*-
"""纯跟踪（Pure Pursuit）+ 加速度受限的速度环。

设计目标：在「车辆直行」这类场景里稳稳跟着 route 走，加速度全程不超过 5 m/s²
（场景元素评分里 |a| ≤ 5 m/s² 每题 +10）。

用法见 main.py 的 --track 分支。
"""
from __future__ import annotations

import math

from metacar_race.config import (
    ACCEL_LIMIT_MS2,
    UPSHIFT_SPEED_THRESHOLD,
    LOOKAHEAD_M,
    MAX_STEER_RAD,
    TRACK_SPEED_MS,
    WHEELBASE_M,
)


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PurePursuit:
    """按路点序列算方向盘转角，输出 -1~1。"""

    def __init__(self, route, lookahead=LOOKAHEAD_M):
        self.route = list(route)
        self.lookahead = float(lookahead)
        self.idx = 0

    def reset(self):
        self.idx = 0

    def _nearest(self, x: float, y: float):
        """在 [idx, idx+40] 窗口里找最近点，索引只许往前，不许回退。"""
        n = len(self.route)
        lo = min(self.idx, n - 1)
        hi = min(n - 1, lo + 40)
        best_i, best_d = lo, float("inf")
        for i in range(lo, hi + 1):
            p = self.route[i]
            d = math.hypot(p.x - x, p.y - y)
            if d < best_d:
                best_i, best_d = i, d
        self.idx = max(self.idx, best_i)
        return self.idx

    def _target(self, x: float, y: float):
        """从最近点沿路径往前找，取第一个距离 ≥ 前视距离的点。"""
        start = self._nearest(x, y)
        n = len(self.route)
        for i in range(start, n):
            p = self.route[i]
            if math.hypot(p.x - x, p.y - y) >= self.lookahead:
                return p
        return self.route[-1]

    def steering(self, x: float, y: float, yaw_deg: float) -> float:
        if len(self.route) < 2:
            return 0.0

        target = self._target(x, y)
        dx, dy = target.x - x, target.y - y
        dist = max(math.hypot(dx, dy), 1e-3)

        # 转到车体坐标：x 向前，y 向左
        # 注意：场景里的 yaw 是顺时针为正（yaw=90° 时车头朝 -y），先转成数学角
        yaw = math.radians(-yaw_deg)
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

        # 纯跟踪曲率 κ = 2·y / L²，再折算成前轮转角
        curvature = 2.0 * local_y / (dist * dist)
        delta = math.atan(curvature * WHEELBASE_M)

        # 目标点在车后（倒着开）时直接打满方向救回来
        if local_x < 0.0:
            delta = MAX_STEER_RAD if local_y > 0 else -MAX_STEER_RAD

        # 场景里 steering 为正 = 往 -x 侧偏（实测），与纯跟踪的 delta 反号
        return max(-1.0, min(1.0, -delta / MAX_STEER_RAD))


class SpeedController:
    """速度环：先把目标速度做成斜坡，再用「期望加速度」反推油门/刹车。

    车辆全油门大约能到 10 m/s²，所以不能直接拿速度误差当油门，必须按加速度闭环控制，
    才能把 |a| 压在 5 m/s² 以内（场景元素加分项）。
    """

    def __init__(self, v_target=TRACK_SPEED_MS, accel_limit=ACCEL_LIMIT_MS2,
                 throttle_max=0.40, kick_throttle=0.45, kick_time=0.40):
        self.v_target = float(v_target)
        self.accel_limit = float(accel_limit)
        # 0.40 是老的单场景限速（约 17.2 km/h）。要跑到 8 m/s 必须放开，交给加速度闭环兜住。
        self.throttle_max = float(throttle_max)
        # 升挡 kick 的强度：顶满油门 0.8 s 会把车速直接甩到 14.3 m/s（实测 20261024-131247），
        # 只能给到刚好触发换挡的量。
        self.kick_throttle = float(kick_throttle)
        self.kick_time = float(kick_time)
        self.v_cmd = 0.0          # 当前允许的速度指令（带斜坡）
        self._throttle = 0.0
        self._brake = 0.0
        self._a_f = 0.0           # 滤波后的实测加速度
        self._stuck_t = 0.0       # 卡在低挡位累计时间
        self._kick_left = 0.0     # 升挡 kick 剩余时间
        self._v_prev = 0.0

    def reset(self, v_now=0.0):
        self.v_cmd = float(v_now)
        self._throttle = 0.0
        self._brake = 0.0
        self._a_f = 0.0
        self._stuck_t = 0.0
        self._kick_left = 0.0
        self._v_prev = float(v_now)

    def update(self, v_now: float, dt: float, v_ceiling=None, a_now=0.0,
               allow_kick=False):
        """返回 (throttle, brake)。v_ceiling 用于终点前限速。"""
        if dt is None or dt <= 0.0:
            dt = 1e-3

        # 1) 速度指令斜坡
        target = self.v_target if v_ceiling is None else min(self.v_target, v_ceiling)
        step = self.accel_limit * dt
        self.v_cmd += max(-step, min(step, target - self.v_cmd))

        # 2) 实测加速度滤波（速度是离散的，原始值毛刺很大）
        a_raw = max(-10.0, min(10.0, a_now))
        self._a_f += 0.25 * (a_raw - self._a_f)

        # 3) 期望加速度：跟着速度误差走，且不超过上限
        a_des = 0.9 * (self.v_cmd - v_now)
        a_des = max(-self.accel_limit, min(self.accel_limit, a_des))

        # 4) 需要减速 -> 松油、给刹车
        if a_des < -0.2:
            self._throttle = max(0.0, self._throttle - 0.10)
            brake_cmd = min(1.0, -a_des / 6.0)
        # 5) 需要加速 -> 用加速度误差积分油门，刹车间隙释放
        else:
            self._throttle += max(-0.03, min(0.03, 0.03 * (a_des - self._a_f)))
            # 全油门约 10 m/s²，这里限制油门把加速度压在 5 以内
            self._throttle = max(0.0, min(self.throttle_max, self._throttle))
            brake_cmd = 0.0

        # 5.5) 升挡 kick：低挡位下油门再大也上不去（实测卡 17.2 km/h），
        #      需要短暂给一脚油门触发换挡，再回落到闭环。
        #      20261024-131247 那次 1.0 油门顶 0.8 s，车速从 4.77 甩到 14.28 m/s，
        #      直接把直角转弯的加速度奖励打没了，所以强度和时长都要收。
        if self._kick_left > 0.0:
            self._kick_left -= dt
            self._throttle = self.kick_throttle
            self._brake = 0.0
            self._stuck_t = 0.0
            return self._throttle, self._brake

        if (allow_kick and v_now < UPSHIFT_SPEED_THRESHOLD
                and self.v_cmd > UPSHIFT_SPEED_THRESHOLD + 0.8):
            if abs(v_now - self._v_prev) < 0.03:
                self._stuck_t += dt
            else:
                self._stuck_t = 0.0
            if self._stuck_t > 0.8:
                self._kick_left = self.kick_time
                self._stuck_t = 0.0
        else:
            self._stuck_t = 0.0
        self._v_prev = v_now

        # 6) 刹车也要斜坡，避免点头
        self._brake += max(-0.10, min(0.10, brake_cmd - self._brake))
        self._brake = max(0.0, min(1.0, self._brake))

        return self._throttle, self._brake


def distance_left(route, x, y, from_idx=0) -> float:
    """剩余路径长度（粗略：从最近点累加到终点）。"""
    if not route:
        return 0.0
    best_i, best_d = from_idx, float("inf")
    for i in range(from_idx, len(route)):
        p = route[i]
        d = math.hypot(p.x - x, p.y - y)
        if d < best_d:
            best_d, best_i = d, i

    total = 0.0
    for i in range(best_i, len(route) - 1):
        a, b = route[i], route[i + 1]
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total + best_d

