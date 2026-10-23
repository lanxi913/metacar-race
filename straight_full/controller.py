# -*- coding: utf-8 -*-
"""「车辆直行」全分控制器。

设计目标（对应 docs/02-rules.md 场景元素评分）：
  * 到达终点 → 100 分
  * 全程 |a| ≤ 5 m/s² → +10 分
  * 不压线、不碰撞 → 不扣分

因此纵向一律按「加速度上限」闭环，宁可慢，也不让加速度冲高。

结构：
  RoutePath      路径几何：投影、剩余里程、前视点
  SpeedPlanner   梯形速度规划（受加速度上限约束）
  LongController 纵向：期望加速度 → 油门/刹车，含低挡位升挡 kick
  StraightDriver 组合三者，产出 VehicleControl
"""
from __future__ import annotations

import math

from metacar import GearMode, VehicleControl

# ---- 车辆/场景常量（估测，来自实测与 AGENTS.md）----
WHEELBASE_M = 2.6          # 轴距
MAX_STEER_RAD = 0.6        # 前轮最大转角，用于把曲率折算成 -1~1 的方向盘量
FULL_THROTTLE_ACCEL = 10.0  # 全油门纵向加速度 m/s²
UPSHIFT_SPEED = 4.9        # 低于此速度基本还在一挡（17.2 km/h），需要顶油门换挡


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class RoutePath:
    """把 route 点列封装成可按里程查询的路径。"""

    def __init__(self, points):
        self.pts = [(float(p.x), float(p.y)) for p in points]
        self.n = len(self.pts)
        self.cum = [0.0] * self.n
        for i in range(1, self.n):
            x0, y0 = self.pts[i - 1]
            x1, y1 = self.pts[i]
            self.cum[i] = self.cum[i - 1] + math.hypot(x1 - x0, y1 - y0)
        self.total = self.cum[-1] if self.n else 0.0
        self.idx = 0

    def project(self, x: float, y: float):
        """返回 (里程 s, 最近点索引, 横向偏差)。索引只允许往前搜，避免跳变。"""
        if self.n == 0:
            return 0.0, 0, 0.0
        if self.n == 1:
            return 0.0, 0, math.hypot(self.pts[0][0] - x, self.pts[0][1] - y)

        lo = min(self.idx, self.n - 1)
        hi = min(self.n - 1, lo + 60)
        best_i, best_d = lo, float("inf")
        for i in range(lo, hi + 1):
            px, py = self.pts[i]
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_i, best_d = i, d
        self.idx = max(self.idx, best_i)

        # 在最近线段上做投影，得到连续里程
        i = best_i
        if i + 1 < self.n:
            x0, y0 = self.pts[i]
            x1, y1 = self.pts[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg > 1e-6:
                t = ((x - x0) * (x1 - x0) + (y - y0) * (y1 - y0)) / (seg * seg)
                t = max(0.0, min(1.0, t))
                px, py = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
                s = self.cum[i] + t * seg
                # 横向偏差带符号：在路径左侧为正
                cross = (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0)
                lat = math.hypot(px - x, py - y)
                if cross < 0:
                    lat = -lat
                return s, i, lat

        px, py = self.pts[i]
        return self.cum[i], i, math.hypot(px - x, py - y)

    def point_at(self, s: float):
        """按里程取点（线性插值）。"""
        if self.n == 0:
            return 0.0, 0.0
        s = max(0.0, min(self.total, s))
        lo, hi = 0, self.n - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.cum[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        i = max(1, lo)
        s0, s1 = self.cum[i - 1], self.cum[i]
        if s1 - s0 < 1e-6:
            return self.pts[i]
        t = (s - s0) / (s1 - s0)
        x0, y0 = self.pts[i - 1]
        x1, y1 = self.pts[i]
        return x0 + t * (x1 - x0), y0 + t * (y1 - y0)


class SpeedPlanner:
    """梯形速度规划：先按上限加速，接近终点按刹车曲线收敛。"""

    def __init__(self, v_max: float = 10.0, a_up: float = 2.0, a_down: float = 1.8):
        self.v_max = float(v_max)
        self.a_up = float(a_up)
        self.a_down = float(a_down)
        self.v_cmd = 0.0   # 自己的指令状态，不能拿实测速度当时钟

    def reset(self, v_now: float = 0.0):
        self.v_cmd = float(v_now)

    def target(self, v_now: float, s_left: float, dt: float) -> float:
        """返回这一步允许的速度指令（已按加速度上限斜坡化）。"""
        # 刹车曲线：v ≤ sqrt(2·a·s)
        v_brake = math.sqrt(max(0.0, 2.0 * self.a_down * max(s_left, 0.0)))
        v_goal = min(self.v_max, v_brake)
        dt = max(dt, 1e-3)
        step = (self.a_up if v_goal >= self.v_cmd else self.a_down) * dt
        self.v_cmd += max(-step, min(step, v_goal - self.v_cmd))
        return self.v_cmd


class LongController:
    """纵向控制：期望加速度 → 油门/刹车。

    不做「误差直接当油门」的粗糙映射，而是用加速度误差做积分，把输出压在 0.45 以下，
    这样实测加速度不会超过 5 m/s²。
    """

    def __init__(self, accel_limit: float = 2.0, throttle_cap: float = 0.45):
        self.accel_limit = float(accel_limit)
        self.throttle_cap = float(throttle_cap)
        self.reset()

    def reset(self):
        self.throttle = 0.0
        self.brake = 0.0
        self._a_meas = 0.0
        self._plateau = 0.0
        self._kick = 0.0
        self._v_prev = None
        self._v_last = None   # 仅供卡滞检测，与加速度计算用的 _v_prev 分开

    def update(self, v: float, v_cmd: float, dt: float, allow_kick: bool = True):
        dt = max(dt, 1e-3)
        v_last = self._v_last
        self._v_last = v

        # 1) 期望加速度：跟随速度指令，限幅
        a_des = 1.1 * (v_cmd - v)
        a_des = max(-self.accel_limit, min(self.accel_limit, a_des))

        # 2) 实测加速度（一阶滤波）
        if self._v_prev is None:
            a_raw = 0.0
        else:
            a_raw = (v - self._v_prev) / dt
        self._v_prev = v
        a_raw = max(-12.0, min(12.0, a_raw))
        self._a_meas += 0.2 * (a_raw - self._a_meas)

        # 3) 换挡 kick 优先：低挡位下油门再高也上不去，必须顶满触发换挡
        if self._kick > 0.0:
            self._kick -= dt
            self.throttle, self.brake = 1.0, 0.0
            return self.throttle, self.brake

        if a_des < -0.3:
            # 需要减速：松油 + 刹车（刹车同样斜坡，避免点头）
            self.throttle = max(0.0, self.throttle - 2.0 * dt)
            brake_cmd = min(1.0, -a_des / 5.0)
            self.brake += max(-0.12, min(0.12, brake_cmd - self.brake))
            self.brake = max(0.0, min(1.0, self.brake))
            self._plateau = 0.0
            return self.throttle, self.brake

        self.brake = max(0.0, self.brake - 0.15)

        # 4) 加速度误差积分油门（PI 的 I 项事实上就是这里的自适应）
        self.throttle += 0.05 * (a_des - self._a_meas) * dt * 10.0
        self.throttle = max(0.0, min(self.throttle_cap, self.throttle))

        # 5) 低挡位检测：给足了油门但速度几乎不动，就顶一次换挡
        if allow_kick and v < UPSHIFT_SPEED and v_cmd > UPSHIFT_SPEED + 1.0:
            dv = 0.0 if v_last is None else abs(v - v_last)
            if dv < 0.05 and self.throttle > self.throttle_cap * 0.8:
                self._plateau += dt
            else:
                self._plateau = max(0.0, self._plateau - dt)
            if self._plateau > 0.9:
                self._kick = 0.7
                self._plateau = 0.0
        else:
            self._plateau = 0.0

        return self.throttle, self.brake


class StraightDriver:
    """直行场景驱动：纯跟踪转向 + 梯形速度规划 + 纵向闭环。

    cruise 不为 None 时走「定油门巡航」：油门只做单调斜坡，不做速度闭环。
    原因是仿真对油门的响应有滞后，闭环在 30 Hz 下容易在 0.45/0/0.17 之间抖，
    实测会把加速度顶到 19 m/s²，直接丢掉 |a| ≤ 5 的 10 分。
    """

    def __init__(self, route, v_max: float = 10.0, accel_limit: float = 2.0,
                 steer_sign: float = -1.0, lookahead_min: float = 4.0,
                 lookahead_max: float = 14.0, allow_kick: bool = True,
                 cruise: float | None = None, cruise_rate: float = 0.6):
        self.path = RoutePath(route)
        self.planner = SpeedPlanner(v_max=v_max, a_up=accel_limit, a_down=accel_limit)
        self.long = LongController(accel_limit=accel_limit)
        self.steer_sign = float(steer_sign)
        self.la_min = float(lookahead_min)
        self.la_max = float(lookahead_max)
        self.allow_kick = bool(allow_kick)
        self.cruise = None if cruise is None else float(cruise)
        self.cruise_rate = float(cruise_rate)
        self._thr = 0.0

    def steering(self, x: float, y: float, yaw_deg: float, v: float) -> float:
        if self.path.n < 2:
            return 0.0
        s, _, _ = self.path.project(x, y)
        # 前视距离随速度增长：低速看得近一点，避免画龙
        la = max(self.la_min, min(self.la_max, 0.9 * abs(v) + 4.0))
        tx, ty = self.path.point_at(s + la)

        dx, dy = tx - x, ty - y
        dist = max(math.hypot(dx, dy), 1e-3)

        # 场景 yaw 顺时针为正，先转成数学角
        yaw = math.radians(-yaw_deg)
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

        curvature = 2.0 * local_y / (dist * dist)
        delta = math.atan(curvature * WHEELBASE_M)
        if local_x < 0.0:
            delta = MAX_STEER_RAD if local_y > 0 else -MAX_STEER_RAD

        return max(-1.0, min(1.0, self.steer_sign * delta / MAX_STEER_RAD))

    def step(self, x: float, y: float, yaw_deg: float, v: float, dt: float):
        """返回 (VehicleControl, 附加信息 dict)。"""
        s, _, lat = self.path.project(x, y)
        s_left = max(0.0, self.path.total - s)

        if self.cruise is not None:
            # 定油门巡航：油门只朝目标值单调靠拢，永不来回抖
            dt = max(dt, 1e-3)
            delta = max(-self.cruise_rate * dt, min(self.cruise_rate * dt, self.cruise - self._thr))
            self._thr = max(0.0, min(1.0, self._thr + delta))
            steer = self.steering(x, y, yaw_deg, v)
            vc = VehicleControl(
                gear=GearMode.DRIVE,
                throttle=float(self._thr),
                brake=0.0,
                steering=float(steer),
            )
            return vc, {"s": s, "s_left": s_left, "lat": lat, "v_cmd": self.cruise}

        v_cmd = self.planner.target(v, s_left, dt)
        throttle, brake = self.long.update(v, v_cmd, dt, allow_kick=self.allow_kick)
        steer = self.steering(x, y, yaw_deg, v)

        vc = VehicleControl(
            gear=GearMode.DRIVE,
            throttle=float(throttle),
            brake=float(brake),
            steering=float(steer),
        )
        info = {
            "s": s,
            "s_left": s_left,
            "lat": lat,
            "v_cmd": v_cmd,
        }
        return vc, info
