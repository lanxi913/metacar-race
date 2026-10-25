"""纵向：加速度治理（收油）、指令限幅、油门上升限幅。

这一层的结论全部来自 `accel_window_probe/` 的 18 局直行实测，改之前请先读那份
`FINDINGS.md`：

* 平台判的是**指令加速度** `a_cmd = A·throttle − B·brake`，要全过程 |a_cmd| ≤ 5；
* 油门 0.50/0.55/0.60 恒定 + 零刹车 → 拿 +10；油门 1.00 → 扣分；
* 刹车 0.05/0.10/0.20/0.30 只要给过 → 扣分（哪怕实测车速一点没掉）；
* 所以"用刹车压加速度"和"拿 +10"在数学上不可兼得 → 减速只能**提前收油**。

`soft_start`（油门上升率限幅）默认关：实测只能降峰、拿不到加速度分，
但它在起步抖动大的场景里有用，所以留着开关。
"""

from __future__ import annotations

import math
from collections import deque

from . import config as C


class SoftStart:
    """油门上升率限幅（可选）。

    语义：`cap` 只允许上涨 `slew`/帧，但**允许瞬间下跌**（安全优先：前方有车要
    急刹时 throttle 归零不能被限幅挡住）。来源：`vertical_park.py` 的 `_smooth()`。
    """

    def __init__(self, slew: float = C.SOFT_START_SLEW, enabled: bool = True) -> None:
        self.slew = float(slew)
        self.enabled = bool(enabled)
        self.cap = 0.0

    def reset(self) -> None:
        self.cap = 0.0

    def apply(self, throttle: float) -> float:
        if not self.enabled:
            return throttle
        if throttle > self.cap:
            self.cap = min(throttle, self.cap + self.slew)
        else:
            self.cap = throttle          # 允许瞬间松油/归零
        return min(throttle, self.cap)


class AccelGovernor:
    """把"超限"翻译成**收油 + 提前降目标车速**，绝不加刹车（方案 B）。

    做三件事：
      1. 1 s 窗平均加速度超过目标 → 上调 `v_trim`（提前降目标车速），并把本帧
         油门压到 `coast_cut` 以下（滑行），下一帧由速度环自然收油；
      2. 低于目标 1.0 以上 → `v_trim` 回调，避免越治理越慢；
      3. 不产出刹车（进来的刹车原样返回），最终由 `limit_command()` 统一封顶。
    """

    def __init__(self, target: float = C.ACCEL_GOV_TARGET,
                 window: float = C.ACCEL_GOV_WIN_S,
                 step: float = C.ACCEL_GOV_STEP,
                 ff: float = C.ACCEL_GOV_FF,
                 ff_gain: float = C.ACCEL_GOV_FF_GAIN,
                 brake_max: float = C.ACCEL_GOV_BRAKE_MAX,
                 ff_throttle: float = C.ACCEL_GOV_FF_THROTTLE,
                 ff_demand: float = C.ACCEL_GOV_FF_DEMAND,
                 hard_throttle: float = C.ACCEL_GOV_HARD_THROTTLE,
                 coast_cut: float = C.COAST_THROTTLE_CUT,
                 enabled: bool = True) -> None:
        self.target = float(target)
        self.window = float(window)
        self.step = float(step)
        self.ff = float(ff)
        self.ff_gain = float(ff_gain)
        self.brake_max = float(brake_max)
        self.ff_throttle = float(ff_throttle)
        self.ff_demand = float(ff_demand)
        self.hard_throttle = float(hard_throttle)
        self.coast_cut = float(coast_cut)
        self.enabled = bool(enabled)
        self.trim = 0.0
        self.v_trim = 0.0
        self.hist: deque[tuple[float, float]] = deque()

    def reset(self) -> None:
        self.trim = 0.0
        self.v_trim = 0.0
        self.hist.clear()

    def accrued(self, throttle: float = 0.0) -> float:
        """方案 B 下不再前馈刹车：保留接口，恒返回 0。"""
        return 0.0

    def apply_trim(self, v_target: float) -> float:
        """把 `v_trim` 作用到目标车速上（调用点在速度环之前）。"""
        if not self.enabled:
            return v_target
        return max(0.3, float(v_target) - self.v_trim)

    def update(self, t: float, speed: float, v_target: float,
               throttle: float, brake: float) -> tuple[float, float]:
        """返回 (throttle, brake)。超限时只降目标车速 + 压油门。"""
        if not self.enabled:
            return throttle, brake
        v = abs(speed)

        # 只在"油门在踩、且目标车速还比实际高"时治理：巡航与刹车都不动。
        demand = float(v_target) - v
        accelerating = (demand > self.ff_demand) or (throttle > self.hard_throttle)
        if throttle <= self.ff_throttle or not accelerating:
            self.trim = max(C.ACCEL_GOV_TRIM_MIN, self.trim - self.step)
            self.v_trim = max(0.0, self.v_trim - C.COAST_TRIM_STEP)
            self.hist.clear()
            return throttle, brake

        self.hist.append((t, v))
        while self.hist and t - self.hist[0][0] > self.window:
            self.hist.popleft()

        if len(self.hist) >= 2 and t - self.hist[0][0] >= self.window * 0.8:
            t0, v0 = self.hist[0]
            a = (v - v0) / max(1e-3, t - t0)
            if a > self.target:
                self.v_trim = min(C.COAST_TRIM_MAX, self.v_trim + C.COAST_TRIM_STEP)
                throttle = min(throttle, self.coast_cut)
            elif a < self.target - 1.0:
                self.v_trim = max(0.0, self.v_trim - C.COAST_TRIM_STEP)
        self.trim = self.v_trim
        return throttle, brake


def limit_command(throttle: float, brake: float, a_prev: float | None = None,
                  thr_cap: float = C.THROTTLE_CAP_BONUS,
                  brk_cap: float = C.BRAKE_CAP_BONUS,
                  a_limit: float = C.ACCEL_CMD_LIMIT,
                  slew: float = C.ACCEL_CMD_SLEW,
                  enabled: bool = True) -> tuple[float, float, float]:
    """把 (throttle, brake) 夹进指令加速度预算，返回 (throttle, brake, a_cmd)。

    口径：`a_cmd = A·throttle − B·brake`，平台要求全过程 |a_cmd| ≤ 5。这里同时封住
    三个已知边界：油门 ≤ `thr_cap`、刹车 ≤ `brk_cap`、|a_cmd| ≤ `a_limit`，且
    **上升**斜率不超过 `slew`/帧（下跌不限，安全优先）。

    急停（`brake ≥ 0.30`）与驻车不走这里，由调用点绕开：安全永远优先于 +10。
    """
    if not enabled:
        return throttle, brake, C.ACCEL_CMD_A * throttle - C.ACCEL_CMD_B * brake
    throttle = max(0.0, min(thr_cap, float(throttle)))
    brake = max(0.0, min(brk_cap, float(brake)))
    a = C.ACCEL_CMD_A * throttle - C.ACCEL_CMD_B * brake
    if a > a_limit:
        throttle = min(throttle, a_limit / C.ACCEL_CMD_A)
        a = C.ACCEL_CMD_A * throttle - C.ACCEL_CMD_B * brake
    elif a < -a_limit:
        brake = min(brake, a_limit / C.ACCEL_CMD_B)
        a = C.ACCEL_CMD_A * throttle - C.ACCEL_CMD_B * brake
    if a_prev is not None and slew > 0.0 and a > float(a_prev) + slew:
        a = float(a_prev) + slew
        throttle = max(0.0, min(thr_cap, a / C.ACCEL_CMD_A))
    return throttle, brake, a


def accel_of(throttle: float, brake: float) -> float:
    """按 13 局拟合口径换算指令加速度（日志/自检用）。"""
    return C.ACCEL_CMD_A * float(throttle) - C.ACCEL_CMD_B * float(brake)


def speed_allowance(gap_m: float, decel: float = C.LIGHT_DECEL) -> float:
    """从当前距离开始以 `decel` 减速，到停止线时刚好停住所允许的车速。"""
    return math.sqrt(2.0 * max(0.0, decel) * max(0.0, float(gap_m)))


class StraightRun:
    """长直道的纵向规划：油门从 0 渐加，按滑行减速度反推松油点。

    参数
    ----
    v_cap : float
        直道上允许的最高车速（一般取 `--vmax`）。
    v_end : float
        终点（下一个弯入口 / 元素终点）的目标车速，**不为 0**：车不停，直接
        把控制权交给下一个元素。
    a_coast : float
        实测松油减速度。允许车速按 `sqrt(v_end² + 2·a_coast·d)` 反推：d 是到
        终点的距离，所以只要速度不超过这条曲线，早松油就能自己滑到 v_end，
    全程不用刹车。

    行为
    ----
    * `speed > v_allow + COAST_MARGIN_MS` → 松油滑行（throttle=brake=0）；
    * 否则油门按 `THROTTLE_RAMP_PER_S` 从 0 单调爬升到 `thr_max`，**不回落**
      （回落由滑行负责），避免开关型油门反复抖；
    * 速度到了 `v_allow` 就只保持油门不再加（不刹车）。
    """

    def __init__(self, v_cap: float = 8.0, v_end: float = C.ELEMENT_END_SPEED_MS,
                 a_coast: float = C.COAST_DECEL_MS2, ramp: float = C.THROTTLE_RAMP_PER_S,
                 thr_max: float = C.STRAIGHT_THROTTLE) -> None:
        self.v_cap = max(0.1, float(v_cap))
        self.v_end = max(0.0, float(v_end))
        self.a_coast = max(0.05, float(a_coast))
        self.ramp = max(0.0, float(ramp))
        self.thr_max = max(0.0, min(1.0, float(thr_max)))
        self.reset()

    def reset(self) -> None:
        self.thr = 0.0
        self.mode = "accel"

    # ------------------------------------------------------------------ 计算
    def allow_speed(self, dist_m: float) -> float:
        """到终点距离 `dist_m` 时允许的最高车速（保证松油能滑到 v_end）。"""
        d = max(0.0, float(dist_m))
        return min(self.v_cap, math.sqrt(self.v_end ** 2 + 2.0 * self.a_coast * d))

    def update(self, dt: float, speed: float, dist_m: float
               ) -> tuple[float, float, bool, str]:
        """→ (throttle, brake, coast, note)。brake 恒为 0：减速只靠松油。"""
        dt = max(1e-3, float(dt))
        v_now = abs(float(speed))
        v_allow = self.allow_speed(dist_m)

        if v_now > v_allow + C.COAST_MARGIN_MS:
            self.thr = 0.0
            self.mode = "coast"
            return 0.0, 0.0, True, f"coast v={v_now:.2f}>{v_allow:.2f}"

        if v_now >= v_allow:                      # 到线了：保持，不加也不刹
            self.mode = "hold"
            return self.thr, 0.0, False, f"hold v={v_now:.2f}~{v_allow:.2f}"

        self.thr = min(self.thr_max, self.thr + self.ramp * dt)
        self.mode = "accel"
        return self.thr, 0.0, False, f"accel v={v_now:.2f}/{v_allow:.2f} thr={self.thr:.2f}"
