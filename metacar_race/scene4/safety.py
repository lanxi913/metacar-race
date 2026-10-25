"""安全层：信号灯闸门（含"绿灯够不够过线"的 ETA 判断）、终点驻车。

信号灯这一块修掉的是 cont-r2 里那个 −39.66 分：老代码把 `light` 与 `stop_line`
**只取一次就缓存**，13 秒里 `remain` 一直报 4.1、`state` 一直 GREEN，车以 8 m/s
在绿灯只剩 4.1 s 时距停止线 58 m（需 7.1 s）自然会闯红。现在：

* 每帧按 `light.id` 从当帧 `traffic_light_groups` 重新取状态与剩余时间；
* 绿灯也要算 `t_line = gap / v`，`t_line + margin > remain` 就按"过不去"处理；
* 红灯/黄灯的收速距离按 `max(LIGHT_STOP_DIST_M, v²/(2·DECEL) + 2)` 自适应，
  不再写死 30 m；
* 已经贴到线前（< `LIGHT_STOP_GAP_M + TOL`）就停住，不越线。

这一层只产出"允许车速"（`v_cap`），不直接下发油门——纵向仍归 driver 的速度环。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from metacar.models import TrafficLightState

from . import config as C
from .track import find_light_by_id, pick_light, pick_stop_line


@dataclass
class LightDecision:
    """一帧的信号灯结论。"""

    v_cap: float | None = None      #: 允许车速（None = 本帧不限制）
    stop_gap: float | None = None   #: 到停止线的距离
    state: str | None = None        #: RED / GREEN / YELLOW / None
    remain: float | None = None     #: 剩余时间（s）
    holding: bool = False           #: 是否停在停止线前
    passed: bool = False            #: 是否已经过线
    events: list[str] = field(default_factory=list)


class LightGate:
    """信号灯 + 停止线闸门：每帧刷新，输出允许车速。"""

    def __init__(self, enabled: bool = True, log=None) -> None:
        self.enabled = bool(enabled)
        self.log = log
        self.reset()

    def reset(self) -> None:
        self.light_id: str | None = None
        self.stop_line: tuple[float, float] | None = None
        self.passed = False
        self.holding = False

    # ------------------------------------------------------------------ 内部
    def _refresh_light(self, groups):
        """按 id 重新取灯；第一次见到就记下 id。"""
        if self.light_id is not None:
            tl = find_light_by_id(groups, self.light_id)
            if tl is not None:
                return tl
            self.light_id = None
        return None

    @staticmethod
    def _straight(light) -> tuple[TrafficLightState, float]:
        return light.straight_state, float(light.straight_remaining_time)

    def update(self, *, groups, roads, x: float, y: float, yaw: float,
               speed: float) -> LightDecision:
        d = LightDecision()
        if not self.enabled or self.passed:
            d.passed = self.passed
            return d

        light = self._refresh_light(groups)
        if light is None:
            light = pick_light(groups, x, y, yaw)
            if light is None:
                return d
            self.light_id = light.id
            d.events.append("light_found")
            if self.log:
                self.log(f"[light] 找到 {light.id} 前方 "
                         f"{math.hypot(float(light.position.x) - x, float(light.position.y) - y):.1f} m")

        if self.stop_line is None:
            got = pick_stop_line(roads, x, y, yaw)
            if got is not None:
                self.stop_line = got[1]
                d.events.append("stop_line_found")
                if self.log:
                    self.log(f"[light] 停止线 ({self.stop_line[0]:.1f},{self.stop_line[1]:.1f})"
                             f" 距 {got[0]:.1f} m")

        if self.stop_line is None:
            return d

        gap = math.hypot(self.stop_line[0] - x, self.stop_line[1] - y)
        d.stop_gap = gap
        state, remain = self._straight(light)
        d.state, d.remain = state.name, remain

        if gap < 0.8:
            self.passed = True
            d.passed = True
            d.events.append("stop_line_passed")
            return d

        v = max(abs(float(speed)), 0.0)
        stop_at = C.LIGHT_STOP_GAP_M + C.LIGHT_STOP_TOL_M
        # 允许车速：从当前位置以 LIGHT_DECEL 减速，到 stop_at 处刚好停住
        v_allow = math.sqrt(2.0 * C.LIGHT_DECEL * max(gap - stop_at, 0.0))

        can_stop = gap > stop_at
        if state in (TrafficLightState.RED, TrafficLightState.YELLOW):
            if can_stop:
                d.v_cap = v_allow
                if gap <= stop_at + 0.05:
                    d.holding = True
            else:
                d.v_cap = 0.0
                d.holding = True
        else:
            # 绿灯：够不够过线？到线时间 + 余量 > 剩余时间就按"过不去"处理
            t_line = gap / max(v, 0.5)
            if t_line + C.LIGHT_ETA_MARGIN_S > remain:
                if gap > stop_at:
                    d.v_cap = v_allow
                # 已经贴线太近就冲过去：总比停在路口中间好
        if d.events and self.log:
            for ev in d.events:
                self.log(f"[light] {ev} gap={gap:.2f} v={v:.2f} {state.name} "
                         f"remain={remain:.1f} cap={d.v_cap}")
        return d


class EndPark:
    """到终点后驻车。

    "拿 +10 模式"（`brk_cap ≤ 0.001`）下不许一脚刹车（4.8→0 只用 0.4 s ≈ −12 m/s²
    会被记一次超限），先滑行到快停住再挂驻车挡。
    """

    def __init__(self, hold_seconds: float = C.HOLD_END_S,
                 coast_only: bool = False) -> None:
        self.hold_seconds = float(hold_seconds)
        self.coast_only = bool(coast_only)
        self.hold_t = 0.0
        self.active = False

    def reset(self) -> None:
        self.hold_t = 0.0
        self.active = False

    def update(self, speed: float, dt: float) -> tuple[bool, float, float, bool]:
        """返回 (是否驻车挡, throttle, brake, 是否该结束本轮)。"""
        self.hold_t += dt
        if self.coast_only and abs(speed) > 0.30:
            brake, throttle, parking = 0.0, 0.0, False
        else:
            brake, throttle, parking = 1.0, 0.0, True
        done = self.hold_t >= self.hold_seconds
        return parking, throttle, brake, done
