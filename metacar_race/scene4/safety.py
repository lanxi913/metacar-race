"""安全层：信号灯闸门（等红转绿）+ 终点驻车。

信号灯这一块修过两次：

1. cont-r2 的 −39.66：老代码把 `light` 与 `stop_line` **只取一次就缓存**，
   13 秒里 `remain` 一直报 4.1、`state` 一直 GREEN，车以 8 m/s 在绿灯只剩 4.1 s
   时距停止线 58 m 自然会闯红。现在每帧按 `light.id` 重新取状态与剩余时间。
2. r14 的 −42.46：元素 5 一公里内有 5 条停止线，老实现只看最近一条就
   `passed=True` 永久关闸。现在把 `roads[*].stop_line` 全部投影成队列，过线自动换下一条。

**当前口径（用户 m04149）**：不再判断「车速够不够过线」，只认一个状态跳变——
灯从非绿变绿的那一帧立刻放行、立即起步；之前的减速全靠「松油滑行」，贴线才给刹车。

这一层只产出"允许车速"（`v_cap`）与等待/放行标志，不直接下发油门。
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
    waiting: bool = False           #: 已决定在路口等下一次绿灯（未被释放）
    released: bool = False          #: 本帧从「等」切到「走」（红转绿那一帧）
    passed: bool = False            #: 是否已经过线
    events: list[str] = field(default_factory=list)


class LightGate:
    """信号灯 + 停止线闸门：每帧刷新，输出允许车速。

    2026-10-25（r14 复盘）：元素 5「十字路口通行」全段有 **5 条停止线**
    （s≈1923.4 / 1935.4 / 1943.4 / 1951.4 / 1963.4），老实现只捡最近的一条就
    `passed=True` 永久关闸，后面三个路口根本没看灯 —— r14 就是这么在出口
    「绿灯只剩 3 s」时冲过去吃了 −42.46。现在改成：

    * 启动时把 `roads[*].stop_line` 全部投影到路线里程，去重排序成一条队列；
    * 每帧用 `track.s_of_world()` 取当前里程，**永远盯住前方最近的那条线**，
      过线后自动换下一条（不再一次性 `passed`）；
    * 每条线配一盏灯：取「投影里程与这条线相距 ≤ `LIGHT_MATCH_M`」的灯里最近的那盏，
      用它的 `straight_state / straight_remaining_time` 判 ETA。
    """

    def __init__(self, enabled: bool = True, log=None, track=None) -> None:
        self.enabled = bool(enabled)
        self.log = log
        self.track = track
        self.reset()

    def reset(self) -> None:
        self.light_id: str | None = None
        self.stop_line: tuple[float, float] | None = None
        self.passed = False
        self.holding = False
        # 「决定停、等红转绿」的状态（m04041 方案）
        self._wait = False
        self._saw_not_green = False
        self._wait_frames = 0
        # 「已承诺通过某个路口组」的状态（m04276 方案 A）
        self._committed: tuple[float, float] | None = None
        # 静态几何只建一次（`reset()` 会被元素切换调用，别把它清掉）
        self.lines: list[float] | None = getattr(self, "lines", None)
        self._groups_cache: list[tuple[float, float]] | None = None
        self._last_note: str = ""

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

    # ---------------------------------------------------------------- 停止线队列
    def _build_lines(self, roads) -> None:
        """把所有停止线投影到路线里程，去重排序。失败就留 None（退回单线逻辑）。"""
        self.lines = None
        if self.track is None:
            return
        s_list: list[float] = []
        try:
            for rd in roads or []:
                for pt in (getattr(rd, "stop_line", None) or []):
                    px = float(getattr(pt, "x", pt[0] if isinstance(pt, (list, tuple)) else 0.0))
                    py = float(getattr(pt, "y", pt[1] if isinstance(pt, (list, tuple)) else 0.0))
                    s_list.append(float(self.track.s_of_world(px, py)))
        except Exception as exc:  # noqa: BLE001
            if self.log:
                self.log(f"[light] 停止线投影失败，退回单线逻辑: {exc!r}")
            self.lines = None
            return
        s_list.sort()
        merged: list[float] = []
        for s in s_list:
            if not merged or s - merged[-1] > C.LIGHT_LINE_DEDUP_M:
                merged.append(s)
        self.lines = merged
        self._groups_cache = None
        if self.log and merged:
            self.log("[light] 停止线队列 %d 条：%s"
                     % (len(merged), ", ".join("%.1f" % s for s in merged)))

    def _groups(self) -> list[tuple[float, float]]:
        """把里程上相邻 ≤ `LIGHT_GROUP_CHAIN_M` 的停止线聚成一个路口组。

        元素 5 的 5 条线只隔 8~20 m，本来就是同一个十字路口的几条停止线；
        每条线各配一盏灯，所以「过第一条线」不等于「过路口」——r20 的教训是
        放行后 2.4 s 就撞上第二条线的红灯（483 帧 `|a|≥5`、峰值 18.6）。
        组内第一次见到绿灯就承诺通过，组内不再看灯（用户 m04276 方案 A）。
        """
        if self._groups_cache is not None:
            return self._groups_cache
        out: list[tuple[float, float]] = []
        for s in self.lines or []:
            s = float(s)
            if out and s - out[-1][1] <= C.LIGHT_GROUP_CHAIN_M:
                out[-1] = (out[-1][0], s)
            else:
                out.append((s, s))
        self._groups_cache = out
        return out

    def _light_for_line(self, groups, s_line: float):
        """给一条停止线找灯：取投影里程最接近且 <= LIGHT_MATCH_M 的那盏。"""
        best = None
        best_d = 1e9
        for grp in groups or []:
            for tl in (getattr(grp, "traffic_lights", None) or []):
                pos = getattr(tl, "position", None)
                if pos is None or self.track is None:
                    continue
                s_l = float(self.track.s_of_world(float(pos.x), float(pos.y)))
                dd = abs(s_l - s_line)
                if dd < best_d:
                    best_d, best = dd, tl
        if best is not None and best_d <= C.LIGHT_MATCH_M:
            return best
        return None

    def _line_index(self, s_now: float) -> int | None:
        """前方最近的那条线（bisect，无状态）。"""
        if not self.lines:
            return None
        import bisect
        i = bisect.bisect_right(self.lines, s_now + 0.30)
        return i if i < len(self.lines) else None

    def update(self, *, groups, roads, x: float, y: float, yaw: float,
               speed: float) -> LightDecision:
        d = LightDecision()
        if not self.enabled:
            return d

        # 首次：把全部停止线投影成队列（一次性）
        if self.lines is None and self.track is not None:
            self._build_lines(roads)

        if self.track is not None and self.lines:
            return self._update_queue(d, groups=groups, x=x, y=y, speed=speed)

        # ---- 退回：没有 track 时的单线逻辑（保持旧行为） ---------------------
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

    def _update_queue(self, d: LightDecision, *, groups, x: float, y: float,
                      speed: float) -> LightDecision:
        """路口组：非绿→绿就起步，看到绿灯就承诺通过整组（m04276 方案 A+B）。

        规则（按用户原话落地）：

        * 「不要判断车速能否过十字路口」——绿灯那一帧直接承诺通过，不算 ETA；
        * 「一旦看到非绿到绿的转变就起步」→ `released`，driver 当帧重新起步；
        * 「不要判断中间能否通过」→ 承诺后组内所有停止线一律不看灯、不减速；
        * 进入 `LIGHT_DECIDE_M`（130 m）后先把巡航压到 `LIGHT_APPROACH_MS`，
          给「红灯突然跳」留滑行余量（r20 是 10 m 处才发现红，−11 m/s² 急刹）；
        * 停止线前 `LIGHT_STOP_AT_M` 就是终点（斑马线读不到，用常量标定）。

        r19 的教训（已删除）：算「整组 5 条线 43 m 要走 43.5 s 绿灯」，
        结果绿灯周期永远不够，车停在线前 2 m 一直到场景结束都没放行。
        """
        s_now = float(self.track.s_of_world(x, y))
        stop_at = float(C.LIGHT_STOP_AT_M)

        # ① 已承诺通过某个路口组：组内不再干预（不看灯、不限速），一直走到组尾
        if self._committed is not None:
            g0, g1 = self._committed
            if s_now <= g1 + 1.0:
                d.stop_gap = g1 - s_now
                return d
            self._committed = None

        # ② 找当前关心的路口组：组尾还没走完、组首已进入决策距离
        target: tuple[float, float] | None = None
        for g0, g1 in self._groups():
            if g1 < s_now - C.LIGHT_MATCH_M:     # 整组都过去了
                continue
            if g0 - s_now > C.LIGHT_DECIDE_M:    # 还太远，不预先减速
                break
            target = (g0, g1)
            break
        if target is None:
            self._wait = False
            self._saw_not_green = False
            return d

        g0, g1 = target
        gap = g0 - s_now
        d.stop_gap = gap

        light = self._light_for_line(groups, g0)
        if light is None:
            self._wait = False
            return d
        state, remain = self._straight(light)
        d.state, d.remain = state.name, remain

        # ③ 方案 B：进决策距离后、压线之前，先把巡航压下来
        if gap > stop_at + 0.05:
            cap = float(C.LIGHT_APPROACH_MS)
            d.v_cap = cap if d.v_cap is None else min(float(d.v_cap), cap)

        # ④ 绿灯（含「非绿→绿」那一帧）：够近就承诺通过整组，组内不再看灯
        if state == TrafficLightState.GREEN:
            if self._wait:
                waited = self._wait_frames / 30.0
                self._wait = False
                self._saw_not_green = False
                self._wait_frames = 0
                d.released = True
                d.events.append("light_go")
                if self.log:
                    self.log("[light] 红转绿 → 立即起步（距线 %.1f m，等了 %.1f s；"
                             "承诺 %d 条线 %.1f~%.1f m 一次通过）"
                             % (gap, waited, self._group_count(g0, g1), g0, g1))
            if gap <= float(C.LIGHT_COMMIT_M):
                # 只有贴着组首才锁定整组；130 m 外见到绿灯不锁，避免半路变红硬闯
                self._committed = (g0, g1)
            return d

        # ⑤ 非绿：松油滑行到线前停住等绿（不判断能否通过）
        if not self._wait:
            self._wait = True
            self._wait_frames = 0
            d.events.append("light_wait")
            if self.log:
                self.log("[light] 距线 %.1f m 非绿（%s 剩 %.1f s）→ 松油滑行，"
                         "线前 %.1f m 停住等绿"
                         % (gap, state.name, remain, stop_at))
        self._saw_not_green = True
        self._wait_frames += 1
        d.waiting = True

        if self._wait_frames > C.LIGHT_WAIT_MAX_S * 30.0:
            # 兜底：等了 LIGHT_WAIT_MAX_S 还没见绿灯，按绿灯走，免得原地卡死
            self._wait = False
            self._wait_frames = 0
            d.released = True
            self._committed = (g0, g1)
            d.events.append("light_go")
            if self.log:
                self.log("[light] 等灯超过 %.0f s 仍非绿，兜底放行" % C.LIGHT_WAIT_MAX_S)
            return d

        if gap > stop_at + 0.05:
            # 允许车速曲线：越靠近线越低；driver 用松油实现，不踩刹车。
            # 和方案 B 的接近巡航取更紧的那个（上面已经压到 LIGHT_APPROACH_MS）。
            slack = math.sqrt(2.0 * C.LIGHT_COAST_DECEL_MS2 * max(gap - stop_at, 0.0))
            d.v_cap = slack if d.v_cap is None else min(float(d.v_cap), slack)
        else:
            d.v_cap = 0.0
            d.holding = True
        return d

    def _group_count(self, g0: float, g1: float) -> int:
        """这个路口组里有几条停止线（只用于日志）。"""
        return sum(1 for s in (self.lines or []) if g0 - 1e-6 <= float(s) <= g1 + 1e-6)


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
