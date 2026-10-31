"""元素路由：把里程切成元素段，按名字挑策略，异常就永久降级。

三条硬约束（都是踩过坑的）：

1. **单元素出错不许杀整局**：策略构造或执行抛异常 → 该元素进 `disabled`，
   以后一律走基线（`--no-elements` 等价）。
2. **瞬移必须复位**：平台把车拽回元素起点时，策略里持有的跟踪器/状态机还在原地
   （full-r1 里瞬移后 `steer=0.000`、横向误差涨到 +13.2 m 直接开出路面）。
3. **起步段是合成的**：`static.sub_scenes` 第一个元素从 s=8.2 m 才开始，前 8 m
   走基线会先给一脚刹车把 +10 丢掉，所以在 s=0 处插一段「起步直行」。
"""

from __future__ import annotations

from dataclasses import dataclass

from . import config as C
from .strategies import (Command, ElementContext, STRATEGY_BY_KIND, Strategy,
                         cumulative, kind_of, slice_route)


@dataclass
class Span:
    """一个元素在路线上的里程段。"""

    index: int
    name: str
    s_start: float
    s_end: float
    synthetic: bool = False


class ElementRouter:
    """里程 → 元素段 → 策略。"""

    def __init__(self, static, route, track, *,
                 vmax: float = 8.0, ego_length: float = 4.70,
                 ego_width: float = 1.90, lane_width: float = 3.50,
                 only: tuple[str, ...] | None = None, log=None,
                 overrides: dict[str, str] | None = None) -> None:
        self.static = static
        self.route = route
        self.track = track
        self.vmax = float(vmax)
        self.ego_length = float(ego_length)
        self.ego_width = float(ego_width)
        self.lane_width = float(lane_width)
        self.only = tuple(only or ())
        self.overrides = dict(overrides or {})   #: 元素名片段 → 策略种类（如 {"车辆直行": "lane"}）
        self.log = log or (lambda _m: None)
        self.cum = cumulative(route)

        spans: list[Span] = []
        for k, sc in enumerate(getattr(static, "sub_scenes", None) or [], start=1):
            a = (0.0 if sc.start_point is None
                 else float(track.s_of_world(float(sc.start_point.x), float(sc.start_point.y))))
            b = (float(track.length) if sc.end_point is None
                 else float(track.s_of_world(float(sc.end_point.x), float(sc.end_point.y))))
            spans.append(Span(k, sc.name, a, b))
        spans.sort(key=lambda sp: sp.s_start)
        if spans and spans[0].s_start > C.START_SPAN_MIN_M:
            spans.insert(0, Span(0, C.START_SPAN_NAME, 0.0, spans[0].s_start, synthetic=True))
        self.spans = spans

        self.current: Span | None = None
        self.strategy: Strategy | None = None
        self.disabled: set[str] = set()
        self.unmatched: set[str] = set()
        self.frames_owned = 0
        self.last_log_t = -99.0

    # ------------------------------------------------------------------ 查询
    def span_at(self, s_now: float) -> Span | None:
        """先按严格包含找；落在缝里就取"已经开始过"的最后一段。"""
        best = None
        for sp in self.spans:
            if sp.s_start - 1.0 <= s_now <= sp.s_end + 1.0:
                return sp
            if s_now >= sp.s_start and (best is None or sp.s_start > best.s_start):
                best = sp
        return best

    def _allowed(self, span: Span) -> bool:
        if not self.only:
            return True
        return any(key in span.name for key in self.only)

    def _build(self, span: Span, ctx: ElementContext) -> Strategy | None:
        kind = ""
        for key, val in self.overrides.items():
            if key and key in span.name:
                kind = val
                break
        if not kind:
            kind = kind_of(span.name)
        cls = STRATEGY_BY_KIND.get(kind)
        if cls is None:
            # 正式赛题的子场景名可能换一种叫法（"直线行驶" 而不是 "直行"），
            # kind_of 的关键字匹配就落空，该元素整段退回基线：一挡 4.78 m/s 跑
            # 1000 m 级元素必然撞时限，而且日志里只有一行"进入元素"，很难发现。
            if span.name not in self.unmatched:
                self.unmatched.add(span.name)
                self.log(f"[elem] '{span.name}' 没匹配到策略关键字，整段走基线"
                         f"（新场景要在 strategies.kind_of 里补映射）")
            return None
        if span.name in self.disabled:
            return None
        route = slice_route(self.route, self.cum, span.s_start, span.s_end)
        try:
            strat = cls(ctx, route, log=self.log)
        except Exception as exc:  # noqa: BLE001
            self.disabled.add(span.name)
            self.log(f"[elem] {span.name} 策略构造失败，永久降级基线: {exc!r}")
            return None
        return strat

    # ------------------------------------------------------------------ 主入口
    def step(self, ctx: ElementContext) -> Command | None:
        span = self.span_at(ctx.s_now)
        if span is None:
            return None
        # 元素边界回填给策略（直道规划要用 s_end 算到终点的距离）
        ctx.s_start = float(span.s_start)
        ctx.s_end = float(span.s_end)
        if span is not self.current:
            self.current = span
            self.strategy = None
            self.log(f"[elem] 进入元素 {span.name} "
                     f"({span.s_start:.1f}~{span.s_end:.1f} m)")
        if not self._allowed(span):
            return None
        if self.strategy is None:
            self.strategy = self._build(span, ctx)
            if self.strategy is None:
                return None
            self.strategy.sync(ctx.x, ctx.y)
        try:
            cmd = self.strategy.step(ctx)
        except Exception as exc:  # noqa: BLE001
            self.disabled.add(span.name)
            self.strategy = None
            self.log(f"[elem] {span.name} 执行异常，永久降级基线: {exc!r}")
            return None
        if cmd is None:
            return None
        self.frames_owned += 1
        if ctx.sim_t - self.last_log_t >= 5.0:
            self.last_log_t = ctx.sim_t
            self.log(f"[elem] 接管 {cmd.note or span.name} "
                     f"steer={'--' if cmd.steer is None else '%+.3f' % cmd.steer} "
                     f"thr={'--' if cmd.throttle is None else '%.2f' % cmd.throttle} "
                     f"brk={'--' if cmd.brake is None else '%.2f' % cmd.brake} "
                     f"v_cap={'--' if cmd.v_cap is None else '%.2f' % cmd.v_cap}")
        return cmd

    # ------------------------------------------------------------------ 查询
    def kind(self) -> str:
        """当前元素被哪个策略接管（"" = 走基线）。driver 用它决定要不要开基线超车。"""
        if self.current is None:
            return ""
        try:
            from .strategies import kind_of
            return kind_of(self.current.name)
        except Exception:  # noqa: BLE001
            return ""

    # ------------------------------------------------------------------ 复位
    def reset(self) -> None:
        """平台瞬移后调用：丢掉策略实例，下一帧按当前里程重建并 sync。"""
        self.current = None
        self.strategy = None

    def names(self) -> list[str]:
        return [sp.name for sp in self.spans]
