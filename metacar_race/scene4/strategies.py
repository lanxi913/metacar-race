"""元素策略：每个元素只声明「我接管哪条通道」，别的通道留 None 交给基线。

这是本轮重构的核心约定，也是在 full-r3/full-r4 两次失败后定下来的：

* **纵向**（油门/刹车）只有一套执行器，就是 driver 里那套"曲率剖面 + 速度环 +
  破挡 + 收油治理"。本车油门 0.45 就有 12~15 m/s²，任何按 1.5 m/s² 闭环的
  第二个速度环都会「加油 → 切 0 → 掉速」抽搐（环岛 175 s 只走 165 m 就是它）。
* 所以策略能给的只有：`steer`（横向）、`v_cap`（纵向限速，交给基线的速度环）、
  以及极少数确实验证过的 `throttle/brake` 直发（起步段的定油门 0.55）。
* `None` = 这一通道沿用基线，**不要**用 0.0 表示"不接管"——0.0 是"油门踩到底
  不动"，语义完全不同。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from metacar import GearMode

from . import config as C
from .lateral import Pt
from .longitudinal import StraightRun
from .track import RouteTrack

try:                                        # 环岛控制器（main 上已验证）
    from metacar_race.control.ring_turn import RingTurnController
    _RING_OK = True
    _RING_ERR = ""
except Exception as exc:  # noqa: BLE001
    RingTurnController = None               # type: ignore[assignment]
    _RING_OK = False
    _RING_ERR = repr(exc)

try:                                        # 纯跟踪（control.tracking 的成熟实现）
    from metacar_race.control.tracking import PurePursuit
    _PP_OK = True
except Exception as exc:  # noqa: BLE001
    PurePursuit = None                      # type: ignore[assignment]
    _PP_OK = False
    _PP_ERR = repr(exc)


@dataclass
class Command:
    """策略返回的控制意图。三个纵向/横向/档位通道都可以是 None（= 沿用基线）。"""

    steer: float | None = None
    v_cap: float | None = None
    throttle: float | None = None
    brake: float | None = None
    gear: GearMode | None = None
    coast: bool | None = None            #: True = 松油滑行（油门刹车都置 0）
    final: bool = False                  #: True = 策略全权接管（基线不许再插手升挡闸门/终点驻车）
    note: str = ""


@dataclass
class ElementContext:
    """每帧交给策略的全部输入（不再让策略自己去找 SDK）。"""

    x: float
    y: float
    yaw: float
    speed: float
    dt: float
    sim_t: float
    s_now: float
    pose: object = None
    obstacles: list = field(default_factory=list)
    static: object = None
    lane_width: float = 3.5
    ego_length: float = 4.70
    ego_width: float = 1.90
    vmax: float = 8.0
    time_limit: float = 0.0
    end_point: object | None = None
    trajectory: object | None = None
    a_now: float = 0.0
    v_cap: float | None = None          #: 基线这一帧的速度上限
    light_state: str | None = None
    stop_gap: float | None = None
    s_start: float = 0.0
    s_end: float = 0.0


def cumulative(route) -> list[float]:
    """路线的累计里程（与 RouteTrack 口径一致）。"""
    pts = [(float(p.x), float(p.y)) for p in route]
    out = [0.0] * len(pts)
    for i in range(1, len(pts)):
        out[i] = out[i - 1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                         pts[i][1] - pts[i - 1][1])
    return out


def slice_route(route, cum: list[float], s0: float, s1: float,
                head_pad: float = 8.0, tail_pad: float = 18.0) -> list[Pt]:
    """按里程切一段路线（带前后缓冲）；切不出至少 2 点时退回整条。"""
    if not cum:
        return []
    a = s0 - float(head_pad)
    b = s1 + float(tail_pad)
    pts = [Pt(float(p.x), float(p.y)) for p, s in zip(route, cum) if a <= s <= b]
    if len(pts) < 2:
        return [Pt(float(p.x), float(p.y)) for p in route]
    return pts


class Strategy:
    """元素策略基类。默认什么都不接管。"""

    key = ""
    label = ""

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        self.route = route
        self.log = log
        self.s_start = ctx.s_start
        self.s_end = ctx.s_end

    def step(self, ctx: ElementContext) -> Command | None:
        return None

    def sync(self, x: float, y: float) -> None:
        """平台瞬移/跳关后把跟踪器状态拉回当前位置。"""

    def reset(self) -> None:
        """元素重开时复位内部状态（默认等价于 sync 上的重置）。"""


class StartStraightStrategy(Strategy):
    """起步/直行：**长直道纵向规划 + 纯跟踪**（油门从 0 渐加，弯前松油滑行）。

    历史：`accel_window_probe` 18 局里，定油门 0.50/0.55/0.60 + 零刹车拿 110；
    而元素 1 用通用栈时起步 0.57 s 就会因为「速度环按 1.5 m/s² 爬坡、实际 12~15
    m/s²」判超速给一脚 `brake=0.17`，直接把 +10 丢掉（full-r1）。改成定油门后
    full-r2/r4 元素 1 都拿到 110。

    2026-10-25 用户要求：长直道时油门从 0 逐渐加大，用「预测减速到终点（终点速度
    不为 0）」反推松油点，用滑行代替刹车。于是纵向交给 `StraightRun`：油门按斜率
    爬到 `STRAIGHT_THROTTLE`（终值与原来定油门的 0.55 相同），到松油点就 `coast`。
    """

    key = "straight"
    label = "直道纵向规划"

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        super().__init__(ctx, route, log)
        self.pursuit = None
        if _PP_OK and len(route) >= 2:
            self.pursuit = PurePursuit(route, lookahead=2.5)
            self.pursuit.idx = 0
        self.throttle = C.STRAIGHT_THROTTLE
        self.run = StraightRun(v_cap=float(ctx.vmax or 8.0))
        self.cmap = None
        try:                       # 延迟导入，避免 curve ← strategies 的环
            from .curve import CurveMap
            base = getattr(getattr(ctx, "static", None), "route", None) or route
            self.cmap = CurveMap(base)
        except Exception as exc:   # noqa: BLE001
            if self.log:
                self.log(f"[straight] 弯道索引构造失败，按到元素终点规划: {exc!r}")
            self.cmap = None

    def sync(self, x: float, y: float) -> None:
        if self.pursuit is not None:
            self.pursuit.sync(x, y, max_jump_m=1e9)

    def reset(self) -> None:
        if self.pursuit is not None:
            self.pursuit.idx = 0
        if self.run is not None:
            self.run.reset()

    def step(self, ctx: ElementContext) -> Command | None:
        steer = None
        if self.pursuit is not None:
            steer = float(self.pursuit.steering(ctx.x, ctx.y, ctx.yaw))

        dist = None
        if self.cmap is not None:
            dist = self.cmap.dist_to_next(ctx.s_now)
        if dist is None:                       # 前方没弯：把元素终点当终点
            dist = max(0.0, float(ctx.s_end or 0.0) - float(ctx.s_now))

        thr, brk, coast, note = self.run.update(ctx.dt, ctx.speed, dist)
        if coast:
            return Command(steer=steer, coast=True, note="coast %.0fm" % dist)
        return Command(steer=steer, throttle=thr, brake=brk,
                       note="%s %.0fm %s" % (self.key, dist, note))


class RingStrategy(Strategy):
    """环岛（元素 3）：**只接管方向盘**，纵向一律交回基线。

    `RingTurnController` 是 main 上环岛用的控制器，但它的油门标定（`config.py` 里
    `RING_TURN_START_THROTTLE_MAX=0.004`、`RING_TURN_CRUISE_THROTTLE_MAX=0.12`）
    是给另一套执行器的——本车照它给油等于不给油（m19 局原地 16 s）。自己再搭一个
    速度环也不行（full-r4 抽搐 175 s）。所以只取它的 `steering`。
    """

    key = "ring"
    label = "环岛仅横向"

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        super().__init__(ctx, route, log)
        self.ctl = None
        self.finish_reported = False
        self.ego = (float(ctx.ego_length or 4.70), float(ctx.ego_width or 1.90),
                    float(ctx.lane_width or 3.50))
        self._build()

    def _build(self) -> None:
        self.ctl = None
        if not _RING_OK or len(self.route) < 3:
            return
        try:
            self.ctl = RingTurnController(self.route, *self.ego)
        except Exception as exc:  # noqa: BLE001
            if self.log:
                self.log(f"[ring] 控制器构造失败，走基线: {exc!r}")
            self.ctl = None

    def sync(self, x: float, y: float) -> None:
        """瞬移后重建控制器（几何没变，内部里程/曲率缓存必须重算）。"""
        self._build()
        self.finish_reported = False

    def step(self, ctx: ElementContext) -> Command | None:
        if self.ctl is None:
            return None
        # 首帧 `scene_status.time_limit` 常常还是 0；也有元素已经把预算用完的情况。
        # 这两种都会让控制器的 `time_budget_left_s` 变成负数（那边会直接抛），所以
        # 这里保证预算恒为正——元素什么时候结束由 router 按里程决定，不靠计时。
        tl = float(ctx.time_limit or 0.0)
        if tl <= 1.0 or tl <= float(ctx.sim_t) + 1.0:
            tl = float(ctx.sim_t) + 60.0
        try:
            res = self.ctl.compute(
                ctx.x, ctx.y, ctx.yaw, ctx.speed, ctx.sim_t,
                tl, dynamic_path=ctx.trajectory, a_now=ctx.a_now)
        except Exception as exc:  # noqa: BLE001
            if self.log:
                self.log(f"[ring] compute 失败，降级通用路径: {exc!r}")
            return None
        finish = bool(getattr(res, "finish_mode", False))
        if finish and not self.finish_reported:
            self.finish_reported = True
            if self.log:
                self.log("[ring] finish_mode，交回基线纵向")
            return None
        return Command(steer=float(getattr(res, "steering", 0.0)),
                       note="%s:%.2f" % (self.key, float(getattr(res, "target_speed_ms", 0.0))))


STRATEGY_BY_KIND = {
    "straight": StartStraightStrategy,
    "ring": RingStrategy,
}

# 弯道切内弯（`curve.py` 反过来引用本模块的 Command/Strategy，所以在文件末尾挂载，
# 让它能拿到已经定义好的基类）。
try:
    from .curve import CurveCutStrategy
    STRATEGY_BY_KIND["curve"] = CurveCutStrategy
except Exception as _exc:  # noqa: BLE001
    _CURVE_ERR = repr(_exc)

# 真变道（`lanechange.py`）：跨虚线整条平移，靠 `--lane-element` 显式指定元素才启用。
try:
    from .lanechange import LaneChangeStrategy
    STRATEGY_BY_KIND["lane"] = LaneChangeStrategy
except Exception as _exc:  # noqa: BLE001
    _LANE_ERR = repr(_exc)

# 平行泊车（`parkslot.py`）：认到 PARKING_SLOT 才接管，走"车位坐标系 + 两段圆弧
# 倒车"，绝不沿 route 直接开进车位（用户 2026-10-25 明确要求）。
try:
    from .parkslot import SlotParkStrategy
    STRATEGY_BY_KIND["parking"] = SlotParkStrategy
except Exception as _exc:  # noqa: BLE001
    _PARK_ERR = repr(_exc)


def kind_of(name: str) -> str:
    """元素名 → 策略种类。只认**已经验证过**的映射，其余返回 ""（走基线）。"""
    if not name:
        return ""
    if C.START_SPAN_NAME in name:
        return "straight"
    if "环岛" in name:
        # 元素 3（m03278）：和元素 2 一样走切弯——环半径小、内侧净空有限，
        # 靠切弯把曲率降下来才敢在元素里维持 6 m/s 以上（稳在二挡）。
        return "curve"
    if "直行" in name:              # 元素 1：定油门 0.55 拿 +10
        return "straight"
    if "连续转弯" in name:          # 元素 2：切内弯
        return "curve"
    if "泊车" in name:              # 平行车道泊车：认车位 → 车位坐标系倒车入库
        return "parking"
    return ""


__all__ = [
    "Command", "ElementContext", "Strategy", "StartStraightStrategy", "RingStrategy",
    "STRATEGY_BY_KIND", "kind_of", "cumulative", "slice_route", "RouteTrack",
]
