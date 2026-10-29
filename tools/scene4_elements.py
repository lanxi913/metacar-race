# -*- coding: utf-8 -*-
"""连续场景 4（lianxu4）的元素级控制器路由。

## 为什么要有这一层

`tools/park_cs4.py` 用一套通用「曲率剖面 + 纯跟踪 + 跟车限速」跑完整条
4718 m 赛道。2026-10-25 的 cont-r2 跑次（15100 帧 / 502 s）暴露三件事：

* 元素 6 复杂路况超车 / 7 匝道行驶 / 8 车辆紧急停止 结算表里整行消失（0 分）；
* 十字路口通行 170.34，其中 −39.66 是闯红灯；
* 车停在 s=3042.9 的前车后面再也不动，直到场景超时收掉。

`main` 分支上每个元素都有单独实测过分的入口脚本。本模块只做
「按平台给的元素里程切一段几何 → 交给那个控制器 → 拿回控制量」，
控制律一行不重写；每个处理器的注释里写明它照抄的是哪个入口。

## 路由表（元素名关键字 → 处理器 → 依据）

| 元素 | 处理器 | 依据（main 上的入口 / 分数） |
|---|---|---|
| 车辆直行 | 定油门 0.55 + 纯跟踪 | 本仓库 2026-10-24 局 17/18 实测 110 |
| 连续转弯 / 匝道行驶 / 辅路驶入驶出 / 复杂路况超车 / 车辆变道 / 车辆切出 | `geo_follower.GeoFollower` | lianxu1_run.py（连续场景1 三局 630）；fudao_run.py 辅路 110；fuza_run.py 复杂超车 113.81 |
| 环岛驶入、驶出 | `ring_turn.RingTurnController` | tools/ring_turn_run.py |
| 车辆跟随 | `follow.select_leader` + `compute_target_speed` + `LongitudinalController` | tools/follow_run.py |
| 车辆紧急停止 | `emergency_stop` 剖面 + `follow.LongitudinalController` | tools/emergency_stop_run.py |
| 十字路口通行 / 丁字路口通行 / 无保护左转 | 同紧急停止的纵向栈 + 绿灯 ETA 闸门 | shizi_run.py / dingzi_run.py 用 RouteFollower 拿 200；**灯逻辑 main 上没有**，闸门是本仓库补的 |
| 平行车道泊车 | `parallel_parking.ParallelParkingController` | parallel_run.py 连续 5 次 210.00 |

车辆变道 / 车辆切出在 main 上只有 `lianxu1_run.py` 的 GeoFollower 路径，
所以也交给 geo；`LaneFollower` 需要 `static` 且目标是整条 route 末点，
放进连续场景会被它带着跑完剩下 4000 m，暂不接。

## 约定

* 处理器返回 `ElementControl`；返回 `None` 表示「本元素不接管」，
  `park_cs4.py` 继续用它原来的纵向/横向栈。
* 处理器内部异常会被路由层吞掉，并把该元素**永久降级**到通用路径：
  一个元素的 bug 不允许杀掉整局。
* 所有几何都来自 `static.route` / `static.sub_scenes[*].end_point`，没有写死坐标。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from metacar import GearMode

from metacar_race.config import (
    EMERGENCY_STOP_HOLD_SECONDS,
    EMERGENCY_STOP_RESPONSE_SECONDS,
    LIMIT_SCENE_ELEMENT_KMH,
    PARALLEL_CAR_LEN,
)
from metacar_race.control.emergency_stop import (
    endpoint_target_speed,
    make_stop_frame,
    minimum_endpoint_clearance,
    safe_stop_margin,
    stop_coordinates,
)
from metacar_race.control.follow import (
    LongitudinalController,
    adaptive_params,
    compute_target_speed,
    select_leader,
)
from metacar_race.control.geo_follower import GeoFollower
from metacar_race.control.parallel_parking import ParallelParkingController
from metacar_race.control.ring_turn import RingTurnController, estimate_lane_width
from metacar_race.control.tracking import PurePursuit, SpeedController

# 信号灯闸门：沿用 park_cs4 十字路口那套标定（那里实测过 200 分），
# 只补「绿灯剩余时间够不够过线」。
LIGHT_DECEL_MS2 = 1.20
LIGHT_STOP_GAP_M = 2.60
LIGHT_ETA_MARGIN_S = 0.50

STRAIGHT_THROTTLE = 0.55      # 车辆直行满分跑法的定油门（局 17/18 实测 110）
EMERGENCY_STOP_GIVEUP_S = 25.0  # 连续场景里不能无限等：停够这么久就交回通用路径
EGO_WIDTH_FALLBACK_M = 1.90
LANE_WIDTH_FALLBACK_M = 3.50

# 起步段接管：元素 1 从 s=8.2 m 才开始，前面那一小段原本走 park_cs4 的通用栈。
# full-r1（2026-10-25）实测 t=0.57 s、v 才 1.96 m/s 时就出现 brake=0.17：速度环
# 按 1.5 m/s² 爬坡、车却因油门量化瞬跳 12–15 m/s²，于是判超速给刹车。按 18 局
# 标定，只要出现 ≥0.10 的刹车就丢 +10 —— 元素 1 的 100 分没有 +10 就丢在这一帧。
START_SPAN_NAME = "起步直行"
START_SPAN_MIN_M = 1.0


class _XY:
    """最小点类型：main 上的控制器只读 .x/.y。"""

    __slots__ = ("x", "y")

    def __init__(self, x: float, y: float) -> None:
        self.x = float(x)
        self.y = float(y)

    def __repr__(self) -> str:  # pragma: no cover
        return f"({self.x:.1f},{self.y:.1f})"


def to_xy_objects(pts) -> list[_XY]:
    return [_XY(x, y) for x, y in pts]


def cumulative(route) -> list[float]:
    cum = [0.0]
    for i in range(1, len(route)):
        dx = float(route[i].x) - float(route[i - 1].x)
        dy = float(route[i].y) - float(route[i - 1].y)
        cum.append(cum[-1] + math.hypot(dx, dy))
    return cum


def slice_route(route, cum, s0: float, s1: float,
                head_pad: float = 8.0, tail_pad: float = 18.0) -> list[tuple[float, float]]:
    """按里程切一段点列；起点回退 head_pad、终点前伸 tail_pad，避免端点硬停。"""
    if not route:
        return []
    a = max(0.0, float(s0) - head_pad)
    b = min(cum[-1], float(s1) + tail_pad)
    pts = [(float(p.x), float(p.y)) for p, s in zip(route, cum) if a - 1e-6 <= s <= b + 1e-6]
    if len(pts) < 2:
        pts = [(float(p.x), float(p.y)) for p in route]
    return pts


@dataclass
class ElementContext:
    """每帧交给元素的快照，由 park_cs4 填。"""

    pose: object
    speed: float
    dt: float
    obstacles: list
    static: object
    sim_t: float
    s_now: float
    x: float
    y: float
    yaw: float
    lane_width: float
    ego_length: float
    ego_width: float
    vmax: float
    time_limit: float
    end_point: object | None
    trajectory: list | None = None
    a_now: float = 0.0
    light: object | None = None
    stop_gap: float | None = None
    # 通用栈这一帧算出来的目标车速（曲率剖面 + 跟车/信号灯限速之后的值）。
    # 元素处理器自己闭环纵向时用它，就不用再猜曲率限速。
    v_target: float | None = None


@dataclass
class ElementControl:
    """元素处理器给的控制量。**None = 沿用 park_cs4 通用栈的对应通道**。

    环岛就是这么用的：横向（方向盘）交给 `RingTurnController`，纵向仍走
    通用栈——full-r4 实测把纵向也接管后，速度环因为油门增益太大（0.45 就是
    12~15 m/s²）把油门切成 0，车在环岛里以 0.6~1.2 m/s 抽搐；而通用栈在
    cont-r2 里能一路把环岛跑完。
    """

    throttle: float | None = None
    brake: float | None = None
    steering: float | None = None
    gear: object | None = None
    note: str = ""


class ElementHandler:
    """元素处理器基类。"""

    key = "generic"

    def step(self, ctx: ElementContext) -> ElementControl | None:
        return None

    def sync(self, x: float, y: float) -> None:
        """把内部跟踪索引对齐到当前位姿。

        平台瞬移（元素超时重开 / 跳关 / 复位）后，`park_cs4.py` 会重置自己那份
        `pursuit.idx`；处理器各自持有的跟踪器也必须一起复位，否则前视点留在
        几百米之外，车会直着开出路面（full-r1：steer 恒 0.000，横向误差 +13.2 m）。
        默认什么都不做，有内部跟踪器的子类覆盖它。
        """
        return None


class StraightElement(ElementHandler):
    """车辆直行：整段同一个油门，转向交给纯跟踪。"""

    key = "straight"

    def __init__(self, pts, throttle: float = STRAIGHT_THROTTLE) -> None:
        self.pursuit = PurePursuit(to_xy_objects(pts))
        self.throttle = float(throttle)

    def step(self, ctx: ElementContext) -> ElementControl:
        steer = self.pursuit.steering(ctx.x, ctx.y, ctx.yaw)
        return ElementControl(self.throttle, 0.0, steer, note=self.key)

    def sync(self, x: float, y: float) -> None:
        self.pursuit.sync(x, y, max_jump_m=1e9)


class GeoElement(ElementHandler):
    """曲率剖面 + 纯跟踪。

    **几何/横向**照抄 `lianxu1_run.py` 的 `GeoFollower`（连续场景1 三局 630）。
    **纵向**不用它自己那份油门：它的输出在连续场景4 的车上会饱和在 0.24，
    车速钉死 4.78 m/s（一挡墙），元素 2（1023 m / 200 s，需 ≥5.1 m/s）因此
    必然超时重开（full-r1 211.9 s、full-r2 211.9 s 两次瞬移都是这个原因）。
    改成 `SpeedController`（直行 110 分那套）闭环追 park_cs4 给的 `v_target`，
    并打开它自带的**升挡 kick**：油门给足但速度 0.8 s 不涨就补一脚，升挡后自动退。
    """

    key = "geo"

    def __init__(self, pts, speed_limit: float) -> None:
        self.pts = list(pts)
        self.speed_limit = float(speed_limit)
        self.geo = GeoFollower(self.pts, speed_limit=self.speed_limit)
        self.speed = SpeedController(v_target=self.speed_limit, accel_limit=1.2,
                                     throttle_max=0.50, kick_throttle=0.75,
                                     kick_time=0.30)
        self.speed.reset(0.0)

    def _target(self, ctx: ElementContext) -> float:
        tgt = self.speed_limit if ctx.v_target is None else float(ctx.v_target)
        return max(0.0, min(self.speed_limit, tgt))

    def step(self, ctx: ElementContext) -> ElementControl | None:
        vc, _info = self.geo.update(ctx.pose, ctx.speed, ctx.dt)
        if getattr(self.geo, "done", False):
            return None
        thr, brk = self.speed.update(ctx.speed, ctx.dt, v_ceiling=self._target(ctx),
                                     a_now=ctx.a_now, allow_kick=True)
        return ElementControl(thr, brk, vc.steering, note=self.key)

    def sync(self, x: float, y: float) -> None:
        # GeoFollower 没有对外 sync：重建一份最稳（它内部索引从 0 起算）。
        self.geo = GeoFollower(self.pts, speed_limit=self.speed_limit)
        self.speed.reset(0.0)


class RingElement(ElementHandler):
    """环岛：**只接管方向盘**，纵向交回 park_cs4 通用栈。

    为什么纵向不接管（2026-10-25 full-r4 实测教训）：本车的油门增益极大
    （0.45 油门就是 12~15 m/s²），而 `SpeedController` 按 1.5 m/s² 的目标加速度
    闭环，于是每次加油都「超调 → 切油门到 0 → 掉速 → 再加油」，车在环岛里以
    0.6~1.2 m/s 反复抽搐，175 s 只走了 165 m。通用栈（cont-r2）用同一个环岛
    跑到 8 m/s、能一路到元素 8，所以纵向交给它。

    `RingTurnController` 自己那份油门也不能用：它的标定是给另一台车/另一套执行器
    的（`RING_TURN_START_THROTTLE_MAX=0.004`、`RING_TURN_CRUISE_THROTTLE_MAX=0.12`），
    本车等于永不供油（元素3 单跑：16 s 里 thr 恒 0.00、原地不动）。
    它真正强的是横向几何，这一份照抄 ring_turn_run.py。
    """

    key = "ring"

    def __init__(self, pts, car_length: float, car_width: float, lane_width: float,
                 speed_cap_ms: float = 6.0) -> None:
        self._route = to_xy_objects(pts)
        self._args = (float(car_length), float(car_width), float(lane_width))
        self.ctrl = RingTurnController(self._route, *self._args)
        self.speed_cap = float(speed_cap_ms)
        self.finish_reported = False
        self.last_log = -1.0
        self.log = print

    def sync(self, x: float, y: float) -> None:
        self.ctrl = RingTurnController(self._route, *self._args)
        self.finish_reported = False
        self.last_log = -1.0

    def step(self, ctx: ElementContext) -> ElementControl | None:
        res = self.ctrl.compute(
            ctx.x, ctx.y, ctx.yaw, ctx.speed, ctx.sim_t, ctx.time_limit,
            dynamic_path=list(ctx.trajectory or []), a_now=ctx.a_now,
        )
        if getattr(res, "finish_mode", False):
            # 控制器认为这一圈走完了：交回通用路径，让平台按里程判元素结束。
            if not self.finish_reported:
                self.finish_reported = True
                self.log(f"[ring] finish_mode，交回通用路径 v={ctx.speed:.2f}")
            return None
        if ctx.sim_t - self.last_log > 2.0:
            self.last_log = ctx.sim_t
            self.log(f"[ring] steer={float(res.steering):+.3f} "
                     f"（纵向沿用通用栈）v={ctx.speed:.2f} s={ctx.s_now:.1f}")
        return ElementControl(steering=float(res.steering), note=self.key)


class FollowElement(ElementHandler):
    """车辆跟随：照抄 follow_run.py（select_leader + 目标速度 + 纵向执行器）。"""

    key = "follow"

    def __init__(self, pts, ego_length: float, ego_width: float,
                 lane_width: float, speed_limit: float) -> None:
        self.pursuit = PurePursuit(to_xy_objects(pts))
        self.params = adaptive_params(ego_length, ego_width, lane_width, speed_limit)
        self.lon = LongitudinalController()

    def step(self, ctx: ElementContext) -> ElementControl:
        leader = select_leader(ctx.obstacles, ctx.pose, ctx.ego_length, self.params)
        target, _desired_gap, _safe = compute_target_speed(leader, None, self.params)
        gap = None if leader is None else float(leader.gap_m)
        thr, brk = self.lon.update(ctx.speed, target, gap, ctx.dt, self.params)
        steer = self.pursuit.steering(ctx.x, ctx.y, ctx.yaw)
        return ElementControl(thr, brk, steer, note=self.key)

    def sync(self, x: float, y: float) -> None:
        self.pursuit.sync(x, y, max_jump_m=1e9)


class StopAtEndElement(ElementHandler):
    """停车类元素：紧急停止 / 十字路口 / 丁字路口 / 无保护左转。

    纵向完全照抄 tools/emergency_stop_run.py：
    `stop_coordinates` → `safe_stop_margin` → `endpoint_target_speed` →
    `LongitudinalController`；横向用该文件的「终点前 0.6 车长回正」规则。
    十字路口额外加一道绿灯 ETA 闸门（main 上的控制器都不读信号灯）。
    """

    key = "stop"

    def __init__(self, pts, end_point, ego_length: float, ego_width: float,
                 lane_width: float, vmax: float, *, use_light: bool = False,
                 stop_at_end: bool = True) -> None:
        self.objects = to_xy_objects(pts)
        self.end_point = end_point
        self.frame = make_stop_frame(end_point, self.objects) if end_point is not None else None
        self.pursuit = PurePursuit(self.objects)
        self.params = adaptive_params(ego_length, ego_width, lane_width,
                                      LIMIT_SCENE_ELEMENT_KMH / 3.6)
        self.lon = LongitudinalController()
        self.ego_length = float(ego_length)
        self.vmax = float(vmax)
        self.use_light = bool(use_light)
        self.stop_at_end = bool(stop_at_end)
        self.clearance = minimum_endpoint_clearance(self.ego_length)
        self.held = 0.0
        self.given_up = False

    def _light_cap(self, ctx: ElementContext) -> float | None:
        if not self.use_light or ctx.light is None or ctx.stop_gap is None:
            return None
        gap = float(ctx.stop_gap)
        if gap <= 0.0:
            return None
        try:
            state = ctx.light.straight_state.name
            remain = float(ctx.light.straight_remaining_time)
        except Exception:  # noqa: BLE001 - 字段缺失就当没有灯
            return None
        if state == "RED":
            return math.sqrt(2.0 * LIGHT_DECEL_MS2 * max(gap - LIGHT_STOP_GAP_M, 0.0))
        t_line = gap / max(ctx.speed, 0.5)
        if t_line + LIGHT_ETA_MARGIN_S > remain:
            return math.sqrt(2.0 * LIGHT_DECEL_MS2 * max(gap - LIGHT_STOP_GAP_M, 0.0))
        return None

    def step(self, ctx: ElementContext) -> ElementControl | None:
        if self.given_up:
            return None

        # 十字路口类：不要求在元素终点停住，只做灯闸门 + 循迹。
        if self.frame is None or not self.stop_at_end:
            steer = self.pursuit.steering(ctx.x, ctx.y, ctx.yaw)
            cap = self._light_cap(ctx)
            target = self.params.max_speed_ms if cap is None else min(self.params.max_speed_ms, cap)
            thr, brk = self.lon.update(ctx.speed, target, None, ctx.dt, self.params)
            if ctx.speed > target + 0.6:
                brk = max(brk, 0.10)
            return ElementControl(thr, brk, steer, note=self.key + ":cross")

        coords = stop_coordinates(ctx.pose, self.frame)
        along = float(coords.along_m)

        leader = select_leader(ctx.obstacles, ctx.pose, ctx.ego_length, self.params)
        leader_clearance = None if leader is None else float(leader.gap_m) - along
        margin = safe_stop_margin(leader_clearance, self.clearance)
        control_along = along - margin

        if control_along <= 0.05 and abs(ctx.speed) <= 0.10:
            # 停稳保持：照抄 emergency_stop_run.py 的刹车斜坡。
            self.held += ctx.dt
            brake = min(1.0, 0.22 + 0.55 * self.held)
            if self.held >= EMERGENCY_STOP_HOLD_SECONDS + EMERGENCY_STOP_GIVEUP_S:
                self.given_up = True
                return None
            return ElementControl(0.0, brake, 0.0, gear=GearMode.PARKING,
                                  note=self.key + ":hold")

        target = endpoint_target_speed(
            control_along, ctx.speed, self.params.max_speed_ms,
            EMERGENCY_STOP_RESPONSE_SECONDS, self.clearance,
        )
        cap = self._light_cap(ctx)
        if cap is not None:
            target = min(target, cap)

        thr, brk = self.lon.update(ctx.speed, target, None, ctx.dt, self.params)
        steer = self.pursuit.steering(ctx.x, ctx.y, ctx.yaw)
        if along < 0.6 * self.ego_length:
            steer = 0.0
        return ElementControl(thr, brk, steer, note=self.key)

    def sync(self, x: float, y: float) -> None:
        self.pursuit.sync(x, y, max_jump_m=1e9)
        self.held = 0.0
        self.given_up = False


class ParallelParkElement(ElementHandler):
    """平行车道泊车：照抄 templates/parking/zyyxnjydhcg/parallel_run.py。"""

    key = "park"

    def __init__(self) -> None:
        self.ctrl = ParallelParkingController()

    def step(self, ctx: ElementContext) -> ElementControl | None:
        if self.ctrl.done():
            return None
        vc, _info = self.ctrl.update(ctx.pose, ctx.speed, ctx.obstacles, ctx.dt)
        return ElementControl(vc.throttle, vc.brake, vc.steering,
                              gear=vc.gear, note=self.key)

    def sync(self, x: float, y: float) -> None:
        self.ctrl = ParallelParkingController()


# 元素名关键字 → 处理器类型（按顺序匹配，先命中先得）
_RULES: tuple[tuple[str, str], ...] = (
    ("紧急停止", "stop"),
    ("十字路口", "cross"),
    ("丁字路口", "cross"),
    ("无保护左转", "cross"),
    ("环岛", "ring"),
    ("跟随", "follow"),
    ("直行", "straight"),
    ("平行车道泊车", "park"),
    ("连续转弯", "geo"),
    ("匝道", "geo"),
    ("辅路", "geo"),
    ("超车", "geo"),
    ("变道", "geo"),
    ("切出", "geo"),
)


def handler_kind(name: str) -> str | None:
    for keyword, kind in _RULES:
        if keyword in name:
            return kind
    return None


class ElementRouter:
    """按里程判定当前元素，构造并调用对应处理器。"""

    def __init__(self, static, route, spans, *, vmax: float,
                 ego_length: float = PARALLEL_CAR_LEN,
                 ego_width: float = EGO_WIDTH_FALLBACK_M,
                 lane_width: float | None = None,
                 only: list[str] | None = None,
                 log=print) -> None:
        self.static = static
        self.route = list(route)
        self.cum = cumulative(self.route)
        self.spans = sorted(
            [s for s in spans
             if s.get("s_start") is not None and s.get("s_end") is not None],
            key=lambda s: float(s.get("s_start") or 0.0))
        # 起步段（s=0 → 元素 1 起点）原本没人管，走的是 park_cs4 的通用栈，
        # 那里 0.57 s 就给出一次 brake=0.17（把元素 1 的 +10 打掉）。
        # 合成一段「起步直行」交给定油门处理器，从第一帧就接管。
        first_start = float(self.spans[0].get("s_start") or 0.0) if self.spans else 0.0
        if first_start > START_SPAN_MIN_M:
            self.spans.insert(0, {
                "index": 0,
                "name": START_SPAN_NAME,
                "s_start": 0.0,
                "s_end": first_start,
            })
        self.vmax = float(vmax)
        self.ego_length = float(ego_length)
        self.ego_width = float(ego_width)
        self.lane_width = float(lane_width or LANE_WIDTH_FALLBACK_M)
        self.log = log
        self.current_key = None
        self.handler: ElementHandler | None = None
        self.disabled: set = set()
        self.stats: dict[str, int] = {}
        self.announced: dict[str, str] = {}
        self.only: list[str] = [t.strip() for t in (only or []) if str(t).strip()]

    def _span_at(self, s_now: float):
        """按里程找当前元素：先严格包含，再退回「最近一个已经开始过的元素」。

        早期的 ±1.0 m 宽容会两头都命中，于是 s=1084.0（元素3 起点）被判成元素2
        （60.1~1083.0），元素2 的 GeoElement 一看自己 slice 已经到底就刹车 0.17，
        车从 8.2 m/s 一路刹停——2026-10-25 元素3 单跑就是这么挂的。
        """
        started = None
        for span in self.spans:
            a, b = span.get("s_start"), span.get("s_end")
            if a is None or b is None:
                continue
            if a <= s_now <= b:
                return span
            if a <= s_now and (started is None or a > started[0]):
                started = (a, span)
        return None if started is None else started[1]

    def _end_point(self, span):
        idx = int(span.get("index", 0)) - 1
        if 0 <= idx < len(self.static.sub_scenes):
            return self.static.sub_scenes[idx].end_point
        return None

    def _build(self, span) -> ElementHandler | None:
        if span is None:
            return None
        name = str(span.get("name") or "")
        if self.only and not any(t in name for t in self.only):
            return None
        kind = handler_kind(name)
        if kind is None:
            return None
        s0 = float(span.get("s_start") or 0.0)
        s1 = float(span.get("s_end") or self.cum[-1])
        pts = slice_route(self.route, self.cum, s0, s1)
        if len(pts) < 2:
            return None
        end_point = self._end_point(span)
        try:
            if kind == "straight":
                handler: ElementHandler | None = StraightElement(pts)
            elif kind == "geo":
                handler = GeoElement(pts, speed_limit=self.vmax)
            elif kind == "ring":
                handler = RingElement(pts, self.ego_length, self.ego_width, self.lane_width,
                                      speed_cap_ms=min(self.vmax, 6.0))
            elif kind == "follow":
                handler = FollowElement(pts, self.ego_length, self.ego_width,
                                        self.lane_width, LIMIT_SCENE_ELEMENT_KMH / 3.6)
            elif kind == "cross":
                handler = StopAtEndElement(pts, end_point, self.ego_length, self.ego_width,
                                           self.lane_width, self.vmax,
                                           use_light=True, stop_at_end=False)
            elif kind == "stop":
                handler = StopAtEndElement(pts, end_point, self.ego_length, self.ego_width,
                                           self.lane_width, self.vmax,
                                           use_light=False, stop_at_end=True)
            elif kind == "park":
                handler = ParallelParkElement()
            else:  # pragma: no cover
                handler = None
        except Exception as exc:  # noqa: BLE001 - 构造失败不能杀整局
            self.log(f"[router] 元素 {name} 处理器构造失败，降级通用路径: {exc!r}")
            return None
        if handler is not None and self.announced.get(name) != handler.key:
            self.announced[name] = handler.key
            self.log(f"[router] 元素 {name} → {handler.key}"
                     f"（{s0:.0f}~{s1:.0f} m，{len(pts)} 点）")
        return handler

    def _sync_handler(self, ctx: ElementContext) -> None:
        """处理器刚建好时把内部跟踪器对齐到当前位姿。"""
        sync = getattr(self.handler, "sync", None)
        if sync is None:
            return
        try:
            sync(ctx.x, ctx.y)
        except Exception:  # noqa: BLE001 - 对齐失败不影响接管
            pass

    def reset(self) -> None:
        """平台瞬移 / 元素超时重开：丢掉已建处理器，下一帧按新位置重建。

        full-r1 的回归：主栈瞬移时重置了自己的 `pursuit.idx`，但路由里的处理器
        各自持有 `PurePursuit`，没人重置 → steer 恒 0.000、横向误差涨到 +13.2 m。
        这里只清当前处理器，禁用列表与统计保留（降级的元素不能借瞬移复活）。
        """
        self.current_key = None
        self.handler = None

    def step(self, ctx: ElementContext) -> ElementControl | None:
        span = self._span_at(ctx.s_now)
        key = None if span is None else (str(span.get("name")), span.get("s_start"))
        if key != self.current_key:
            self.current_key = key
            self.handler = None if key is None else self._build(span)
            if key is not None and key in self.disabled:
                self.handler = None
            if self.handler is not None:
                self._sync_handler(ctx)
        if self.handler is None:
            return None
        try:
            ctl = self.handler.step(ctx)
        except Exception as exc:  # noqa: BLE001 - 降级而不是崩
            self.disabled.add(key)
            self.handler = None
            self.log(f"[router] 元素 {key[0]} 处理器异常，降级通用路径: {exc!r}")
            return None
        if ctl is not None:
            self.stats[self.handler.key] = self.stats.get(self.handler.key, 0) + 1
        return ctl


def lane_width_at(static, x: float, y: float, fallback: float = LANE_WIDTH_FALLBACK_M) -> float:
    """按最近道路估一次车道宽，失败就退回 3.5 m。"""
    try:
        return float(estimate_lane_width(static.roads, x, y, fallback))
    except Exception:  # noqa: BLE001
        return float(fallback)
