# -*- coding: utf-8 -*-
"""真变道：跨**虚线**把整车平移一个车道，跑一段再平移回来。

和 `curve.py` 的「切内弯」不是一回事：

* 切弯 = 贴着本车道内侧走，偏移植上限 `MAX_CUT_M=1.60 m`，永远不越线；
* 变道 = 整条车道平移，偏移量 = 两条车道中心线间距 `(w1 + w2) / 2 ≈ 3.9 m`，
  而且**只在该侧边界是虚线时**才做（`LaneCorridor.neighbor()` 负责判定，
  实线 `SOLID_LINE`、双黄 `MIDDLE_LINE`、路边线 `SIDE_LINE` 一律拒绝）。

变道窗口：进段后先直行 → 用 `LANE_CHANGE_RAMP_M` 米斜到目标车道中心 → 保持
`LANE_CHANGE_HOLD_M` 米 → 再用同样长度斜回原车道。只接管方向盘，纵向一律交回基线
（第二个速度环必定抽搐，环岛 175 s 只走 165 m 那次已经证明）。
"""
from __future__ import annotations

import math

from . import config as C
from .corridor import LaneCorridor
from .strategies import Command, ElementContext, Pt, Strategy

try:                                   # 纯跟踪在 control 层，包一层防御性导入
    from ..control.tracking import PurePursuit

    _PP_OK = True
except Exception as _exc:  # noqa: BLE001
    PurePursuit = None                 # type: ignore[assignment]
    _PP_OK = False
    _PP_ERR = repr(_exc)


def _cum(pts) -> list[float]:
    out = [0.0] * len(pts)
    for i in range(1, len(pts)):
        out[i] = out[i - 1] + math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y)
    return out


def _normal(pts, i: int):
    """第 i 点的左法向（正 = 车头左侧）。"""
    a = pts[max(0, i - 1)]
    b = pts[min(len(pts) - 1, i + 1)]
    dx, dy = b.x - a.x, b.y - a.y
    L = math.hypot(dx, dy) or 1.0
    return (-dy / L, dx / L)


class LaneChangeStrategy(Strategy):
    """真变道：跨虚线整条平移，只给方向盘。"""

    key = "lane"
    label = "真变道"

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        super().__init__(ctx, route, log)
        self.base: list[Pt] = [Pt(float(p.x), float(p.y)) for p in route]
        self.path: list[Pt] = list(self.base)
        self.offsets: list[float] = [0.0] * len(self.base)
        self.corridor = None
        if ctx.static is not None:
            try:
                self.corridor = LaneCorridor(ctx.static,
                                             ego_width=float(ctx.ego_width or 1.81),
                                             margin=0.25,
                                             log_note=self.log)
            except Exception as exc:  # noqa: BLE001
                if self.log:
                    self.log(f"[lane] 车道走廊构造失败，不变道: {exc!r}")
                self.corridor = None
        self.side = 0
        self.shift = 0.0
        self.window = (0.0, 0.0, 0.0)     # (起, 保持结束, 结束)
        self.plan_ok = False
        self.pursuit = None               # 延迟到第一次 step/sync 再建（对齐其它策略）
        self._build(ctx)

    # ------------------------------------------------------------------ 建偏移线
    def _pick_side(self, ctx) -> tuple[int, float]:
        """在路段中部查一次走廊：哪侧是虚线就往哪侧变，两侧都行优先左。"""
        if self.corridor is None or len(self.base) < 4:
            return 0, 0.0
        cum = _cum(self.base)
        total = cum[-1]
        probe = total * 0.45
        i = 0
        for k, s in enumerate(cum):
            if s >= probe:
                i = k
                break
        room = self.corridor.room(self.base[i].x, self.base[i].y)
        if self.log and room is not None:
            self.log(f"[lane] 探测点 s={probe:.0f}: 车道 {room.lane_id} 宽 {room.lane_width:.2f} "
                     f"左type={room.left_type} 右type={room.right_type} "
                     f"净空 左{room.left:+.2f} 右{room.right:+.2f}")
        for side in (+1, -1):              # 左优先
            nb = self.corridor.neighbor(self.base[i].x, self.base[i].y, side)
            if self.log:
                tag = "左" if side > 0 else "右"
                if nb is None:
                    self.log(f"[lane] 探测点 {tag}侧不可变（非虚线/对向/无线）")
                else:
                    self.log(f"[lane] 探测点 {tag}侧可变为 {nb['lane_id']}，位移 {nb['shift_m']:.2f} m")
            if nb is None:
                continue
            shift = float(nb["shift_m"])
            if shift < C.LANE_CHANGE_MIN_M:
                continue
            return side, min(C.LANE_CHANGE_MAX_M, shift)
        return 0, 0.0

    def _build(self, ctx) -> None:
        self.side, self.shift = self._pick_side(ctx)
        n = len(self.base)
        self.offsets = [0.0] * n
        self.path = list(self.base)
        if self.side == 0 or self.shift <= 0.0 or n < 6:
            return
        cum = _cum(self.base)
        total = cum[-1]
        ramp = float(C.LANE_CHANGE_RAMP_M)
        hold = float(C.LANE_CHANGE_HOLD_M)
        if total < ramp * 2 + hold + 12.0:      # 段太短，退化成「去 + 回」各半
            ramp = max(6.0, (total - 12.0) * 0.30)
            hold = max(4.0, total - 12.0 - 2 * ramp)
        start = max(6.0, min(total * 0.20, total - (2 * ramp + hold) - 6.0))
        s1, s2, s3 = start, start + ramp, start + ramp + hold
        s4 = min(total - 4.0, s3 + ramp)      # 注意是 s3 + ramp（s3 = s2 + hold）
        # 整段校验（实车教训 2026-10-25）：只在 45% 处取一个点会被「段首是双黄、
        # 后面才变虚线」骗过去，车就压了双黄。这里沿整个变道窗口每 3 m 复核一次，
        # 任何一点不是可跨虚线（或对面是反向车道）就整段放弃。
        bad_at = self._first_block(cum, s1, s4)
        if bad_at is not None:
            if self.log:
                self.log(f"[lane] s={s1:.0f}~{s4:.0f} 窗口里 s={bad_at:.0f} 不可跨"
                         f"（实线/双黄/对向），整段放弃变道")
            return
        self.window = (s1, s3, s4)
        for i in range(n):
            s = cum[i]
            if s <= s1 or s >= s4:
                u = 0.0
            elif s <= s2:
                u = 0.5 - 0.5 * math.cos(math.pi * (s - s1) / max(1e-6, s2 - s1))
            elif s <= s3:
                u = 1.0
            else:
                u = 0.5 + 0.5 * math.cos(math.pi * (s - s3) / max(1e-6, s4 - s3))
            self.offsets[i] = float(self.side) * self.shift * u
            if u > 0.0:
                nx, ny = _normal(self.base, i)
                self.path[i] = Pt(self.base[i].x + nx * self.offsets[i],
                                  self.base[i].y + ny * self.offsets[i])
        self.plan_ok = True
        if self.log:
            self.log(f"[lane] 变道 {'左' if self.side > 0 else '右'} {self.shift:.2f} m，"
                     f"s={s1:.0f}~{s4:.0f}（ramp {ramp:.0f} m ×2，hold {hold:.0f} m）")

    def _first_block(self, cum, s1: float, s4: float, step: float = 3.0):
        """沿 [s1, s4] 每 step 米复核一次：该点还能不能往 self.side 变道。

        返回第一个不行的里程（float），全段都行返回 None。
        """
        if self.corridor is None or not self.plan_ok_side():
            return s1
        s = s1
        while s <= s4 + 1e-6:
            i = 0
            for k, c in enumerate(cum):
                if c >= s:
                    i = k
                    break
            p = self.base[i]
            if self.corridor.neighbor(p.x, p.y, self.side) is None:
                return s
            s += step
        return None

    def plan_ok_side(self) -> bool:
        return self.side != 0

    # ------------------------------------------------------------------ 接口
    def sync(self, x: float, y: float) -> None:
        if PurePursuit is None or len(self.path) < 2:
            return
        if self.pursuit is None:
            self.pursuit = PurePursuit(self.path, lookahead=C.LANE_CHANGE_LOOKAHEAD_M)
        self.pursuit.sync(x, y, max_jump_m=1e9)

    def reset(self) -> None:
        self.pursuit = None
        self.watch_t = 0.0

    def _lateral(self, x: float, y: float) -> float:
        """车相对计划路径的有符号横向偏差（正 = 路径左侧）。"""
        best_i, best_d = 0, 1e18
        for i, p in enumerate(self.path):
            d = (p.x - x) ** 2 + (p.y - y) ** 2
            if d < best_d:
                best_i, best_d = i, d
        nx, ny = _normal(self.path, best_i)
        p = self.path[best_i]
        return (x - p.x) * nx + (y - p.y) * ny

    def step(self, ctx: ElementContext) -> Command | None:
        if not self.plan_ok or PurePursuit is None:
            return None
        # 跑偏看门狗（实车教训 2026-10-25）：目标车道若判断错，车会一路斜出去。
        # 偏离计划路径 1.5 m 以上持续 0.8 s 就整段放弃，把方向盘交回基线。
        lat = self._lateral(ctx.x, ctx.y)
        if abs(lat) > 1.5:
            self.watch_t += float(ctx.dt or 0.0)
            if self.watch_t > 0.8:
                if self.log:
                    self.log(f"[lane] 偏离计划路径 {lat:+.2f} m 超过 0.8 s，放弃变道交回基线")
                self.plan_ok = False
                return None
        else:
            self.watch_t = 0.0
        if self.pursuit is None:
            self.pursuit = PurePursuit(self.path, lookahead=C.LANE_CHANGE_LOOKAHEAD_M)
            self.pursuit.sync(ctx.x, ctx.y, max_jump_m=1e9)
        steer = float(self.pursuit.steering(ctx.x, ctx.y, ctx.yaw))
        return Command(steer=steer, note=f"lane{self.side:+d}:{self.shift:.2f}")
