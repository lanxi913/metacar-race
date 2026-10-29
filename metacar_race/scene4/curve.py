# -*- coding: utf-8 -*-
"""弯道切内弯（apron / racing line 的最简版本）。

用户要求（2026-10-25 12:xx）：

* 起步段继续用已验证的定油门代码，**不动**；
* 弯道「切入内弯」：入口转向偏角更大 → 弯心附近近似直线 → 出口回正；
* 允许自由变道，但只守两条：**不跨双黄实线（中线）**、**不压路边沿线（侧线）**；
* 车辆左右宽度要算进净空里。

实现方式（不是又一个速度环，也不写状态机）：

1. 用 `LaneCorridor` 把每一帧的左右净空算出来（虚线可以跨，实线/中线/侧线不行，
   已扣半个车宽 + 安全余量）；
2. 沿路线求曲率，找出「连续的弯」（半径 < `R_THRESH` 且同号），弯心那一侧就是内侧；
3. 给这段弯生成一条**横向偏移曲线** `o(s) = A·sin(π·u)`：入口 0 → 弯心 A → 出口 0。
   这条曲线本身就是一条弦，跟着它走自然就是「偏角大 → 近似直线 → 回正」；
4. `A` 取这段弯内侧净空的最小值（再夹到 `MAX_CUT_M`），所以不会因为切得太多去压线；
5. 用 `PurePursuit` 跟踪这条偏移后的折线，**只接管方向盘**，纵向仍归基线
   （环岛那次教训：本车油门是开关型的，第二个速度环一定抽搐）。

可验证性：`run.csv` 里有 x/y/yaw/steer，跑完把轨迹和原路线做横向投影，
就能直接量出「实际切进去多少米」，不需要看画面。
"""
from __future__ import annotations

import math

from . import config as C
from .corridor import LaneCorridor
from .lateral import Pt
from .longitudinal import StraightRun
from .strategies import Command, ElementContext, Strategy

try:
    from metacar_race.control.tracking import PurePursuit
    _PP_OK = True
except Exception:  # noqa: BLE001
    PurePursuit = None          # type: ignore[assignment]
    _PP_OK = False

R_THRESH = 260.0        #: 半径小于这个值才算「弯」，需要切
#: 单次最多往里切多少。2026-10-25 第一轮实测后从 2.40 下调到 1.60：
#: 计划切 2.40 m，`run.csv` 里的实际横向却到了 **+2.90 m**（跟踪滞后 + 弯心超调），
#: 多出来 0.5 m。按「实际 ≈ 计划 + 0.5」反推，计划值必须留出这段余量，
#: 否则车的一侧就会压到双黄实线或路边沿线。
MAX_CUT_M = 1.60
#: 小于这个量就别切了。0.12 会切出一堆 0.18/0.30 的碎段（实测八段里三段是这种），
#: 既省不了时间又增加横向抖动，抬到 0.50。
MIN_CUT_M = 0.50
#: 实测「实际横向 − 计划横向」的系统性超调（curve-r5：2.90 − 2.40 = 0.50 m）
CUT_OVERSHOOT_M = 0.50
#: 再留一道安全余量（车宽已由 corridor 扣过，这里防的是跟踪抖动）
CUT_SAFETY_M = 0.25
#: 运行中实测横向超出计划这么多、且持续 CUT_ABORT_HOLD_S，就取消这一段切弯
CUT_ABORT_M = 0.40
CUT_ABORT_HOLD_S = 0.60
LOOKAHEAD_M = 6.0       #: 跟踪偏移线的前视距离
SMOOTH_WIN = 5          #: 偏移曲线滑动平均窗口（点，1 点≈1 m）


def _cum(pts) -> list[float]:
    out = [0.0]
    for a, b in zip(pts, pts[1:]):
        out.append(out[-1] + math.hypot(b.x - a.x, b.y - a.y))
    return out


def _curvature(pts, i: int, span: int = 10):
    """半径与转向符号：符号 +1 = 左转（圆心在左侧）。

    用**最小二乘圆拟合**而不是三点外接圆：导出的 route 是抽稀过的折线（点距 1 m、
    转角处有折点），三点法在折点附近会算出 8~14 m 的假半径（实测过），拟合窗口
    ±10 m 就稳定得多，和真实弯道半径（40~140 m）对得上。
    """
    a = max(0, i - span)
    b = min(len(pts) - 1, i + span)
    P = pts[a:b + 1]
    n = len(P)
    if n < 5:
        return 1e9, 0
    mx = sum(p.x for p in P) / n
    my = sum(p.y for p in P) / n
    u = [(p.x - mx, p.y - my) for p in P]
    Suu = sum(q[0] * q[0] for q in u)
    Svv = sum(q[1] * q[1] for q in u)
    Suv = sum(q[0] * q[1] for q in u)
    Suuu = sum(q[0] ** 3 for q in u)
    Svvv = sum(q[1] ** 3 for q in u)
    Suvv = sum(q[0] * q[1] * q[1] for q in u)
    Svuu = sum(q[1] * q[0] * q[0] for q in u)
    det = Suu * Svv - Suv * Suv
    if abs(det) < 1e-9:
        return 1e9, 0
    rhs1 = (Suuu + Suvv) / 2.0
    rhs2 = (Svvv + Svuu) / 2.0
    uc = (rhs1 * Svv - rhs2 * Suv) / det
    vc = (Suu * rhs2 - Suv * rhs1) / det
    R = math.hypot(uc, vc)
    if R <= 0.0 or not math.isfinite(R):
        return 1e9, 0
    # 方向：圆心在航向左侧 → 左转
    if i == 0:
        tx, ty = pts[1].x - pts[0].x, pts[1].y - pts[0].y
    elif i >= len(pts) - 1:
        tx, ty = pts[-1].x - pts[-2].x, pts[-1].y - pts[-2].y
    else:
        tx, ty = pts[i + 1].x - pts[i - 1].x, pts[i + 1].y - pts[i - 1].y
    # 圆心相对当前点的方向：左法向分量为正即圆心在左
    cx, cy = mx + uc, my + vc
    left_x, left_y = -ty, tx
    dot = (cx - pts[i].x) * left_x + (cy - pts[i].y) * left_y
    return R, (1 if dot >= 0 else -1)


def _regions(pts, r_thresh: float = R_THRESH, min_len_m: float = 12.0,
             gap_m: float = 25.0):
    """找连续的弯：返回 [(i0, i1, sign)]。

    真实路线上「弯」不是一整段恒定的曲率：90° 拐角只有几十米，中间还夹着几米
    近似直的点。所以按**长度和间隙**合并，而不是要求逐点同号：

    * 同号继续走；
    * 遇到 0（不够弯）时，如果 `gap_m` 内又回到同号，就当作同一段弯；
    * 整段长度 ≥ `min_len_m` 才认。
    """
    n = len(pts)
    if n < 3:
        return []
    cum = _cum(pts)
    flags = []
    for i in range(n):
        r, sign = _curvature(pts, i)
        flags.append(sign if r < r_thresh else 0)

    out = []
    i = 0
    while i < n:
        if flags[i] == 0:
            i += 1
            continue
        sign = flags[i]
        j = i
        while j + 1 < n:
            if flags[j + 1] == sign:
                j += 1
                continue
            if flags[j + 1] == 0:
                k = j + 1
                while k < n and flags[k] == 0:
                    k += 1
                if k < n and flags[k] == sign and cum[k] - cum[j] <= gap_m:
                    j = k
                    continue
            break
        if cum[j] - cum[i] >= min_len_m:
            out.append((i, j, sign))
        i = j + 1
    return out


class CurveMap:
    """路线上「弯区段」的里程索引：回答「前方多远开始弯」。

    只做几何查询，不产生控制量。直道纵向规划（`StraightRun`）用它反推松油点。
    """

    def __init__(self, route, r_thresh: float = R_THRESH, min_len_m: float = 12.0) -> None:
        self.spans: list[tuple[float, float, int]] = []
        if route is not None and len(route) >= 6:
            cum = _cum(route)
            for (i0, i1, sign) in _regions(route, r_thresh=r_thresh, min_len_m=min_len_m):
                self.spans.append((cum[i0], cum[i1], sign))

    def dist_to_next(self, s: float) -> float | None:
        """到下一个弯**入口**的距离；已经在弯里返回 0；前方无弯返回 None。"""
        s = float(s)
        for (a, b, _sign) in self.spans:
            if s < a:
                return max(0.0, a - s)
            if a <= s <= b:
                return 0.0
        return None

    def in_curve(self, s: float) -> bool:
        s = float(s)
        return any(a <= s <= b for (a, b, _sign) in self.spans)


class CurveCutStrategy(Strategy):
    """连续转弯：切内弯，只给方向盘。"""

    key = "curve"
    label = "切内弯"

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        super().__init__(ctx, route, log)
        self.corridor = None
        if ctx.static is not None:
            try:
                self.corridor = LaneCorridor(ctx.static,
                                             ego_width=float(ctx.ego_width or 1.81),
                                             margin=0.25)
            except Exception as exc:  # noqa: BLE001
                if self.log:
                    self.log(f"[curve] 车道走廊构造失败，只跟原路线: {exc!r}")
                self.corridor = None
        self.base: list[Pt] = list(route)
        self.path: list[Pt] = list(route)
        self.offsets: list[float] = [0.0] * len(self.base)
        self.regions: list[dict] = []
        #: 被运行期保护取消掉的弯区段（下标指向**已识别的全部弯**，而不是 self.regions）
        self.disabled: set[int] = set()
        self._abort_t: dict[int, float] = {}
        self._build(self.base)
        #: 直道纵向：弯前用「松油滑行」减速，不踩刹车（里程按**整条路线**算）
        base = getattr(getattr(ctx, "static", None), "route", None) or route
        self.cmap = CurveMap(base)
        self.run = StraightRun(v_cap=float(ctx.vmax or 8.0))
        self.pursuit = None
        if _PP_OK and len(self.path) >= 2:
            self.pursuit = PurePursuit(self.path, lookahead=LOOKAHEAD_M)
            self.pursuit.idx = 0

    # ------------------------------------------------------------------ 建线
    def _room_inside(self, x: float, y: float, sign: int) -> float:
        """内侧还能挪多少（已扣车宽/余量）；查不到车道就返回 0（不切）。"""
        if self.corridor is None:
            return 0.0
        room = self.corridor.room(x, y)
        if room is None:
            return 0.0
        return float(room.left if sign > 0 else room.right)

    def _tangent(self, route, i: int):
        n = len(route)
        if i <= 0:
            return route[1].x - route[0].x, route[1].y - route[0].y
        if i >= n - 1:
            return route[-1].x - route[-2].x, route[-1].y - route[-2].y
        return route[i + 1].x - route[i - 1].x, route[i + 1].y - route[i - 1].y

    def _build(self, route) -> None:
        n = len(route)
        if n < 6:
            return
        cum = _cum(route)
        offsets = [0.0] * n
        self.regions = []
        for ridx, (i0, i1, sign) in enumerate(_regions(route)):
            if ridx in self.disabled:
                continue
            # 这段弯内侧净空的最小值决定切多深；实际横向会比计划多走
            # CUT_OVERSHOOT_M（curve-r5 实测），所以先扣掉超调和安全余量。
            room = min(self._room_inside(route[i].x, route[i].y, sign)
                       for i in range(i0, i1 + 1))
            avail = room - CUT_OVERSHOOT_M - CUT_SAFETY_M
            amp = max(0.0, min(MAX_CUT_M, avail))
            if amp < MIN_CUT_M:
                if self.log:
                    self.log(f"[curve] s={cum[i0]:.0f}~{cum[i1]:.0f} 内侧净空 {room:.2f} m"
                             f"（扣超调/余量后只剩 {max(0.0, avail):.2f} m），不切")
                continue
            r_min = min(_curvature(route, i)[0] for i in range(i0, i1 + 1))
            self.regions.append({"ridx": ridx, "i0": i0, "i1": i1,
                                 "s0": cum[i0], "s1": cum[i1], "sign": sign,
                                 "amp": amp, "room": room, "r_min": r_min})
            if self.log:
                self.log(f"[curve] 切弯 s={cum[i0]:.0f}~{cum[i1]:.0f} "
                         f"长{cum[i1]-cum[i0]:.0f} m Rmin={r_min:.0f} m "
                         f"{'左' if sign > 0 else '右'}内切 {amp:.2f} m（净空 {room:.2f}）")
            span = max(1e-6, cum[i1] - cum[i0])
            for i in range(i0, i1 + 1):
                u = (cum[i] - cum[i0]) / span
                offsets[i] = sign * amp * math.sin(math.pi * u)

        # 滑动平均，去掉起止处的折角
        if n > SMOOTH_WIN:
            sm = []
            half = SMOOTH_WIN // 2
            for i in range(n):
                a = max(0, i - half)
                b = min(n, i + half + 1)
                sm.append(sum(offsets[a:b]) / (b - a))
            offsets = sm

        path = []
        for i in range(n):
            tx, ty = self._tangent(route, i)
            L = math.hypot(tx, ty) or 1.0
            nx, ny = -ty / L, tx / L      # 左法向
            path.append(Pt(route[i].x + nx * offsets[i],
                           route[i].y + ny * offsets[i]))
        self.offsets = offsets
        self.path = path
        if _PP_OK and len(path) >= 2:
            self.pursuit = PurePursuit(path, lookahead=LOOKAHEAD_M)

    # ------------------------------------------------------------------ 帧回调
    def sync(self, x: float, y: float) -> None:
        if self.pursuit is not None:
            self.pursuit.sync(x, y, max_jump_m=1e9)

    def reset(self) -> None:
        if self.pursuit is not None:
            self.pursuit.idx = 0

    def _overshoot_guard(self, ctx) -> None:
        """运行期保护：实测横向比计划多出太多就取消这一段切弯。

        计划值已经预先扣掉 `CUT_OVERSHOOT_M`，但跟踪滞后随曲率/车速变化，
        所以再加一道实测闭环：只要超出 `CUT_ABORT_M` 且持续 `CUT_ABORT_HOLD_S`，
        就把这一段从计划里摘掉、重建偏移线。宁可不切，也不压双黄实线或路边沿线。
        """
        if self.pursuit is None or not self.regions or not self.base:
            return
        i = int(getattr(self.pursuit, "idx", 0) or 0)
        i = max(0, min(len(self.base) - 1, min(len(self.offsets) - 1, i)))
        hit = None
        for r in self.regions:
            if r["i0"] <= i <= r["i1"]:
                hit = r
                break
        if hit is None:
            return
        p = self.base[i]
        tx, ty = self._tangent(self.base, i)
        L = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / L, tx / L
        lat = (float(ctx.x) - p.x) * nx + (float(ctx.y) - p.y) * ny
        planned = float(self.offsets[i])
        over = abs(lat) - abs(planned)
        ridx = int(hit["ridx"])
        if over <= CUT_ABORT_M:
            self._abort_t.pop(ridx, None)
            return
        now = float(getattr(ctx, "sim_t", 0.0) or 0.0)
        t0 = self._abort_t.get(ridx)
        if t0 is None:
            self._abort_t[ridx] = now
            return
        if now - t0 < CUT_ABORT_HOLD_S:
            return
        self.disabled.add(ridx)
        self._abort_t.pop(ridx, None)
        if self.log:
            self.log(f"[curve] 实测横向 {lat:+.2f} m 超出计划 {planned:+.2f} m"
                     f"（多 {over:.2f} m > {CUT_ABORT_M:.2f} m）持续 "
                     f"{CUT_ABORT_HOLD_S:.1f} s，取消该段切弯")
        self._build(self.base)
        if self.pursuit is not None:
            self.pursuit.sync(float(ctx.x), float(ctx.y), max_jump_m=1e9)

    def step(self, ctx: ElementContext) -> Command | None:
        self._overshoot_guard(ctx)
        if self.pursuit is None:
            return None
        steer = float(self.pursuit.steering(ctx.x, ctx.y, ctx.yaw))
        amp = self.regions[0]["amp"] if len(self.regions) == 1 else sum(
            r["amp"] for r in self.regions) / max(1, len(self.regions))

        # ---- 纵向：只在「快到弯」时介入，且只松油不刹车 ------------------
        # 弯里（d == 0）与前方无弯（d is None）都交回基线，避免第二个速度环。
        d = self.cmap.dist_to_next(ctx.s_now)
        v_cap = None
        coast = None
        note = "cut:%.2f" % amp
        if d is not None and d > 0.0:
            if d < C.COAST_LOOKAHEAD_MIN_M:
                d = 0.0
            else:
                allow = self.run.allow_speed(d)
                v_cap = allow
                if abs(float(ctx.speed)) > allow + C.COAST_MARGIN_MS:
                    coast = True
                    note = "cut:%.2f coast d=%.0f" % (amp, d)
                else:
                    note = "cut:%.2f d=%.0f" % (amp, d)
        return Command(steer=steer, v_cap=v_cap, coast=coast, note=note)


__all__ = ["CurveCutStrategy", "R_THRESH", "MAX_CUT_M"]
