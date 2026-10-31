"""横向：Stanley 跟踪、可横移路线、超车并线状态机。

`PurePursuit` 直接复用 `metacar_race.control.tracking`（弯道 110 分的那个实现）；
这里补三样东西：

* `Stanley`：环岛这类曲率大的路段，PurePursuit 6 m 前视会内切 1.2 m 以上，
  实测横向 2.02 m 直接压边线；Stanley 用"横向偏差 + 航向误差"反馈，不会切弧。
* `ShiftRoute`：只在车附近一段改路点，不每帧重建 4700 个点（超车并线用）。
* `Overtaker`：左道超车状态机，三条硬约束（只往左并、横纵不同时做、回中要等两件事）
  写在类 docstring 里，改动前先读一遍。
"""

from __future__ import annotations

import math

from . import config as C
from .track import wrap_deg


class Stanley:
    """折线路径上的 Stanley 跟踪（移植自 `roundabout/main.py`，那里实测环岛 110 分）。

    转向约定与 PurePursuit 一致：steering > 0 = 往右打（场景 yaw 顺时针为正）。
    """

    def __init__(self, route, k_heading: float = C.K_HEADING,
                 k_cross: float = C.K_CROSS, smooth: int = 3) -> None:
        self.route = route
        self.k_h = float(k_heading)
        self.k_c = float(k_cross)
        self.smooth = int(smooth)
        self.speed_ref = 0.0
        self.idx = 0

    def reset(self, route=None) -> None:
        if route is not None:
            self.route = route
        self.idx = 0
        self.speed_ref = 0.0

    def _pt(self, i: int) -> tuple[float, float]:
        n = len(self.route)
        p = self.route[max(0, min(n - 1, i))]
        return float(p.x), float(p.y)

    def sync(self, x: float, y: float, max_jump_m: float = 1.0) -> int:
        self._nearest(x, y)
        return self.idx

    def _nearest(self, x: float, y: float) -> int:
        n = len(self.route)
        if n == 0:
            return 0
        lo = max(0, self.idx - 10)
        hi = min(n - 1, self.idx + 40)
        best, bd = lo, float("inf")
        for i in range(lo, hi + 1):
            px, py = self._pt(i)
            d = math.hypot(px - x, py - y)
            if d < bd:
                best, bd = i, d
        if bd > 10.0:                      # 跟丢了就回全局搜索重新对齐
            best = min(range(n), key=lambda k: math.hypot(
                self._pt(k)[0] - x, self._pt(k)[1] - y))
        self.idx = best                    # 允许回退，只增不减会漂
        return self.idx

    def steering(self, x: float, y: float, yaw_deg: float) -> float:
        n = len(self.route)
        if n < 2:
            return 0.0
        i = self._nearest(x, y)
        a = max(0, i - self.smooth)
        b = min(n - 1, i + self.smooth)
        ax, ay = self._pt(a)
        bx, by = self._pt(b)
        tx, ty = bx - ax, by - ay
        m = math.hypot(tx, ty)
        if m < 1e-6:
            return 0.0
        tx, ty = tx / m, ty / m
        px, py = self._pt(i)
        lat = (x - px) * (-ty) + (y - py) * tx        # 车在路径左侧为正
        yaw_ref = -math.degrees(math.atan2(ty, tx))
        # 横向项：车偏左(lat>0)就要往右打，而本平台 steering>0 是右转，所以取 +lat。
        # 取 -lat 会让偏差发散（闭环仿真 60 s 漂到 350 m，取 +lat 稳定在 0.25 m 内）。
        corr = math.degrees(math.atan2(self.k_c * lat,
                                       max(abs(self.speed_ref), 0.8)))
        err = wrap_deg(yaw_ref + corr - yaw_deg)
        return max(-1.0, min(1.0, self.k_h * math.radians(err)))


class Pt:
    """只带 x/y 的轻量路点，给 PurePursuit 用。"""

    __slots__ = ("x", "y")

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y


class ShiftRoute:
    """可临时横移的路线：只改车辆附近一段，不每帧重建 4700 个点。"""

    def __init__(self, route) -> None:
        self.base = [(float(p.x), float(p.y)) for p in route]
        self.n = len(self.base)
        self.items = [Pt(bx, by) for bx, by in self.base]
        self.nx = [0.0] * self.n
        self.ny = [0.0] * self.n
        for k in range(self.n):
            a = self.base[max(0, k - 1)]
            b = self.base[min(self.n - 1, k + 1)]
            tx, ty = b[0] - a[0], b[1] - a[1]
            m = math.hypot(tx, ty)
            if m < 1e-6:
                continue
            self.nx[k] = -ty / m      # 左法向
            self.ny[k] = tx / m

    def apply(self, off: float, i_now: int, span: int = 90) -> list[Pt]:
        lo = max(0, i_now - 12)
        hi = min(self.n, i_now + span)
        for k in range(lo, hi):
            bx, by = self.base[k]
            self.items[k].x = bx + self.nx[k] * off
            self.items[k].y = by + self.ny[k] * off
        return self.items


class Overtaker:
    """左道超车状态机：CRUISE → SHIFT_OUT → PASS → SHIFT_BACK → CRUISE。

    **先向左变道 → 提速 → 超越右侧车辆 → 向右变道回来**，全程不撞车。三条硬约束：

    1. **只往左并**。左道不空就老实跟车，绝不往右并——场景 4 这一段是四辆车错位
       停放（本道/右道交替、跨度 34 m），右道同样有车，并到右侧 4 m 就顶上第二辆。
    2. **横向和纵向不同时做**。横移用余弦 S 曲线，起止横向速度都是 0；横移期间
       速度保持不变（`v_hold`），提速只放在直线段（PASS）。这样 |a_lat| 峰值
       (A/2)(π/T)² ≈ 2.8 m/s² 与纵向 1.5 m/s² 不叠加，离平台 5 m/s² 还有一倍余量。
    3. **回中间要等两件事**：本道前方 `RETURN_CLEAR_M` 内无车，且最后一辆被超的车
       已经在身后 `RETURN_BEHIND_M` 以上（那排车跨度 34 m，只看 14 m 会插回第二辆
       车前面）。

    状态机是显式的：每帧只推进一步。不要改成"每 0.2 s 按净空重挑一次车道"——
    那种写法会在换道半途改主意，车会左右摆。
    """

    CRUISE, SHIFT_OUT, PASS, SHIFT_BACK = 0, 1, 2, 3
    NAMES = ("CRUISE", "SHIFT_OUT", "PASS", "SHIFT_BACK")

    def __init__(self, offset: float = C.OVERTAKE_OFFSET_M, enabled: bool = True,
                 shift_seconds: float = C.SHIFT_SECONDS) -> None:
        self.offset = float(offset)
        self.enabled = bool(enabled)
        self.shift_seconds = max(0.2, float(shift_seconds))
        self.state = self.CRUISE
        self.t_state = 0.0
        self.lat_off = 0.0
        self.from_off = 0.0
        self.shift_dist = 0.0
        self.v_hold: float | None = None
        self.last_block = None

    def reset(self, t: float = 0.0) -> None:
        """元素重开/平台瞬移后复位：回 CRUISE，横移与锁速全部清零。"""
        self.state = self.CRUISE
        self.t_state = float(t)
        self.lat_off = 0.0
        self.from_off = 0.0
        self.shift_dist = 0.0
        self.v_hold = None
        self.last_block = None

    # ---------------- 车道占用查询 ----------------

    @staticmethod
    def _usable(o) -> bool:
        return o["type"] not in C.IGNORE_TYPES and o["dist"] > -200.0

    def _in_band(self, o, off: float) -> bool:
        return abs(o["lat"] - off) <= C.OVERTAKE_BAND_M

    def nearest_forward(self, items, off: float):
        """这条车道正前方最近的障碍（dict）或 None。"""
        best = None
        for o in items:
            if not self._usable(o) or not self._in_band(o, off):
                continue
            if o["dist"] < 0.0:
                continue
            if best is None or o["dist"] < best["dist"]:
                best = o
        return best

    def clearance(self, items, off: float) -> float:
        """这条车道正前方净空（`LANE_MAX_M` 封顶）。"""
        near = self.nearest_forward(items, off)
        return C.LANE_MAX_M if near is None else min(C.LANE_MAX_M, near["dist"])

    def beside(self, items, off: float, gap: float = 3.0):
        """这条车道上紧贴车身（±gap）的障碍，用于判断"并进去会不会撞"。"""
        for o in items:
            if not self._usable(o) or not self._in_band(o, off):
                continue
            if -gap <= o["dist"] < gap:
                return o
        return None

    def rear_gap(self, items, off: float) -> float:
        """这条车道**身后**最近的障碍还差多少米（没有则 999）。"""
        best = 999.0
        for o in items:
            if not self._usable(o) or not self._in_band(o, off):
                continue
            if o["dist"] <= 0.0:
                best = min(best, -o["dist"])
        return best

    # ---------------- 状态推进 ----------------

    def _enter(self, state: int, t: float, speed: float) -> None:
        self.state = state
        self.t_state = float(t)
        self.from_off = self.lat_off
        if state in (self.SHIFT_OUT, self.SHIFT_BACK):
            self.shift_dist = 0.0
            # 高速时锁速横移（不同时加速）；低速时横向加速度按 v² 缩，允许继续提速
            self.v_hold = float(speed) if speed >= C.SHIFT_HOLD_MIN_MS else None
        if state == self.CRUISE:
            self.v_hold = None

    def _cosine(self, t: float) -> float:
        """余弦 S 曲线：起止横向速度都是 0，没有横向加速度冲量。

        按**走过的纵向距离**推进而不是墙钟时间：车速低时横移自动拉长，不会出现
        "低速时要在 3 m 行程里横移 3.5 m"这种物理上做不到的动作。
        """
        u = self.shift_dist / max(C.SHIFT_LENGTH_M, 0.1)
        u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
        s = 0.5 * (1.0 - math.cos(math.pi * u))
        dst = self.offset if self.state == self.SHIFT_OUT else 0.0
        return self.from_off + (dst - self.from_off) * s

    def _shift_done(self, el: float) -> bool:
        return self.shift_dist >= C.SHIFT_LENGTH_M or el >= C.SHIFT_TIMEOUT_S

    def _step(self, t: float, items, speed: float) -> None:
        st = self.state
        el = t - self.t_state

        if st == self.CRUISE:
            self.lat_off = 0.0
            self.v_hold = None
            near = self.nearest_forward(items, 0.0)
            if near is None or near["v"] >= C.LANE_MOVING_MS:
                return                       # 前车在动 → 跟车，不算障碍
            if near["dist"] > C.OVERTAKE_PREP_M:
                return                       # 还远，先不提速也别刹车
            if self.clearance(items, self.offset) < C.PASS_CLEAR_M:
                return                       # 左道不够空
            if self.beside(items, self.offset) is not None:
                return
            self.last_block = near
            self._enter(self.SHIFT_OUT, t, speed)
            return

        if st == self.SHIFT_OUT:
            if (self.beside(items, self.offset) is not None
                    or self.clearance(items, self.offset) < C.ABORT_GAP_M):
                self._enter(self.SHIFT_BACK, t, speed)   # 目标道被占，原路退回
                return
            self.lat_off = self._cosine(t)
            if self._shift_done(el):
                self.lat_off = self.offset
                self._enter(self.PASS, t, speed)
            return

        if st == self.PASS:
            self.lat_off = self.offset
            if (self.clearance(items, 0.0) >= C.RETURN_CLEAR_M
                    and self.rear_gap(items, 0.0) >= C.RETURN_BEHIND_M):
                self._enter(self.SHIFT_BACK, t, speed)
            elif el >= C.PASS_MAX_S and self.clearance(items, 0.0) >= C.ABORT_GAP_M:
                self._enter(self.SHIFT_BACK, t, speed)   # 兜底：待太久影响后面转弯
            return

        if st == self.SHIFT_BACK:
            if (self.beside(items, 0.0) is not None
                    or self.clearance(items, 0.0) < C.ABORT_GAP_M):
                self._enter(self.SHIFT_OUT, t, speed)    # 本道又堵，回左侧
                return
            self.lat_off = self._cosine(t)
            if self._shift_done(el):
                self.lat_off = 0.0
                self._enter(self.CRUISE, t, speed)

    def update(self, t: float, items, dt: float, speed: float) -> dict:
        prev = self.state
        if self.enabled:
            if self.state in (self.SHIFT_OUT, self.SHIFT_BACK):
                self.shift_dist += max(float(speed), 0.0) * max(float(dt), 0.0)
            self._step(t, items, speed)
        else:
            self.state = self.CRUISE
            self.lat_off = 0.0
            self.v_hold = None
        return {
            "lat_off": self.lat_off,
            "state": self.NAMES[self.state],
            "changed": self.state != prev,
            "target": self.offset if self.state in (self.SHIFT_OUT, self.PASS) else 0.0,
            "v_hold": self.v_hold,
            "blocked": False,
        }
