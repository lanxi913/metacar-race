"""环岛（元素 3）的曲率限速剖面。

为什么单独一个模块
------------------
元素 3 长 497.6 m，半径中位 **121 m**——它压根不是一个急弯，而是绕一大圈，只有两处
真急弯：入口 S 弯 s≈1197（R≈7.4~23 m）、出口 s≈1565（R≈7.7 m），R<50 m 的路段只占 8%。

2026-10-25 的 curve-r14 就是栽在这 8% 上：9.10 m/s 冲进入口，纯跟踪给出满舵 0.73，
横向跑到 −1.71 m（压线 −20），同时侧向加速度 ≈5.6 m/s²（超平台 5.0 → 丢 +10）。

切弯在这条元素里救不了（内侧净空 −1.06 / −0.54 / −0.43 m，4 段只切成 1 段 18 m），
所以这里走另一条路：**按当地曲率给限速**，`v_cap = sqrt(a_lat × R)`，
再用滑行反推减速点，保证减速不产生新的加速度事件（不踩刹车、不给油门阶跃）。

用法
----
`RingProfile(static.route, s0, s1)` 一次算好整段剖面（元素内每点一个 cap），
运行期只查 `cap_at(全局里程 s)`。半径用前后 ±10 点做最小二乘圆拟合——
三点外接圆在 1 m 抽稀的折线上会算出 8~14 m 的假半径（2026-10-25 实测）。
"""

from __future__ import annotations

import math

from . import config as C


def _xy(p):
    """兼容 Vector3 / dict（mapfile 里的 {"x","y","z"}）/ (x, y) 元组。"""
    if hasattr(p, "x"):
        return float(p.x), float(p.y)
    if isinstance(p, dict):
        return float(p["x"]), float(p["y"])
    return float(p[0]), float(p[1])


def _radius(pts, i, span=10):
    """i 附近 ±span 点的最小二乘圆半径；近直线返回一个大数。"""
    lo = max(0, i - span)
    hi = min(len(pts) - 1, i + span)
    n = hi - lo + 1
    if n < 5:
        return 1e6
    mx = sum(pts[j][0] for j in range(lo, hi + 1)) / n
    my = sum(pts[j][1] for j in range(lo, hi + 1)) / n
    u = [(pts[j][0] - mx, pts[j][1] - my) for j in range(lo, hi + 1)]
    suu = sum(a * a for a, b in u)
    svv = sum(b * b for a, b in u)
    suv = sum(a * b for a, b in u)
    suuu = sum(a * a * a for a, b in u)
    svvv = sum(b * b * b for a, b in u)
    suvv = sum(a * b * b for a, b in u)
    svuu = sum(b * a * a for a, b in u)
    det = suu * svv - suv * suv
    if abs(det) < 1e-9:
        return 1e6
    uc = (svv * (suuu + suvv) - suv * (svvv + svuu)) / (2.0 * det)
    vc = (suu * (svvv + svuu) - suv * (suuu + suvv)) / (2.0 * det)
    r = sum(math.hypot(a - uc, b - vc) for a, b in u) / n
    return r if r > 0.5 else 1e6


class RingProfile:
    """元素里程区间 [s0, s1] 上的限速剖面（按全局里程索引）。"""

    def __init__(self, route, s0, s1, *,
                 a_lat: float = C.RING_LAT_BUDGET_MS2,
                 v_max: float = 9.0,
                 v_floor: float = C.RING_V_FLOOR_MS,
                 coast: float = C.RING_PROFILE_COAST_MS2,
                 span: int = 10) -> None:
        self.v_max = float(v_max)
        self.v_floor = float(v_floor)
        self.ok = False
        self.s = []
        self.caps = []
        self.length = 0.0
        self.r_min = 1e6
        pts = [_xy(p) for p in route]
        if len(pts) < 12:
            return
        cum = [0.0] * len(pts)
        for i in range(1, len(pts)):
            cum[i] = cum[i - 1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                             pts[i][1] - pts[i - 1][1])
        lo = 0
        while lo < len(pts) - 1 and cum[lo] < float(s0):
            lo += 1
        hi = len(pts) - 1
        while hi > lo and cum[hi] > float(s1):
            hi -= 1
        if hi - lo < 8:
            return
        caps = []
        for i in range(lo, hi + 1):
            r = _radius(pts, i, span)
            self.r_min = min(self.r_min, r)
            caps.append(min(self.v_max, math.sqrt(max(0.01, a_lat * r))))
        # 反向一遍：让减速点提前到「用滑行刚好能降到下一段限速」的位置
        for k in range(len(caps) - 2, -1, -1):
            ds = cum[lo + k + 1] - cum[lo + k]
            caps[k] = min(caps[k], math.sqrt(caps[k + 1] ** 2 + 2.0 * coast * ds))
        self.s = cum[lo:hi + 1]
        self.caps = caps
        self.length = self.s[-1] - self.s[0]
        self.ok = True

    def cap_at(self, s: float) -> float:
        """按全局里程取限速；区间外返回 v_max。"""
        if not self.ok:
            return self.v_max
        if s <= self.s[0]:
            return self.caps[0]
        if s >= self.s[-1]:
            return self.caps[-1]
        lo, hi = 0, len(self.s) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if self.s[mid] <= s:
                lo = mid
            else:
                hi = mid
        return self.caps[lo]

    def floor(self) -> float:
        """本段剖面允许的最低限速，用来自检（不应低于 RING_V_FLOOR_MS）。"""
        return min(self.caps) if self.caps else self.v_max


__all__ = ["RingProfile"]
