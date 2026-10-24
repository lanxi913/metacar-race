# -*- coding: utf-8 -*-
"""曲率限速剖面：把 route 折成「每点的允许速度」，再按加减速能力前后向积分。

为什么需要它：连续场景1 的五个元素里，连续转弯 857.8 m 只给 200 s，均速 4.3 m/s 才不超时；
而 8 m/s 直冲又会在 R≈5 m 的弯里压线（每类扣 20）。定速跑两头都不讨好，所以按曲率给
每一点一个速度上限，弯前提前减速、出弯再加速。

实测依据（20261024-125832 采集）：整段 1339.6 m 里 κ>0.05 只占 8.2%，最急的连续弯 R=5.0 m；
取 a_lat=2.5、a_tan=1.5、v_max=8.0 时整段 196.8 s，连续转弯 132.0 s（时限 200 s）。
"""
from __future__ import annotations

import math

DEFAULT_V_MAX = 8.0      # m/s，28.8 km/h，离 60 km/h 限速还有一倍余量
DEFAULT_A_LAT = 2.5      # m/s²，横向加速度上限（压线余量换来的）
DEFAULT_A_TAN = 1.5      # m/s²，纵向加减速上限（评分要求 |a| ≤ 5，留 3.3 倍余量）


def _xy(route):
    """兼容 Vector3 / Vector2 / 元组，统一成 (x, y) 列表。"""
    out = []
    for p in route:
        if hasattr(p, "x") and hasattr(p, "y"):
            out.append((float(p.x), float(p.y)))
        else:
            out.append((float(p[0]), float(p[1])))
    return out


def arc_lengths(route):
    """逐点累计弧长，返回 (s 列表, 总长)。"""
    pts = _xy(route)
    s = [0.0]
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    return s, (s[-1] if s else 0.0)


def curvature(route, window=2):
    """三点外接圆曲率，window 为前后各取几个点。

    路点间距约 1 m，window=2 相当于 ±2 m 的平滑，能把定位毛刺压掉又不至于削平发夹弯。
    """
    pts = _xy(route)
    n = len(pts)
    k = []
    for i in range(n):
        j = max(0, i - window)
        m = min(n - 1, i + window)
        if m - j < 2:
            k.append(0.0)
            continue
        (x1, y1), (x2, y2), (x3, y3) = pts[j], pts[i], pts[m]
        a = math.hypot(x2 - x1, y2 - y1)
        b = math.hypot(x3 - x2, y3 - y2)
        c = math.hypot(x3 - x1, y3 - y1)
        if a * b * c <= 0.0:
            k.append(0.0)
            continue
        area2 = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))
        k.append(2.0 * area2 / (a * b * c))
    return k


def speed_limit(route, v_max=DEFAULT_V_MAX, a_lat=DEFAULT_A_LAT, a_tan=DEFAULT_A_TAN,
                window=2):
    """返回逐点允许速度：min(v_max, sqrt(a_lat/κ))，再按 a_tan 做前后向积分。

    前向保证起步段不会要求它做不到的加速，后向保证弯前留足刹车距离。
    """
    k = curvature(route, window=window)
    v = [min(float(v_max), math.sqrt(a_lat / max(ki, 1e-9))) for ki in k]
    if len(v) < 2:
        return v

    s, _ = arc_lengths(route)
    for i in range(1, len(v)):
        ds = max(s[i] - s[i - 1], 1e-6)
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_tan * ds))
    for i in range(len(v) - 2, -1, -1):
        ds = max(s[i + 1] - s[i], 1e-6)
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_tan * ds))
    return v


class SpeedProfile:
    """剖面查询：给当前路点索引和当前车速，返回此刻允许的目标速度。"""

    def __init__(self, route, v_max=DEFAULT_V_MAX, a_lat=DEFAULT_A_LAT,
                 a_tan=DEFAULT_A_TAN, window=2):
        self.route = list(route)
        self.v_max = float(v_max)
        self.a_lat = float(a_lat)
        self.a_tan = float(a_tan)
        self.s, self.length = arc_lengths(self.route)
        self.v = speed_limit(self.route, v_max, a_lat, a_tan, window=window)
        self.idx = 0

    def __len__(self):
        return len(self.v)

    def reset(self, idx=0):
        self.idx = int(idx)

    def target(self, idx, v_now, margin_m=2.0):
        """巡航目标：当前点速度，以及「刹车距离内」所有点的最低速度，取更小者。

        v_now 越大学到的刹车距离越长，因此进弯前会提前松油，而不是到了弯口才急刹。
        """
        n = len(self.v)
        if n == 0:
            return 0.0
        i = max(0, min(int(idx), n - 1))
        v_here = self.v[i]
        if v_now is None or v_now <= 0.0:
            return v_here
        horizon = self.s[i] + v_now * v_now / (2.0 * self.a_tan) + margin_m
        j = i
        v_min = v_here
        while j < n and self.s[j] <= horizon:
            if self.v[j] < v_min:
                v_min = self.v[j]
            j += 1
        return v_min
