"""路线几何：累计里程、局部投影、停止线/信号灯几何、车道宽。

这一层不产生控制量，只回答"车在哪、前方几何是什么"。所有函数都是纯计算（除了
`RouteTrack.idx` 这个局部缓存），方便离线冒烟测试。
"""

from __future__ import annotations

import math

from . import config as C


def wrap_deg(angle: float) -> float:
    """把角度折到 (-180, 180]。"""
    return (angle + 180.0) % 360.0 - 180.0


class RouteTrack:
    """路线几何：累计里程 + 局部索引跟踪 + 带符号横向偏差。

    `idx` 是**局部缓存**：`sync()` 只在索引附近 ±30/+400 点里找，跟丢了（>20 m）
    才退回全局搜索——这样既够快，又能在平台瞬移后自己找回来。
    """

    def __init__(self, route) -> None:
        self.pts = [(float(p.x), float(p.y)) for p in route]
        self.n = len(self.pts)
        self.s = [0.0] * self.n
        for i in range(1, self.n):
            self.s[i] = self.s[i - 1] + math.hypot(
                self.pts[i][0] - self.pts[i - 1][0],
                self.pts[i][1] - self.pts[i - 1][1])
        self.idx = 0

    @property
    def length(self) -> float:
        return self.s[-1] if self.n else 0.0

    def s_of(self, i: int) -> float:
        return self.s[max(0, min(self.n - 1, i))] if self.n else 0.0

    def sync(self, x: float, y: float) -> int:
        """就近定位，返回索引；跳关/瞬移时自动全局重找。"""
        lo = max(0, self.idx - 30)
        hi = min(self.n, self.idx + 400)
        best, bd = self.idx, float("inf")
        for i in range(lo, hi):
            px, py = self.pts[i]
            d = (px - x) ** 2 + (py - y) ** 2
            if d < bd:
                bd, best = d, i
        if bd > 400.0:                       # 20 m 以外：认为被瞬移了
            best, bd = 0, float("inf")
            for i in range(self.n):
                px, py = self.pts[i]
                d = (px - x) ** 2 + (py - y) ** 2
                if d < bd:
                    bd, best = d, i
        self.idx = best
        return best

    def tangent(self, i: int) -> tuple[float, float]:
        j = min(self.n - 1, i + 3)
        tx = self.pts[j][0] - self.pts[i][0]
        ty = self.pts[j][1] - self.pts[i][1]
        n = math.hypot(tx, ty) or 1.0
        return tx / n, ty / n

    def project(self, x: float, y: float, lo: int, hi: int) -> tuple[int, float]:
        """在 [lo, hi) 里找最近点，返回 (idx, 带符号横向偏差)。

        偏差约定：车在路线**左侧为正**（与 `lat_err` 一致）。必须带符号，
        否则左右两条相邻车道分不开（踩过一次坑）。
        """
        best, bd = lo, float("inf")
        for i in range(lo, hi):
            px, py = self.pts[i]
            d = (px - x) ** 2 + (py - y) ** 2
            if d < bd:
                bd, best = d, i
        px, py = self.pts[best]
        tx, ty = self.tangent(best)
        return best, (x - px) * (-ty) + (y - py) * tx

    def lateral_error(self, i: int, x: float, y: float) -> float:
        """相对路线第 i 点的横向偏差（左正右负）。"""
        tx, ty = self.tangent(i)
        px, py = self.pts[i]
        return (x - px) * (-ty) + (y - py) * tx

    def s_of_world(self, px: float, py: float) -> float:
        """世界坐标 → 路线累计里程（米）。"""
        j, _ = self.project(px, py, 0, self.n)
        return float(self.s[j])


def pick_stop_line(roads, x: float, y: float, yaw_deg: float,
                   max_dist: float = 80.0):
    """车正前方最近的停止线中点，返回 (距离, (sx, sy)) 或 None。

    停止线直接用 SDK 的道路数据（`RoadInfo.stop_line`），不自己猜几何——
    移植自 `crossroad/main.py`，那里实测十字路口 210 分。
    """
    psi = math.radians(yaw_deg)
    fx, fy = math.cos(psi), -math.sin(psi)        # 场景 yaw 顺时针为正
    best = None
    for rd in roads or []:
        pts = getattr(rd, "stop_line", None) or []
        for k in range(len(pts) - 1):
            mx = (float(pts[k].x) + float(pts[k + 1].x)) * 0.5
            my = (float(pts[k].y) + float(pts[k + 1].y)) * 0.5
            dx, dy = mx - x, my - y
            dist = math.hypot(dx, dy)
            if dist > max_dist or dist < 0.5:
                continue
            if (dx * fx + dy * fy) / dist < 0.70:    # 只认前方 ±45°
                continue
            if best is None or dist < best[0]:
                best = (dist, (mx, my))
    return best


def pick_light(groups, x: float, y: float, yaw_deg: float,
               max_dist: float = C.LIGHT_LOOK_M):
    """车前方最近的直行信号灯（`TrafficLightInfo`）或 None。"""
    psi = math.radians(yaw_deg)
    fx, fy = math.cos(psi), -math.sin(psi)
    best, bd = None, float("inf")
    for g in groups or []:
        for tl in getattr(g, "traffic_lights", None) or []:
            dx = float(tl.position.x) - x
            dy = float(tl.position.y) - y
            dist = math.hypot(dx, dy)
            if dist > max_dist or dist < 0.5:
                continue
            if (dx * fx + dy * fy) / dist < 0.50:    # 前方 ±60°
                continue
            if dist < bd:
                best, bd = tl, dist
    return best


def find_light_by_id(groups, light_id: str):
    """按 id 在当帧的灯组里重新找同一个灯（每帧刷新用，避免拿到过期状态）。"""
    for g in groups or []:
        for tl in getattr(g, "traffic_lights", None) or []:
            if tl.id == light_id:
                return tl
    return None


def lane_width_at(static, x: float, y: float, fallback: float = 3.50) -> float:
    """离车最近的车道宽度，找不到就用 fallback。"""
    best = None
    for rd in getattr(static, "roads", None) or []:
        for ln in getattr(rd, "lanes", None) or []:
            width = float(getattr(ln, "width", 0.0) or 0.0)
            for p in getattr(ln, "path_points", None) or []:
                d = math.hypot(float(p.x) - x, float(p.y) - y)
                if best is None or d < best[0]:
                    best = (d, width)
    if best is not None and best[0] < 10.0 and best[1] > 1.5:
        return best[1]
    return max(1.5, float(fallback))
