# -*- coding: utf-8 -*-
"""车道走廊：把「能往左右各挪多少」算清楚。

平台的静态数据里，每条车道都带左右边界（`BorderInfo`），边界的 `type` 就是线的类型：

* `MIDDLE_LINE=1`  中线（双黄实线）——**不可跨**
* `SIDE_LINE=2`    侧线（路边沿线）——**不可压**
* `SOLID_LINE=3`   实线——不可跨
* `STOP_LINE=4`    停止线
* `ZEBRA_CROSSING=5` 斑马线
* `DASH_LINE=6`    虚线——**可跨**（跨过去就是相邻车道，等于允许自由变道）

所以「走廊」不是一条车道，而是：从当前车道出发，边界只要是虚线就继续往那边数，
直到遇到第一条不可跨/不可压的线为止。算出来的净空再扣掉半个车宽和安全余量，
就是这一帧真正能用的左右空间。

所有长度单位 m；横向符号约定与 `track.py` 一致：**正 = 车头方向的左侧**。
"""
from __future__ import annotations

import math

DASH_LINE = 6
MIDDLE_LINE = 1
SIDE_LINE = 2
SOLID_LINE = 3
STOP_LINE = 4
ZEBRA_CROSSING = 5

CROSSABLE = {DASH_LINE}
BLOCKING = {MIDDLE_LINE, SIDE_LINE, SOLID_LINE, STOP_LINE, ZEBRA_CROSSING}

MAX_LANE_HOPS = 4  # 最多连续跨 4 条虚线，防止链接成环时无限走


def _xy(p):
    """兼容 Vector2 / Vector3 / tuple / 带 x,y 的对象。"""
    if isinstance(p, (tuple, list)):
        return float(p[0]), float(p[1])
    return float(getattr(p, "x")), float(getattr(p, "y"))


def _line_type(border) -> int:
    t = getattr(border, "type", None)
    if t is None:
        return 0
    return int(getattr(t, "value", t))


class _Lane:
    __slots__ = ("road", "id", "width", "pts", "left_t", "right_t", "left_id", "right_id")

    def __init__(self, road, lane):
        self.road = road
        self.id = str(getattr(lane, "id", ""))
        self.width = float(getattr(lane, "width", 0.0) or 0.0)
        self.pts = [_xy(p) for p in (getattr(lane, "path_points", None) or [])]
        self.left_t = _line_type(getattr(lane, "left_border", None))
        self.right_t = _line_type(getattr(lane, "right_border", None))
        self.left_id = str(getattr(lane, "left_lane_id", "") or "")
        self.right_id = str(getattr(lane, "right_lane_id", "") or "")


class Room:
    """一次查询的结果。left/right 已经扣掉半个车宽与余量，负数表示车本身就压线了。"""

    __slots__ = ("left", "right", "lane_width", "offset", "road_id", "lane_id", "left_type", "right_type")

    def __init__(self, left, right, lane_width, offset, road_id, lane_id, left_type, right_type):
        self.left = left
        self.right = right
        self.lane_width = lane_width
        self.offset = offset
        self.road_id = road_id
        self.lane_id = lane_id
        self.left_type = left_type
        self.right_type = right_type

    def __repr__(self) -> str:
        return (f"Room(left={self.left:+.2f}, right={self.right:+.2f}, "
                f"lane={self.lane_width:.2f}, off={self.offset:+.2f}, "
                f"types=({self.left_type},{self.right_type}))")


class LaneCorridor:
    """按静态车道图算左右净空。构造一次，之后每帧 `room(x, y)` 即可（纯几何，不做决策）。"""

    def __init__(self, static, ego_width: float = 1.81, margin: float = 0.25, cell: float = 8.0,
                 log_note=None):
        self.half_width = max(0.0, float(ego_width)) / 2.0
        self.margin = max(0.0, float(margin))
        self.cell = float(cell)
        self.log_note = log_note
        self.lanes: dict[tuple[str, str], _Lane] = {}
        self.grid: dict[tuple[int, int], list[tuple[str, str, float, float]]] = {}
        for road in getattr(static, "roads", None) or []:
            rid = str(getattr(road, "id", ""))
            for lane in getattr(road, "lanes", None) or []:
                ln = _Lane(rid, lane)
                if len(ln.pts) < 2:
                    continue
                key = (rid, ln.id)
                self.lanes[key] = ln
                for px, py in ln.pts:
                    self.grid.setdefault((int(px // self.cell), int(py // self.cell)), []).append(
                        (rid, ln.id, px, py)
                    )
                # 自车道中心线也存一份，供曲率/切线查询
                ln.pts = ln.pts

    # ---------- 几何 ----------

    def _nearest_lane(self, x: float, y: float, max_d: float = 8.0):
        cx, cy = int(x // self.cell), int(y // self.cell)
        best = None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for rid, lid, px, py in self.grid.get((cx + dx, cy + dy), ()):
                    d = math.hypot(px - x, py - y)
                    if best is None or d < best[0]:
                        best = (d, rid, lid)
        if best is None or best[0] > max_d:
            return None
        return best[1], best[2]

    @staticmethod
    def _project(pts, x, y):
        """返回 (最近点, 该处单位切线, 无符号距离)。点在折线左侧当且仅当 cross>0。"""
        best = None
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            vx, vy = bx - ax, by - ay
            L = math.hypot(vx, vy)
            if L <= 1e-9:
                continue
            ux, uy = vx / L, vy / L
            t = ((x - ax) * ux + (y - ay) * uy)
            t = max(0.0, min(L, t))
            px, py = ax + ux * t, ay + uy * t
            d = math.hypot(x - px, y - py)
            if best is None or d < best[0]:
                best = (d, (px, py), (ux, uy))
        if best is None:
            return (x, y), (1.0, 0.0), 0.0
        d, p, u = best
        return p, u, d

    # ---------- 对外 ----------

    def room(self, x: float, y: float):
        """返回 `Room`；找不到车道返回 None（调用方应退回「只信轨迹」的老行为）。"""
        hit = self._nearest_lane(x, y)
        if hit is None:
            return None
        rid, lid = hit
        lane = self.lanes[(rid, lid)]
        _, (ux, uy), _ = self._project(lane.pts, x, y)
        # 横向偏移：正 = 中心线左侧
        offset = (x - lane.pts[0][0]) * (-uy) + (y - lane.pts[0][1]) * (ux)
        # 用最近点重算，避免长折线首点误差
        p, u, _ = self._project(lane.pts, x, y)
        offset = (x - p[0]) * (-u[1]) + (y - p[1]) * (u[0])

        half = lane.width / 2.0
        left_room = half - offset      # 到本车道左边界的距离
        right_room = half + offset     # 到本车道右边界的距离
        left_type, right_type = lane.left_t, lane.right_t

        # 左边界是虚线 → 可以继续往左借邻居的车道
        cur, hops, acc = lane, 0, 0.0
        while left_type in CROSSABLE and cur.left_id and hops < MAX_LANE_HOPS:
            nxt = self.lanes.get((rid, cur.left_id))
            if nxt is None:
                break
            acc += nxt.width
            cur, hops = nxt, hops + 1
            left_type = nxt.left_t
        left_room += acc

        cur, hops, acc = lane, 0, 0.0
        while right_type in CROSSABLE and cur.right_id and hops < MAX_LANE_HOPS:
            nxt = self.lanes.get((rid, cur.right_id))
            if nxt is None:
                break
            acc += nxt.width
            cur, hops = nxt, hops + 1
            right_type = nxt.right_t
        right_room += acc

        return Room(left=left_room - self.half_width - self.margin,
                    right=right_room - self.half_width - self.margin,
                    lane_width=lane.width, offset=offset,
                    road_id=rid, lane_id=lid,
                    left_type=left_type, right_type=right_type)

    def neighbor(self, x: float, y: float, side: int):
        """沿 `side`（+1 左 / −1 右）跨一条**虚线**能到达的相邻车道。

        返回 dict（`lane_id/width/pts/shift_m/side`）或 None。
        `shift_m` = 两条车道中心线的横向间距（= (w1+w2)/2），也就是「真变道」要挪的量。
        边界不是虚线（实线/双黄/路边线/停止线/斑马线）时返回 None——那种情况不许变道。
        """
        hit = self._nearest_lane(x, y)
        if hit is None:
            return None
        rid, lid = hit
        lane = self.lanes[(rid, lid)]
        if side > 0:
            line_t, nid = lane.left_t, lane.left_id
        else:
            line_t, nid = lane.right_t, lane.right_id
        if line_t not in CROSSABLE or not nid:
            return None
        nxt = self.lanes.get((rid, nid))
        if nxt is None or len(nxt.pts) < 2:
            return None
        # 两侧证词都要看（2026-10-25 实车教训）：线型可能只记在邻居那一侧。
        # 本车这侧写成虚线、邻居那侧写成双黄的情形真实存在——元素 1 就是这么画的，
        # 只看自己这侧就会把「双黄」当成「虚线」跨过去。
        their_t = nxt.right_t if side > 0 else nxt.left_t
        if their_t not in CROSSABLE and their_t != 0:
            if self.log_note:
                self.log_note(f"[corridor] 拒绝变道：本侧 {line_t}、对侧 {their_t}"
                              f"（对侧不是虚线，可能就是双黄/实线）")
            return None
        # 方向校验（2026-10-25 实车教训：光看「虚线」不够）：
        # 相邻车道若与当前车道方向相反，就是对向车道，跨过去就是压双黄/逆行的重罚，
        # 一律不许变。用两条中心线在最近点处的单位切线点积判断，<0.3 视为反向。
        _, (ux, uy), _ = self._project(lane.pts, x, y)
        _, (vx, vy), _ = self._project(nxt.pts, x, y)
        if ux * vx + uy * vy < 0.30:
            return None
        shift = 0.5 * (lane.width + nxt.width)
        return {"lane_id": nxt.id, "width": nxt.width, "pts": nxt.pts,
                "shift_m": shift, "side": side, "road_id": rid}

    def neighbors(self, x: float, y: float) -> dict:
        """左右都试一次，返回 {"left": dict|None, "right": dict|None}。"""
        return {"left": self.neighbor(x, y, +1), "right": self.neighbor(x, y, -1)}
