"""车道边界保护：让车身四角离**实线**保持至少 `BORDER_SAFE_M`。

为什么要有这一层
----------------
p2 与 p4 两趟的环岛出口，车身四角到实线的最近余量分别是 0.02 m 与 0.01 m
（`tools/replay_score.py --lines` 实测，坐标 (-3028,-889) 一带），只差 1 cm，
结果 p4 被判 `-20 压道路边线`、p2 没有。**1 cm 级的差别离线无法复现，也没有
任何控制手段能保证**，所以策略不是"贴着线走"，而是把所有实线的余量整体抬到
0.4~0.5 m 以上，让平台的采样相位怎么跳都跳不过去。

口径
----
* 只认 `borderType in (1, 2)`：mapd 里 1/2 是实线（道路边线，压了要扣分），
  6/3 是虚线（车道分界，可以跨）。lianxu4 实测 16278 条边界线段里实线 6278 条。
* 车身用四角矩形（`EGO_LEN` x `EGO_WID`），取四角到最近实线的**最小**距离。
* 近侧在左（`lat > 0`）就朝右打舵（本车 steer > 0 = 顺时针 = 向右），反之亦然。

刻意做成"失联可用"：地图找不到、格式不对，一律返回 None，调用方原样放行。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

DEFAULT_SCENE_ROOT = Path(r"D:\无人算法\场景包\scenes")
SOLID_TYPES = (1, 2)


def _xy(p):
    """mapd 的 pathPoint 有三种写法：dict / [x, y] / 带 x,y 属性的对象。"""
    if hasattr(p, "x"):
        return float(p.x), float(p.y)
    if isinstance(p, dict):
        return float(p["x"]), float(p["y"])
    return float(p[0]), float(p[1])


def _load_points(value):
    if isinstance(value, dict):
        for key in ("pathPoint", "path_points", "points"):
            if key in value:
                return [_xy(p) for p in value[key]]
        return []
    if isinstance(value, list):
        return [_xy(p) for p in value]
    return []


def load_solid_segments(mapd_path) -> list[tuple[float, float, float, float]]:
    """读出所有实线线段 `(x1, y1, x2, y2)`。"""
    data = json.loads(Path(mapd_path).read_text(encoding="utf-8"))
    roads = data.get("roads", data) if isinstance(data, dict) else data
    segs: list[tuple[float, float, float, float]] = []
    for road in roads or []:
        for lane in road.get("laneData") or road.get("lane_data") or []:
            for side in ("LeftBorder", "RightBorder", "leftBorder", "rightBorder"):
                border = lane.get(side)
                if not border:
                    continue
                try:
                    kind = int(border.get("borderType", border.get("border_type", 0)))
                except (TypeError, ValueError):
                    continue
                if kind not in SOLID_TYPES:
                    continue
                pts = _load_points(border)
                for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                    segs.append((ax, ay, bx, by))
    return segs


def find_mapd(route_pts, scene_root=DEFAULT_SCENE_ROOT, tol: float = 0.8) -> Path | None:
    """按路线指纹认场景包，返回该场景的 mapd（不写死场景名）。"""
    root = Path(scene_root)
    if not root.is_dir() or not route_pts:
        return None
    want = [(_xy(p) if not isinstance(p, tuple) else p) for p in route_pts]
    best, best_err = None, float(tol)
    for cand in root.iterdir():
        rp = cand / "SmartCar_Data" / "StreamingAssets" / "mapfile" / "route"
        mp = cand / "SmartCar_Data" / "StreamingAssets" / "mapfile" / "mapd"
        if not (rp.is_file() and mp.is_file()):
            continue
        try:
            raw = json.loads(rp.read_text(encoding="utf-8"))
            pts = _load_points(raw if not isinstance(raw, list) else raw)
            if not isinstance(raw, list) and not pts:
                pts = [_xy(p) for p in raw.get("route", [])]
        except Exception:                                  # noqa: BLE001
            continue
        if len(pts) != len(want):
            continue
        err = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(pts, want))
        if err <= best_err:
            best, best_err = mp, err
    return best


def _dist_point_seg(px, py, ax, ay, bx, by) -> float:
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 <= 1e-9 else max(0.0, min(1.0, (wx * vx + wy * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


class BorderGrid:
    """线段网格索引 + 车身四角到实线的最小余量。"""

    def __init__(self, segs, cell: float = 6.0) -> None:
        self.cell = float(cell)
        self.grid: dict[tuple[int, int], list[tuple[float, float, float, float]]] = {}
        for seg in segs:
            ax, ay, bx, by = seg
            for cx in range(int(math.floor(min(ax, bx) / self.cell)) - 1,
                            int(math.floor(max(ax, bx) / self.cell)) + 2):
                for cy in range(int(math.floor(min(ay, by) / self.cell)) - 1,
                                int(math.floor(max(ay, by) / self.cell)) + 2):
                    self.grid.setdefault((cx, cy), []).append(seg)

    def __bool__(self) -> bool:
        return bool(self.grid)

    def near(self, x: float, y: float, radius: float = 8.0):
        cx0 = int(math.floor((x - radius) / self.cell))
        cx1 = int(math.floor((x + radius) / self.cell))
        cy0 = int(math.floor((y - radius) / self.cell))
        cy1 = int(math.floor((y + radius) / self.cell))
        seen = set()
        for cx in range(cx0, cx1 + 1):
            for cy in range(cy0, cy1 + 1):
                for seg in self.grid.get((cx, cy), ()):
                    if seg not in seen:
                        seen.add(seg)
                        yield seg

    @staticmethod
    def corners(x: float, y: float, yaw: float, length: float, width: float):
        """车身四角（场景航向：0=+x，90=-y）。"""
        th = math.radians(float(yaw))
        hx, hy = math.cos(th), -math.sin(th)          # 车头
        lx, ly = -hy, hx                              # 左侧
        hl, hw = 0.5 * float(length), 0.5 * float(width)
        out = []
        for sx in (+1, -1):
            for sy in (+1, -1):
                out.append((x + hx * hl * sx + lx * hw * sy,
                            y + hy * hl * sx + ly * hw * sy))
        return out

    def clearance(self, x: float, y: float, yaw: float,
                  length: float, width: float, radius: float = 8.0):
        """返回 `(最小余量, 近侧横向符号, 近点)`；无实线时返回 `(inf, 0, None)`。

        近侧横向符号：`+1` 表示最近的实线在车身左侧，`-1` 在右侧。
        """
        best = (float("inf"), 0, None)
        for cx, cy in self.corners(x, y, yaw, length, width):
            for ax, ay, bx, by in self.near(cx, cy, radius):
                d = _dist_point_seg(cx, cy, ax, ay, bx, by)
                if d < best[0]:
                    # 近点取线段上的投影点近似（够用：只用来判左右）
                    vx, vy = bx - ax, by - ay
                    L2 = vx * vx + vy * vy
                    t = 0.0 if L2 <= 1e-9 else max(0.0, min(1.0, ((cx - ax) * vx + (cy - ay) * vy) / L2))
                    best = (d, 0, (ax + t * vx, ay + t * vy))
        if best[2] is None:
            return best
        d, _, (px, py) = best
        th = math.radians(float(yaw))
        lx, ly = math.sin(th), math.cos(th)           # 左侧法向
        lat = (px - x) * lx + (py - y) * ly
        return d, (1 if lat >= 0.0 else -1), (px, py)
