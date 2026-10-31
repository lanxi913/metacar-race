# -*- coding: utf-8 -*-
"""场景上下文兜底：把「从 API 读目标/几何」和「读不到怎么办」集中到一处。

正式赛题是训练场景的泛化版（路线、车位、停止线、信号灯、时限都可能变），
所以任何目标都必须现场读；但读不到时**绝不允许抛异常**，一律退回默认值并打警告，
让车继续按已有的控制律跑完（宁可拿不到奖励分，也不能整局 0 分）。

设计原则（见 AGENTS.md 第 4、9 条）：
  1. 只提供"读数据 + 兜底"，不含任何控制律，改了它不会改变车的行为；
  2. 每个兜底都打 [warn] 日志，方便赛后对着日志判断是"场景变了"还是"控制坏了"；
  3. 默认值取训练场景实测值，保证正常路径上与原行为逐位一致。

用法：
    from metacar_race.scene_ctx import SceneCtx
    ctx = SceneCtx.from_api(api)
    route = ctx.route            # [(x, y), ...]，可能是空列表
    ...
    limit = ctx.time_limit(msg.scene_status, default=100.0)
"""
from __future__ import annotations

import math
from statistics import median

# 训练场景实测值，作为兜底默认（只在 API 读不到时使用）
DEFAULT_TIME_LIMIT_S = 100.0
DEFAULT_LANE_WIDTH_M = 3.5
DEFAULT_CAR_LENGTH_M = 4.0
DEFAULT_CAR_WIDTH_M = 1.8


def warn(msg: str) -> None:
    """兜底警告。带 [warn] 前缀，方便 grep。"""
    print(f"[warn] {msg}", flush=True)


def _xy(obj):
    """把带 x/y 的 Vector2/Vector3 转成 (float, float)；失败返回 None。"""
    try:
        return float(obj.x), float(obj.y)
    except Exception:  # noqa: BLE001
        return None


class SceneCtx:
    """一次跑局里的静态数据 + 逐帧状态的只读视图。"""

    def __init__(self, static=None, route=None, sub_scenes=None, roads=None, route_raw=None):
        self.static = static
        self.route = route or []
        self.route_raw = route_raw or []   # 原始 Vector3 列表（PurePursuit 等需要 .x/.y）
        self.sub_scenes = sub_scenes or []
        self.roads = roads or []

    # ---------- 构造 ----------
    @classmethod
    def from_api(cls, api):
        static = None
        try:
            static = api.get_scene_static_data()
        except Exception as e:  # noqa: BLE001
            warn(f"get_scene_static_data 失败（{type(e).__name__}: {e}），按空场景继续")
        return cls.from_static(static)

    @classmethod
    def from_static(cls, static):
        route = []
        route_raw = []
        sub = []
        roads = []
        if static is not None:
            for p in (getattr(static, "route", None) or []):
                xy = _xy(p)
                if xy is not None:
                    route.append(xy)
                    route_raw.append(p)
            for s in (getattr(static, "sub_scenes", None) or []):
                sub.append(getattr(s, "name", "") or "")
            roads = list(getattr(static, "roads", None) or [])
        else:
            warn("没有静态数据：路线/路网按空处理，只循迹当前位置或驻车")
        if not route:
            warn("static.route 为空：没有任何参考路径，控制器应驻车而不是硬开")
        if not sub:
            warn("static.sub_scenes 为空：拿不到子场景名与端点")
        return cls(static=static, route=route, sub_scenes=sub, roads=roads,
                   route_raw=route_raw)

    # ---------- 几何 ----------
    @property
    def route_length_m(self) -> float:
        return polyline_length(self.route)

    @property
    def last_route_point(self):
        return self.route[-1] if self.route else None

    def lane_width(self, default: float = DEFAULT_LANE_WIDTH_M) -> float:
        """路网里所有车道宽度的中位数。用来把横向偏置/阈值按车道宽度缩放。"""
        widths = []
        for road in self.roads:
            for lane in (getattr(road, "lanes", None) or []):
                w = getattr(lane, "width", 0.0) or 0.0
                if w > 0.5:
                    widths.append(float(w))
        if not widths:
            return default
        return float(median(widths))

    def sub_scene_end_point(self, name: str | None = None):
        """按子场景名取 end_point；name 为空时取最后一个子场景。"""
        for s in (getattr(self.static, "sub_scenes", None) or []):
            if name is None or getattr(s, "name", None) == name:
                xy = _xy(getattr(s, "end_point", None))
                if xy is not None:
                    return xy
        return None

    # ---------- 逐帧 ----------
    @staticmethod
    def time_limit(status, default: float = DEFAULT_TIME_LIMIT_S) -> float:
        """平台给的时限；读不到用默认值。"""
        try:
            t = float(getattr(status, "time_limit", 0.0) or 0.0)
        except Exception:  # noqa: BLE001
            t = 0.0
        return t if t > 1.0 else default

    @staticmethod
    def time_left(status, default: float = DEFAULT_TIME_LIMIT_S) -> float:
        try:
            limit = SceneCtx.time_limit(status, default)
            used = float(getattr(status, "used_time", 0.0) or 0.0)
        except Exception:  # noqa: BLE001
            return default
        return max(0.0, limit - used)

    @staticmethod
    def end_point(status, route=None, fallback=None):
        """终点：优先平台给的 end_point，其次路线末点，最后 fallback。"""
        xy = _xy(getattr(status, "end_point", None))
        if xy is not None:
            return xy
        if route:
            return route[-1]
        return fallback

    @staticmethod
    def car_size(msg, default=(DEFAULT_CAR_LENGTH_M, DEFAULT_CAR_WIDTH_M)):
        """主车尺寸：MainVehicleInfo 里有 length/width，读不到用默认。"""
        try:
            veh = msg.main_vehicle
            length = float(veh.length)
            width = float(veh.width)
            if length > 0.5 and width > 0.3:
                return length, width
        except Exception:  # noqa: BLE001
            pass
        return default

    @staticmethod
    def obstacles(msg, kind=None):
        """当前帧障碍物；kind 可传 ObstacleType 成员做过筛。"""
        items = list(getattr(msg, "obstacles", None) or [])
        if kind is None:
            return items
        out = []
        for ob in items:
            t = getattr(ob, "type", None)
            if t == kind or getattr(t, "value", None) == getattr(kind, "value", None):
                out.append(ob)
        return out


# ---------- 纯函数工具 ----------
def polyline_length(points) -> float:
    total = 0.0
    for i in range(1, len(points)):
        total += math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
    return total


def distance(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest(points, x, y):
    """返回 (下标, 距离)；points 为空时返回 (None, inf)。"""
    best_i, best_d = None, float("inf")
    for i, p in enumerate(points):
        d = math.hypot(p[0] - x, p[1] - y)
        if d < best_d:
            best_i, best_d = i, d
    return best_i, best_d


def pick_nearest_obstacle(items, x, y, kind=None):
    """在障碍物里挑离 (x, y) 最近的一个；可先按 type 过滤。"""
    best, best_d = None, float("inf")
    for ob in items or []:
        if kind is not None:
            t = getattr(ob, "type", None)
            if not (t == kind or getattr(t, "value", None) == getattr(kind, "value", None)):
                continue
        d = math.hypot(float(ob.pos_x) - x, float(ob.pos_y) - y)
        if d < best_d:
            best, best_d = ob, d
    return best, best_d


def obstacle_xy(ob):
    return (float(ob.pos_x), float(ob.pos_y))


def obstacle_yaw_deg(ob) -> float:
    """障碍物朝向（度）。SDK 里 ori_z 是偏航角。"""
    try:
        return float(ob.ori_z) % 360.0
    except Exception:  # noqa: BLE001
        return 0.0


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def yaw_error_deg(a: float, b: float) -> float:
    """两个朝向的最小夹角（0~180），正反方向都算对齐。"""
    d = abs(wrap180(a - b))
    return min(d, 180.0 - d)
