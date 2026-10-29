# -*- coding: utf-8 -*-
"""泊车元素（连续场景里的「平行车道泊车」）：先认车位，再把自车"搬"进车位坐标系里
复用已经练熟的侧方停车轨迹，最后**倒车入库**。

2026-10-26 从 `origin/feat/continuous4`（PR #43，平台 p9 = 1522）移植过来，替换本分支
`park.py` 的接近段/交班逻辑。相对原文件只改两处：

1. `DEFAULT_SCENE_ROOT` 换成候选目录 + 环境变量（`SMARTCAR_SCENE_ROOT` /
   `SCENE_ROOT`）；本机没有场景包时 `find_mapd()` 返回 None，退回经验偏置。
2. `Command.final=True` —— 本分支 `Command` 新增同义字段（语义：策略全权接管，
   driver 不许再插手升挡闸门与终点驻车）。

原文件的原始说明（保留，便于对照）：

    停车场景可以单独写一个分支，在 while 循环里识别到特殊停车才启动；启动之后也
    不是直接停车，而是先去识别停车位置和停车情况，然后把车的运动轨迹转移到已经
    练熟的习题库中的相对位置，然后进行停车。……停车永远是先驶过去之后再倒回来的。

也就是说：**不许沿着 route 直接开进车位**（此前 `park_cs4.py` 基线就是把车开到
route 末点再 `EndPark` 一停了事，平台既不算完成也没有分）。本模块分三段：

1. **认车位**：`obstacles` 里 `type == 30 (PARKING_SLOT)` 的对象给出中心、朝向、
   长宽。看不到车位 → 返回 None，整个元素交回基线（不猜、不写死坐标）。
2. **对齐（相对位置换算）**：建立"车位坐标系"——原点在车位中心，`u` = 车位长度
   方向，`r` = 该方向的右侧法向；再旋到**模板坐标系**：车位在原点、朝向 90°、
   自车沿 −Y 前进（这正是 main 上 `ParallelParkingController` 连续 5 次 210 分的
   坐标约定）。自车先按规定侧向偏置开过去，停在车位"前方"的 staging 点。
   侧向偏置优先用地图实测（车位中心所在那条车道的中线），拿不到地图才退回经验
   值，并把实际取值打进日志。
3. **倒车**：把模板坐标系里的位姿喂给 `ParallelParkingController`（两段圆弧 →
   反舵回正 → 前后微调 → 轻刹保持）。两个坐标系之间只差一个刚体旋转（`det = +1`，
   无镜像），所以舵角可以原样下发。
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from types import SimpleNamespace

from metacar import GearMode

from ..config import (PARALLEL_A_MAX, PARALLEL_BRAKE_CMD, PARALLEL_CAR_LEN,
                      PARALLEL_MAX_STEER_RAD, PARALLEL_SLOT_TYPE,
                      PARALLEL_STEER, PARALLEL_THROTTLE_MAX, PARALLEL_V_APPROACH,
                      PARALLEL_WHEELBASE)
from ..control.parallel_parking import ParallelParkingController, turning_radius
from . import config as C
from .strategies import Command, ElementContext, Strategy

_CAR_TYPE = 6                    # ObstacleType.CAR 的枚举值（int）


def _candidate_roots() -> list[str]:
    """场景包根目录候选：不同机器的路径不一样，找不到就退回经验偏置。"""
    roots: list[str] = []
    for env in ("SMARTCAR_SCENE_ROOT", "SCENE_ROOT"):
        v = os.environ.get(env)
        if v:
            roots.append(v)
    roots += [
        r"D:\download3\算法邀請賽\场景包\scenes",
        r"D:\无人算法\场景包\scenes",
        r"D:\SmartCar\场景包\scenes",
        r"C:\SmartCar\场景包\scenes",
    ]
    return roots


#: 兼容外部按名导入；真正搜索时会遍历 `_candidate_roots()`
DEFAULT_SCENE_ROOT = r"D:\无人算法\场景包\scenes"


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def dir_of(a_deg: float) -> tuple[float, float]:
    """场景航向 → 单位向量（0=+x，90=−y，顺时针为正）。"""
    r = math.radians(a_deg)
    return (math.cos(r), -math.sin(r))


@dataclass(frozen=True)
class SlotFrame:
    """车位坐标系 + 到模板坐标系的换算。"""

    x: float
    y: float
    ori: float
    length: float
    width: float

    @property
    def u(self) -> tuple[float, float]:
        return dir_of(self.ori)

    @property
    def r(self) -> tuple[float, float]:
        return dir_of(self.ori + 90.0)

    def to_slot(self, x: float, y: float) -> tuple[float, float]:
        """(沿车位长度方向, 右法向) —— 右法向 > 0 表示车在车位右侧。"""
        dx, dy = x - self.x, y - self.y
        ux, uy = self.u
        rx, ry = self.r
        return dx * ux + dy * uy, dx * rx + dy * ry

    def to_template(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        """场景位姿 → 模板位姿（车位在原点、朝向 90°、自车沿 −Y 前进）。"""
        du, dr = self.to_slot(x, y)
        return -dr, -du, 90.0 + wrap180(yaw - self.ori)

    def lateral_of(self, x: float, y: float) -> float:
        """模板坐标系下的横向偏置（带符号）。"""
        return -self.to_slot(x, y)[1]


def _slot_fields(o) -> dict | None:
    """把障碍统一读成 dict：既吃 SDK 对象（`o.type.value`），也吃 `ObstacleScan` 出的 dict。

    2026-10-25 教训（元素 12 平行车道泊车全程没接管）：`ctx.obstacles` 传进来的是
    `ObstacleScan` 的 **dict**（键 `type/x/y/ori/len/wid`），而这里原来只按 SDK 对象
    读 `o.type.value / o.pos_x`，于是 `find_slot()` 永远返回 None —— 侧方停车策略一次
    都没启动，基线直接把车头拐进车位。现在两种都认。
    """
    if isinstance(o, dict):
        try:
            return {
                "type": int(o.get("type", -1) if o.get("type") is not None else -1),
                "x": float(o.get("x", 0.0)), "y": float(o.get("y", 0.0)),
                "ori": float(o.get("ori", 0.0) or 0.0),
                "len": float(o.get("len", 0.0) or 0.0),
                "wid": float(o.get("wid", 0.0) or 0.0),
            }
        except Exception:                               # noqa: BLE001
            return None
    t = getattr(o, "type", None)
    try:
        return {
            "type": int(getattr(t, "value", -1) or -1),
            "x": float(getattr(o, "pos_x", 0.0)), "y": float(getattr(o, "pos_y", 0.0)),
            "ori": float(getattr(o, "ori_z", 0.0) or 0.0),
            "len": float(getattr(o, "length", 0.0) or 0.0),
            "wid": float(getattr(o, "width", 0.0) or 0.0),
        }
    except Exception:                                   # noqa: BLE001
        return None


def find_slot(obstacles) -> SlotFrame | None:
    """从障碍物列表里挑出车位（type=30），挑不到返回 None。"""
    for o in obstacles or ():
        f = _slot_fields(o)
        if f is None or f["type"] != int(PARALLEL_SLOT_TYPE):
            continue
        return SlotFrame(f["x"], f["y"], f["ori"], f["len"], f["wid"])
    return None


# ---------------------------------------------------------------- 地图（可选）
def _load_points(path: str):
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
    except Exception:                                   # noqa: BLE001
        return None
    if isinstance(raw, dict):
        for key in ("points", "route", "pathPoint", "data"):
            if isinstance(raw.get(key), list):
                raw = raw[key]
                break
    out = []
    for p in raw if isinstance(raw, list) else []:
        try:
            if isinstance(p, dict):
                out.append((float(p["x"]), float(p["y"])))
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                out.append((float(p[0]), float(p[1])))
        except Exception:                               # noqa: BLE001
            continue
    return out or None


def _xy_of(p) -> tuple[float, float]:
    """SDK 的 Vector3 / list / tuple 都能取 (x, y)。

    注意不能在 `getattr(p, "x", p[0])` 里直接写 `p[0]`：默认值是**先求值**的，
    Vector3 不支持下标就会抛 `TypeError: 'Vector3' object is not subscriptable`
    （2026-10-25 p2 跑次就是这样让 mapd 认不出来，退回经验偏置 +3.00 m）。
    """
    if hasattr(p, "x") and hasattr(p, "y"):
        return float(getattr(p, "x")), float(getattr(p, "y"))
    return float(p[0]), float(p[1])


def _find_mapd_in(scene_root: str, route_pts, tol: float = 0.8):
    if not route_pts or not os.path.isdir(scene_root):
        return None
    want = [_xy_of(p) for p in route_pts]
    best, best_err = None, float("inf")
    try:
        names = os.listdir(scene_root)
    except Exception:                                   # noqa: BLE001
        return None
    for name in names:
        base = os.path.join(scene_root, name, "SmartCar_Data", "StreamingAssets", "mapfile")
        rp = os.path.join(base, "route")
        mp = os.path.join(base, "mapd")
        if not (os.path.isfile(rp) and os.path.isfile(mp)):
            continue
        got = _load_points(rp)
        if not got or len(got) != len(want):
            continue
        err = max(math.hypot(g[0] - w[0], g[1] - w[1]) for g, w in zip(got, want))
        if err < best_err and err <= tol:
            best, best_err = mp, err
    return best


def find_mapd(route_pts, scene_root: str | None = None, tol: float = 0.8):
    """按路线指纹认场景包（不写死场景名），返回 mapd 路径；认不出返回 None。

    2026-10-26：本机没有场景包（`mapfile` 目录不存在），这里会遍历候选目录后
    返回 None；`_prepare()` 随之退回「车位半宽 + 半车道」的经验偏置，并把来源
    打进日志——不影响倒车入库本身。
    """
    roots = [scene_root] if scene_root else _candidate_roots()
    for root in roots:
        if not root:
            continue
        got = _find_mapd_in(root, route_pts, tol)
        if got:
            return got
    return None


def load_border_segments(mapd_path: str):
    """mapd → [(borderType, x1, y1, x2, y2), ...]，只取车道左右边界。"""
    with open(mapd_path, "r", encoding="utf-8") as fh:
        roads = json.load(fh)
    segs = []
    for rd in roads if isinstance(roads, list) else []:
        for ln in rd.get("laneData") or []:
            for key in ("LeftBorder", "RightBorder"):
                b = ln.get(key) or {}
                pts = b.get("pathPoint") or []
                bt = b.get("borderType")
                for i in range(len(pts) - 1):
                    a, c = pts[i], pts[i + 1]
                    try:
                        segs.append((bt, float(a["x"]), float(a["y"]),
                                     float(c["x"]), float(c["y"])))
                    except Exception:                   # noqa: BLE001
                        continue
    return segs


def lane_offset_via_map(slot: SlotFrame, segs, span: float = 7.0) -> float | None:
    """车位中心所在那条车道的**中线**相对车位中心的横向偏置（模板坐标系，带符号）。

    做法：沿车位右法向拉一条横断面，找跨越车位中心的最近左右两条边界线，取中点。
    """
    if not segs:
        return None
    rx, ry = slot.r
    hits: list[float] = []
    for _bt, x1, y1, x2, y2 in segs:
        dx, dy = x2 - x1, y2 - y1
        det = -dx * ry + rx * dy
        if abs(det) < 1e-9:
            continue
        ex, ey = x1 - slot.x, y1 - slot.y
        s = (ex * ry - rx * ey) / det
        t = (-dx * ey + ex * dy) / det
        if -1e-6 <= s <= 1.0 + 1e-6 and abs(t) <= span:
            hits.append(t)
    if not hits:
        return None
    left = max([h for h in hits if h < -1e-6], default=None)
    right = min([h for h in hits if h > 1e-6], default=None)
    if left is None or right is None:
        return None
    width = right - left
    if width < 2.0 or width > 6.5:                       # 不是一条正常车道
        return None
    return -0.5 * (left + right)                          # 模板横向 = −t，故取负


# ---------------------------------------------------------------- 元素策略
@dataclass
class _Pt:
    x: float
    y: float


class SlotParkStrategy(Strategy):
    """平行泊车元素：认车位 → 对齐 staging → 倒车入库。"""

    key = "parking"
    label = "车位坐标系侧方停车"

    #: 对齐段的转向增益（场景实测：0.030/度 + 0.18/米）
    _K_HEAD = 0.030
    _K_CROSS = 0.18

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        super().__init__(ctx, route, log)
        self.frame: SlotFrame | None = None
        self.ctl: ParallelParkingController | None = None
        self.phase = "WAIT"                  # WAIT → ALIGN → STOP → PARK → DONE
        self.wps: list[_Pt] = []
        self.idx = 0
        self.lat_target = 0.0
        self.lat_source = ""
        self.ego_len = float(ctx.ego_length or PARALLEL_CAR_LEN)
        self.ego_wid = float(ctx.ego_width or 1.90)
        self.stage_y = 0.0
        self.approach_stall = 0.0
        self.segs = None
        self.mapd_path = None
        self.last_note = ""
        self.stall_t = 0.0
        self._stall_said = False
        self.fit_t = 0.0                     # FIT 收紧后累计时间（超时放宽容差）
        self.fit_relaxed = False
        # 低速段的油门/刹车速率限制（见 return 处）：0.06 的刹车在 0.3 m/s 上
        # 一帧就是 −5.6 m/s²，平台照样记一次加速度事件
        self._thr_prev = 0.0
        self._brk_prev = 0.0
        # 基向量在 `_set_basis` 里定；先给个安全默认（1.0, 0.0）
        self.eX = (1.0, 0.0)
        self.eY = (0.0, 1.0)

    # ------------------------------------------------------------------ 准备
    def _say(self, msg: str) -> None:
        self.last_note = msg
        if self.log:
            self.log(msg)

    def _set_basis(self, slot: SlotFrame, x: float, y: float) -> None:
        """定下"车位坐标系 → 模板坐标系"的基向量（元素进入时定一次，之后不再变）。

        模板 = 参考实现（单场景 210×5）所在的那套坐标：车位在原点、自车从 +Y 侧
        沿 −Y 驶来、车头 yaw=90。

        2026-10-25 p2 教训：原来用 `(-dr, -du, 90+(yaw−ori))`，在 ori=270、自车朝
        +y 的那一侧会算出 yaw_t=270（车头朝 +Y），而参考实现的 ARC1 判据是
        `90 − yaw ≥ theta`，于是永远不成立、两段圆弧乱走，最后停在离车位 3 m 的
        对侧。这里改成按**自车实际来向**定轴：`eY` 指向上游，`eX` 取来向的右法向，
        于是 yaw_t 恒等于 90，参考实现的每一条判据都原样适用。
        """
        ux, uy = slot.u
        sgn = 1.0 if ((x - slot.x) * ux + (y - slot.y) * uy) >= 0.0 else -1.0
        self.eY = (ux * sgn, uy * sgn)                    # 上游方向（自车来向）
        self.eX = (-self.eY[1], self.eY[0])               # 来向（−eY）的右法向

    def to_t(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        """场景位姿 → 模板位姿（车位在原点、自车沿 −Y 前进、车头 yaw=90）。"""
        slot = self.frame
        ex, ey = self.eX, self.eY
        X = (slot.x - x) * ex[0] + (slot.y - y) * ex[1]
        Y = (x - slot.x) * ey[0] + (y - slot.y) * ey[1]
        hx, hy = dir_of(yaw)
        # 模板 X 轴 = (eY_y, −eY_x)（右手法，和位置的 X 同一根轴）；
        # eX = −X轴，所以航向在 X 上的分量要取负 —— 少了这个负号，非 90° 航向会被
        # 镜像（yaw_t = 180 − yaw），对齐阶段就会往反方向打死舵（单场景实测 t=7 s 撞墙）。
        tx = -(hx * ex[0] + hy * ex[1])
        ty = hx * ey[0] + hy * ey[1]
        return X, Y, math.degrees(math.atan2(-ty, tx))

    def _prepare(self, ctx: ElementContext, slot: SlotFrame) -> None:
        self.frame = slot
        self._set_basis(slot, ctx.x, ctx.y)
        X0, Y0, _yaw0 = self.to_t(ctx.x, ctx.y, ctx.yaw)
        # 侧向偏置：先用地实测，拿不到再退回经验值。
        # 2026-10-25 p2 教训：`lane_offset_via_map` 找的是"车位中心所在的那条车道"，
        # 平行泊车的车位本身就在**停车带**里（本场车位中心离车道中线只有 0.30 m），
        # 那个数不能当行车横向偏置用。参考实现（单场景 210×5）是 3.43 m ——
        # 车位半宽 + 半个车道。所以实测值只在一个合理区间内才采信。
        lat = None
        try:
            self.mapd_path = find_mapd(getattr(ctx.static, "route", None) or self.route)
            if self.mapd_path:
                self.segs = load_border_segments(self.mapd_path)
                raw = lane_offset_via_map(slot, self.segs)
                if raw is not None and 2.40 <= abs(raw) <= 4.60:
                    lat = raw
                    self.lat_source = "地图实测"
                elif raw is not None:
                    self._say(f"[park] 地图给出的横向偏置 {raw:+.2f} m 不像行车车道，改用经验值")
        except Exception as exc:                        # noqa: BLE001
            self._say(f"[park] 地图读取失败，退回经验偏置: {exc!r}")
        if lat is None:
            lat = slot.width / 2.0 + 1.90               # 车位半宽 + 半个车道（参考实现 3.43）
            self.lat_source = "经验值(车位半宽+半车道)"
        lat = max(2.40, min(4.20, abs(lat)))             # 太贴会啃路缘，太远甩不进去
        self.lat_target = math.copysign(lat, X0 if abs(X0) > 0.05 else 1.0)
        self.stage_y = -(slot.length / 2.0 + self.ego_len / 2.0 + 0.8)
        self._build_wps(X0, Y0)
        self._say(f"[park] 车位 ({slot.x:.2f},{slot.y:.2f}) ori={slot.ori:.1f} "
                  f"长宽 {slot.length:.2f}x{slot.width:.2f} | 自车相对 X={X0:+.2f} "
                  f"Y={Y0:+.2f} yaw_t={_yaw0:+.1f} | 侧向偏置 {self.lat_target:+.2f}"
                  f"（{self.lat_source}） | staging Y={self.stage_y:+.2f}"
                  f" | 平台元素 '{getattr(ctx, 'platform_element', '') or '?'}'"
                  f" 限时 {getattr(ctx, 'time_limit', None)}"
                  f" end_point {getattr(ctx, 'end_point', None)}")
        self.phase = "ALIGN"

    def _build_wps(self, X0: float, Y0: float) -> None:
        """模板坐标系里铺一条"开过去"的参考线：S 弯横移到车道，再直行到 staging。"""
        pts: list[_Pt] = []
        span = max(6.0, min(abs(Y0 - self.stage_y) * 0.55, 28.0))
        y_turn_end = max(self.stage_y + 4.0, Y0 - span)
        steps = max(8, int(span / 0.5))
        for i in range(steps + 1):
            t = i / float(steps)
            f = t * t * (3.0 - 2.0 * t)                  # smoothstep
            pts.append(_Pt(X0 + (self.lat_target - X0) * f, Y0 + (y_turn_end - Y0) * t))
        y = y_turn_end
        while y > self.stage_y - 4.0:
            y -= 0.5
            pts.append(_Pt(self.lat_target, y))
        self.wps = pts
        self.idx = 0

    def _project(self, X: float, Y: float):
        """在参考折线上找最近投影：返回 (px, py, 切向单位向量, 弧长下标小数)。"""
        pts = self.wps
        best = (pts[0].x, pts[0].y, 1.0, 0.0, 0.0)       # px,py,dx,dy,frac
        best_d = 1e9
        for i in range(len(pts) - 1):
            ax, ay = pts[i].x, pts[i].y
            bx, by = pts[i + 1].x, pts[i + 1].y
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            if L2 < 1e-9:
                continue
            t = max(0.0, min(1.0, ((X - ax) * vx + (Y - ay) * vy) / L2))
            px, py = ax + t * vx, ay + t * vy
            d = math.hypot(px - X, py - Y)
            if d < best_d:
                L = math.sqrt(L2)
                best_d = d
                best = (px, py, vx / L, vy / L, i + t)
        return best

    def _pursuit(self, X: float, Y: float, yaw_t: float, look: float = 6.0):
        """模板坐标系里的 Stanley 式跟踪：航向误差 + 横向偏差（比纯跟踪稳得多）。

        2026-10-25 教训：原来用"欧氏距离 6 m 的点"当前视点，一旦车越过参考线就找不到
        候选点、退化去追最后一个点（30 m 外），车直接画圈跑飞（单场景实测 x 从 -6.35
        跑到 +8.5）。改成"先投影到折线、再沿弧长前进 look"，横向偏差用带符号距离。
        """
        px, py, tx_dir, ty_dir, frac = self._project(X, Y)
        # 前视点：沿折线弧长前进 look（折线点距 0.5 m）
        step = 0.5
        j = frac + look / step
        i0 = min(int(math.floor(j)), len(self.wps) - 1)
        t0 = j - i0
        if i0 + 1 < len(self.wps):
            tx_pt = (self.wps[i0].x + (self.wps[i0 + 1].x - self.wps[i0].x) * t0,
                     self.wps[i0].y + (self.wps[i0 + 1].y - self.wps[i0].y) * t0)
        else:
            tx_pt = (self.wps[i0].x, self.wps[i0].y)
        yaw_ref = math.degrees(math.atan2(-ty_dir, tx_dir))
        e_head = wrap180(yaw_ref - yaw_t)
        # 车相对参考线"左侧"的距离（左为正）。正舵 = 顺时针 = 向右，
        # 所以"车在参考线左边（e_ct>0）"要往右打 → +K_CROSS*e_ct。
        e_ct = -(X - px) * ty_dir + (Y - py) * tx_dir
        steer = self._K_HEAD * e_head + self._K_CROSS * e_ct
        _ = tx_pt                                       # 保留前视点，便于将来换回纯跟踪
        return max(-1.0, min(1.0, steer))

    def _envelope(self, X: float, Y: float, yaw_t: float) -> tuple[float, float, float, float]:
        """模板坐标系里自车矩形的轴向包络（横向 X、纵向 Y），用来和库边线比。"""
        th = math.radians(yaw_t)
        half_x = abs(self.ego_len / 2.0 * math.cos(th)) + abs(self.ego_wid / 2.0 * math.sin(th))
        half_y = abs(self.ego_len / 2.0 * math.sin(th)) + abs(self.ego_wid / 2.0 * math.cos(th))
        return X - half_x, X + half_x, Y - half_y, Y + half_y

    # ------------------------------------------------------------------ 主入口
    def step(self, ctx: ElementContext) -> Command | None:
        if self.frame is None:
            slot = find_slot(ctx.obstacles)
            if slot is None:
                return None                              # 还没看到车位：交回基线
            self._prepare(ctx, slot)
        slot = self.frame
        X, Y, yaw_t = self.to_t(ctx.x, ctx.y, ctx.yaw)

        if self.phase == "ALIGN":
            steer = self._pursuit(X, Y, yaw_t)
            dist = max(0.0, Y - self.stage_y)
            # 对齐段剖面：巡航上限 6 m/s、按 1.0 m/s² 规划减速（elem12-r2 的 1600 版本）。
            # 3.2 m/s + 0.5 m/s² 的慢剖面要多花 ~9 s 才到 staging，平台按元素计时扣时间。
            v_des = min(C.PARK_ALIGN_MS, math.sqrt(2.0 * C.PARK_ALIGN_DECEL_MS2 * dist))
            err = v_des - ctx.speed
            thr = max(0.0, min(0.30, 0.30 * err))
            # 2026-10-28：入口刹车封到 0.08。23:02 那局从元素 11 的 ~7.9 m/s 收进来时
            # 用了 0.12（比例项截顶），0.50 s 那几帧逐帧 −5.7 / −6.65 m/s²，正好把
            # 「平行车道泊车」的 +10 打掉（结算 200 而不是 210）。0.08 的减速更平缓，
            # 到 staging 只多花 1~2 s（元素限时 200 s，本局泊车段只用了 52 s）。
            brk = max(0.0, min(0.08, 0.30 * (-err)))
            if dist <= 0.7:
                self.phase = "STOP"
                self._say(f"[park] 到 staging（X={X:+.2f} Y={Y:+.2f} yaw_t={yaw_t:+.1f}，"
                          f"差 {dist:.2f} m），准备倒车")
                return Command(steer=0.0, throttle=0.0, brake=PARALLEL_BRAKE_CMD,
                               gear=GearMode.DRIVE, final=True, note="park:stop")
            return Command(steer=float(steer), throttle=float(thr), brake=float(brk),
                           gear=GearMode.DRIVE, final=True,
                           note="park:align %.1fm X%+.2f" % (dist, X))

        if self.phase == "STOP":
            if abs(ctx.speed) <= 0.10:
                self.ctl = ParallelParkingController()
                # FIT 容差分开给：横向必须容得下 PARK_SLOT_BIAS_M（落点回正时控制器看到的
                # dx≈+0.29），纵向按参考实现的 15 cm 收紧；20 s 内收不拢由 PARK 段统一放宽。
                try:
                    import metacar_race.control.parallel_parking as _pp
                    _pp.PARALLEL_TOL_DX_M = C.PARK_TOL_DX_M
                    _pp.PARALLEL_TOL_DY_M = C.PARK_TOL_DY_M
                except Exception:
                    pass
                self.fit_t = 0.0
                self.fit_relaxed = False
                # 直接跳到 STOP：参考控制器自己还有一段 APPROACH（从远处开过来），
                # 但它的前进油门上限 `PARALLEL_THROTTLE_MAX = 0.01` 是给"怠速倒车"
                # 标定的，用来前进根本推不动车。我们已经在 ALIGN 里开到位并停稳，
                # 交班时让它从 STOP 起手，下一帧就进 ARC1（实测 p3 卡死 166 s）。
                self.ctl.phase = "STOP"
                self.phase = "PARK"
                self._say("[park] 停稳，开始两段圆弧倒车入库")
            else:
                return Command(steer=0.0, throttle=0.0, brake=PARALLEL_BRAKE_CMD,
                               gear=GearMode.DRIVE, final=True, note="park:stop")

        if self.phase in ("PARK", "DONE") and self.ctl is not None:
            pose_t = SimpleNamespace(pos_x=X, pos_y=Y, ori_z=yaw_t)
            # 车位中心沿 -X 偏 PARK_SLOT_BIAS_M：参考控制器按 lateral=|sx-x| 算圆弧角度，
            # 而它的舵角斜坡会让实际横移恒少 ≈0.29 m（四局实测 dx=+0.266~+0.318），
            # 偏置后设计横移变大、落点回到真实车位中心。偏置只影响它自己的坐标系。
            obs_t = [SimpleNamespace(
                type=SimpleNamespace(value=PARALLEL_SLOT_TYPE, name="PARKING_SLOT"),
                pos_x=-C.PARK_SLOT_BIAS_M, pos_y=0.0, ori_z=90.0,
                length=slot.length, width=slot.width)]
            # 邻车也要一起扔进模板坐标系（参考实现靠"远端那辆车"定 staging 点）。
            # ctx.obstacles 是 ObstacleScan 的 dict，type 是 int（CAR = 6）。
            for o in ctx.obstacles or ():
                f = _slot_fields(o)
                if f is None:
                    continue
                is_car = (f["type"] == _CAR_TYPE
                          or getattr(getattr(o, "type", None), "name", "") == "CAR")
                if not is_car:
                    continue
                cx, cy, _ = self.to_t(f["x"], f["y"], f["ori"])
                obs_t.append(SimpleNamespace(
                    type=SimpleNamespace(value=_CAR_TYPE, name="CAR"),
                    pos_x=cx, pos_y=cy, ori_z=90.0,
                    length=f["len"] or 4.0, width=f["wid"] or 1.8))
            vc, info = self.ctl.update(pose_t, ctx.speed, obs_t, ctx.dt)
            # FIT 收紧到 15 cm 后可能一直微调不到位（0.010 的怠速油门一次只挪几厘米）：
            # 20 s 还没收敛就把容差放宽回 PARK_TOL_RELAX_M，先停进去再说。
            if info.phase == "FIT" and not self.fit_relaxed:
                self.fit_t += ctx.dt
                if self.fit_t >= C.PARK_FIT_TIMEOUT_S:
                    self.fit_relaxed = True
                    try:
                        import metacar_race.control.parallel_parking as _pp
                        _pp.PARALLEL_TOL_DX_M = C.PARK_TOL_RELAX_M
                        _pp.PARALLEL_TOL_DY_M = C.PARK_TOL_RELAX_M
                    except Exception:
                        pass
                    self._say("[park] FIT 收紧 %.0f s 仍未收敛，容差退回 %.2f m"
                              % (self.fit_t, C.PARK_TOL_RELAX_M))
            # 兜底：万一控制器自己又退回 APPROACH（它的 stage_y 和我们算的差 0.2 m），
            # 0.01 的前进油门会把它憋死在原地。停着不动超过 1.5 s 就强行跳到倒车段。
            if info.phase == "APPROACH" and abs(ctx.speed) < 0.05:
                self.approach_stall += ctx.dt
                if self.approach_stall > 1.5:
                    self.ctl.phase = "STOP"
                    self._say("[park] 控制器停在 APPROACH 推不动车，强制进入倒车段")
                    self.approach_stall = 0.0
            else:
                self.approach_stall = 0.0
            # 倒车油门是 0.010 级的"怠速量"，卡住不动就按 3 倍往上试（只在本帧放大，
            # 一旦动起来立刻退回原标定，避免 0.010→大油门那种窜车）。
            if abs(ctx.speed) < 0.06 and info.phase in ("ARC1", "ARC2", "FIT"):
                self.stall_t += ctx.dt
            else:
                self.stall_t = 0.0
            if self.stall_t > 2.5 and float(vc.throttle) > 0.0:
                vc = SimpleNamespace(gear=vc.gear, throttle=float(vc.throttle) * 3.0,
                                     brake=float(vc.brake), steering=float(vc.steering))
                if self.stall_t > 2.5 and not self._stall_said:
                    self._stall_said = True
                    self._say("[park] 倒车起步卡住，油门临时放大 3 倍")
            elif abs(ctx.speed) > 0.15:
                self._stall_said = False
            if self.ctl.done() and self.phase != "DONE":
                self.phase = "DONE"
                lo_la, hi_la, lo_lo, hi_lo = self._envelope(X, Y, yaw_t)
                self._say(f"[park] 入库完成 dx={info.dx:+.2f} dy={info.dy:+.2f} "
                          f"（真实 X={X:+.2f} Y={Y:+.2f}，偏置 {-C.PARK_SLOT_BIAS_M:+.2f}）"
                          f"yaw_err={info.yaw_error:.1f}° theta={info.theta:.1f}° | "
                          f"横向 {lo_la:+.2f}..{hi_la:+.2f}（库边 ±{slot.width / 2.0:.2f}）"
                          f" 纵向 {lo_lo:+.2f}..{hi_lo:+.2f}（库边 ±{slot.length / 2.0:.2f}）"
                          f" | 平台元素 '{getattr(ctx, 'platform_element', '') or '?'}'")
            # 低速段（|v|<1.0）给油门/刹车做速率限制：入库时车速只有 0.3~0.6 m/s，
            # 控制器 0.06 的刹车一帧就能甩出 −5.6 m/s²，平台按"加速度超限"记一次，
            # 泊车的 +10 就没了（2026-10-29 15:26 那局 42.44 s 的 +5.6 就是这样）。
            thr_out, brk_out = float(vc.throttle), float(vc.brake)
            if abs(ctx.speed) < 1.0 and info.phase != "DONE":
                thr_out = min(max(thr_out, self._thr_prev - 0.02),
                              self._thr_prev + 0.02)
                brk_out = min(max(brk_out, self._brk_prev - 0.015),
                              self._brk_prev + 0.015)
                if abs(ctx.speed) < 0.6:      # 0.1 的刹车在 0.38 m/s 上就是 −5.3 m/s²
                    brk_out = min(brk_out, 0.045)
            self._thr_prev, self._brk_prev = thr_out, brk_out
            return Command(steer=float(vc.steering), throttle=thr_out,
                           brake=brk_out, gear=vc.gear, final=True,
                           note="park:%s dx=%+.2f dy=%+.2f" % (info.phase, info.dx, info.dy))

        # DONE：驻车挡保持，等平台收尾
        return Command(steer=0.0, throttle=0.0, brake=1.0,
                       gear=GearMode.PARKING, final=True, note="park:hold")

    def sync(self, x: float, y: float) -> None:
        """瞬移后重新识别车位（几何没变就继续）。"""
        self.frame = None
        self.ctl = None
        self.phase = "WAIT"

    def reset(self) -> None:
        self.sync(0.0, 0.0)


__all__ = ["SlotFrame", "SlotParkStrategy", "find_slot", "find_mapd",
           "load_border_segments", "lane_offset_via_map", "DEFAULT_SCENE_ROOT"]
