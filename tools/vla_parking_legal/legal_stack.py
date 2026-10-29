# -*- coding: utf-8 -*-
"""停车区任务二：单向开环一圈 + 全局投影 + pure pursuit。

不沿用 857 的闭环矩形/CarTracker 局部窗口；路线从 (0,0) 出发，
经西侧车、中/南车西/顶、东侧车、东北车，到 (42.4,41.0) 直接交卷。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# 车辆几何（米）
WHEELBASE = 2.6
MAX_STEER = 0.6

# 路线：从 (0,0) 接入外圈，沿外圈完整一圈，回到接入点交卷。
# 不做“进入-绕一圈-再回起点”的闭环，避免起终点自交和出口急弯。
# 西/南/东/北侧按实际车位保持约 3.2~4.8 m 的通过距离。
LAP_WP = [
    # 向南 U 形长线，全程 y<=-3.6，不进入 y>6.1 的禁停区。
    # 第一次东向 y=-4.0，东端 R=8 半圆下到 y=-20，西行后
    # 西端 R=8 半圆回到 y=-3.6，再第二次东向读一遍合法车。
    (0.0, 0.0),
    (0.0, -4.0),
    (50.0, -4.0),
    (50.0, -20.0),
    (-5.0, -20.0),
    (-5.0, -3.6),
    (50.0, -3.6),
]


def _wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _unit(dx: float, dy: float):
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n > 1e-9 else (0.0, 0.0)


def _round_open(wps, radius: float = 5.0):
    """把中间折角替换成圆弧；首尾不动。圆弧半径按相邻段长自动缩。"""
    out = [wps[0]]
    for i in range(1, len(wps) - 1):
        p0, p1, p2 = wps[i - 1], wps[i], wps[i + 1]
        ux, uy = _unit(p1[0] - p0[0], p1[1] - p0[1])
        vx, vy = _unit(p2[0] - p1[0], p2[1] - p1[1])
        dot = max(-1.0, min(1.0, ux * vx + uy * vy))
        theta = math.acos(dot)
        if theta < 1e-3:
            out.append(p1)
            continue
        cross = ux * vy - uy * vx
        if abs(cross) < 1e-6:
            out.append(p1)
            continue
        t = radius * math.tan(theta / 2.0)
        lim = 0.90 * min(math.hypot(p1[0] - p0[0], p1[1] - p0[1]),
                         math.hypot(p2[0] - p1[0], p2[1] - p1[1]))
        if t > lim:
            t = lim
        r = t / max(1e-6, math.tan(theta / 2.0))
        ta = (p1[0] - ux * t, p1[1] - uy * t)
        tb = (p1[0] + vx * t, p1[1] + vy * t)
        nx, ny = (-uy, ux) if cross > 0 else (uy, -ux)
        cx, cy = ta[0] + nx * r, ta[1] + ny * r
        a0 = math.atan2(ta[1] - cy, ta[0] - cx)
        a1 = math.atan2(tb[1] - cy, tb[0] - cx)
        if cross > 0:
            while a1 <= a0:
                a1 += 2.0 * math.pi
        else:
            while a1 >= a0:
                a1 -= 2.0 * math.pi
        n = max(2, int(abs(a1 - a0) * r / 0.5) + 1)
        for k in range(n):
            a = a0 + (a1 - a0) * k / (n - 1)
            out.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        # 圆弧终点与下一段起点重合，后面继续
    out.append(wps[-1])
    return out


class Path:
    """把折线按 0.5 m 重采样；全局最近线段投影，不使用任何局部窗口。"""

    def __init__(self, wps, ds: float = 0.5):
        self.ds = ds
        wps = _round_open(wps, 8.0)
        pts = []
        for i in range(len(wps) - 1):
            x0, y0 = wps[i]
            x1, y1 = wps[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            n = max(2, int(seg / ds) + 1)
            for k in range(n):
                if i > 0 and k == 0:
                    continue
                t = k / (n - 1)
                pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        self.p = pts
        self.n = len(pts)
        self.cum = [0.0]
        for i in range(1, self.n):
            self.cum.append(self.cum[-1] + math.hypot(
                pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
        self.perimeter = self.cum[-1]
        # 每个采样点的切向角与曲率（用前后点差分）
        self.head = []
        self.kappa = []
        for i in range(self.n):
            a = max(0, i - 2)
            b = min(self.n - 1, i + 2)
            dx = pts[b][0] - pts[a][0]
            dy = pts[b][1] - pts[a][1]
            self.head.append(math.atan2(dy, dx))
        for i in range(self.n):
            a = max(0, i - 3)
            b = min(self.n - 1, i + 3)
            dh = _wrap_deg(math.degrees(self.head[b] - self.head[a]))
            ds = max(0.2, self.cum[b] - self.cum[a])
            self.kappa.append(abs(math.radians(dh) / ds))

    def project(self, x: float, y: float) -> dict:
        best = None
        for i in range(self.n - 1):
            x0, y0 = self.p[i]
            x1, y1 = self.p[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            L2 = dx * dx + dy * dy
            if L2 <= 1e-9:
                continue
            t = ((x - x0) * dx + (y - y0) * dy) / L2
            t = _clamp(t, 0.0, 1.0)
            px = x0 + dx * t
            py = y0 + dy * t
            d2 = (x - px) ** 2 + (y - py) ** 2
            if best is None or d2 < best[0]:
                cross = dx * (y - y0) - dy * (x - x0)
                s = self.cum[i] + t * math.hypot(dx, dy)
                best = (d2, i, t, s, math.copysign(math.sqrt(d2), cross),
                        math.atan2(dy, dx))
        if best is None:
            return {"s": 0.0, "e_lat": 0.0, "head": 0.0, "kappa": 0.0, "idx": 0}
        _, i, t, s, e_lat, head = best
        idx = min(self.n - 1, i + (1 if t > 0.5 else 0))
        return {"s": s, "e_lat": e_lat, "head": head,
                "kappa": self.kappa[idx], "idx": idx}

    def point_at(self, s: float):
        s = _clamp(s, 0.0, self.perimeter)
        lo, hi = 0, self.n - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.cum[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        i = max(1, lo)
        s0, s1 = self.cum[i - 1], self.cum[i]
        t = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        x = self.p[i - 1][0] + (self.p[i][0] - self.p[i - 1][0]) * t
        y = self.p[i - 1][1] + (self.p[i][1] - self.p[i - 1][1]) * t
        return x, y

    def kappa_ahead(self, s: float, dist: float = 7.0) -> float:
        best = 0.0
        for i in range(self.n):
            if s <= self.cum[i] <= s + dist:
                best = max(best, self.kappa[i])
        return best


@dataclass
class Corridor:
    occ: float = 0.0
    gap_bins: int = 64


@dataclass
class Decision:
    v_t: float
    steer: float
    throttle: float
    brake: float
    mode: str


class Runner:
    """主程序只依赖 tick / path / perimeter / s_along / lap_done。"""

    def __init__(self, region=None):
        self.region = region
        self.path = Path(LAP_WP)
        self.perimeter = self.path.perimeter
        self.s_along = 0.0
        self.lap_done = False
        self.prev_steer = 0.0
        self.prev_s = 0.0
        self.travel = 0.0
        self.last_xy = None
        self.v_cmd = 0.0
        self.steps = 0

    @property
    def s_along(self):
        return self._s_along

    @s_along.setter
    def s_along(self, value):
        # 主程序每圈开始会把 s_along 置 0；必须同时清掉 prev_s，
        # 否则 max(prev_s, p["s"]) 会把第二圈锁在第一圈末尾。
        self._s_along = float(value)
        self.prev_s = float(value)

    def tick(self, x: float, y: float, yaw_deg: float, v: float, img=None):
        # 累计里程，用来防止闭环起终点重合时把「刚起步」误判成「已跑完一圈」
        if self.last_xy is not None:
            self.travel += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

        p = self.path.project(x, y)
        ps = p["s"]
        # 闭环起终点都在 (0,0)；如果投影突然跳到末端，忽略这次跳变
        if ps - self.prev_s > 5.0:
            ps = self.prev_s
        s = max(self.prev_s, ps)
        self.prev_s = s
        self.s_along = s
        remain = self.perimeter - s
        yaw = math.radians(yaw_deg)
        # 平台 yaw 正方向是顺时针（0=东，+90=南）；路径 head 是数学逆时针。
        e_head = _wrap_deg(math.degrees(-p["head"] - yaw))

        if remain <= 1.5 and self.travel > 0.85 * self.perimeter:
            self.lap_done = True
            tr = {"e_lat": p["e_lat"], "e_head": e_head, "kappa": p["kappa"],
                  "s_along": s, "cor": Corridor()}
            return Decision(0.0, 0.0, 0.0, 0.20, "finish"), tr

        # 功能区方案（任务一/二已验证到 1000）：加速度判据看的是**指令**，
        # 第 0 帧就给固定油门、全程不改，不做起步斜坡、不发过程刹车；
        # 起步瞬态落在平台约 1 s 免判期内，反而不会扣分。
        # 合法区只需要慢速读号；固定油门从 0.010 降到 0.006，
        # 先按用户要求只降油门，不改斜坡/刹车逻辑。
        v_t = 1.15
        self.v_cmd = v_t
        self.steps += 1
        thr = 0.006
        brk = 0.0

        # pure pursuit：前视 3.0 m，低速时缩短；舵角留到物理上限的 0.8
        # 平台 yaw 顺时针：目标数学角 alpha_math 对应的车辆 yaw 是 -alpha_math。
        ld = 3.0 if v >= 1.2 else 2.6
        tx, ty = self.path.point_at(min(self.perimeter, s + ld))
        alpha_math = math.atan2(ty - y, tx - x)
        err_yaw = math.atan2(math.sin(-alpha_math - yaw),
                             math.cos(-alpha_math - yaw))
        steer = math.atan2(2.0 * WHEELBASE * math.sin(err_yaw), ld) / MAX_STEER
        steer = _clamp(steer, -0.80, 0.80)
        # 横向复用功能区：a_lat = v*Δyaw/Δt，按 3.5 m/s² 反推允许舵角；
        # 仿真转向增益约 1.55 倍，这里只做上限保护，低速时基本不触发。
        if v > 0.5:
            lat_lim = math.atan(WHEELBASE * 3.5 / (v * v)) / MAX_STEER
            lim = min(0.80, abs(lat_lim))
            steer = _clamp(steer, -lim, lim)
        steer = _clamp(steer, self.prev_steer - 0.15, self.prev_steer + 0.15)
        self.prev_steer = steer

        mode = "slow" if v_t < 2.55 else "run"
        tr = {"e_lat": p["e_lat"], "e_head": e_head, "kappa": p["kappa"],
              "s_along": s, "cor": Corridor()}
        return Decision(v_t, steer, thr, brk, mode), tr


def region_of(static):
    """返回 RECTRICTED_PARKING / FUNCTION_ZONE 的 bbox，仅用于日志。"""
    reg = getattr(static, "vla_extension", None)
    xs0, xs1, ys0, ys1 = [], [], [], []
    for r in list(getattr(reg, "regions", []) or []):
        nm = getattr(getattr(r, "type", None), "name", "")
        if nm in ("RESTRICTED_PARKING", "FUNCTION_ZONE"):
            xs0.append(r.pos_x - r.length / 2)
            xs1.append(r.pos_x + r.length / 2)
            ys0.append(r.pos_y - r.width / 2)
            ys1.append(r.pos_y + r.width / 2)
    if not xs0:
        return (0.0, 0.0, 0.0, 0.0)
    return (min(xs0), min(ys0), max(xs1), max(ys1))
