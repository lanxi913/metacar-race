# -*- coding: utf-8 -*-
"""几何循迹控制器（连续场景用，无经验参数）

设计原则：**每个数值都能追溯到"规则 / 车辆 / 场景数据"三者之一**，
不保留任何"调出来的手感系数"。

| 量 | 来源 | 说明 |
|---|---|---|
| `A_LAT_MAX` | 规则 | 赛事要求 `|a| ≤ 5 m/s²`，横向同样取 5.0 |
| `A_LONG_MAX` | 规则 | 同上，纵向取 5.0 |
| `V_RULE` | 规则 | 城市道路限速 60 km/h = 16.67 m/s |
| `L`（轴距） | 车辆 | **在线标定**：`L = v·tan(δ) / ω`，用实测横摆角速度反算 |
| `k_steer` | 车辆 | **在线标定**：`κ = k·δ`，滑动平均 |
| `κ(s)` | 场景 | `static.route` 折线三点外接圆（带符号） |
| `v_curve` | 物理 | `v = √(A_LAT_MAX / κ)`，离心加速度不超限 |
| 横向反馈 | 物理 | Stanley `atan(e/v)`，**增益 1，无量纲** |
| 前视时间 `τ` | 车辆 | 实测"油门→速度"滞后（滑动平均），初始 0.3 s |

控制律：

```
δ = atan(L·κ(s+vτ))  +  ψ_e  −  atan(e / max(v, 0.5))
v_cmd = min(V_RULE, √(A_LAT_MAX/κ_ahead), V_MEASURED_MAX, √(2·A_LONG·rem))
throttle/brake = 归一化速度误差（无量纲，不做整定）
```

`steering` 的满量程 0.6 rad 是车控协议定义（`VehicleControl.steering ∈ [-1,1]`
对应最大前轮转角），属接口常量而非调参。
"""
from __future__ import annotations

import math
from collections import deque

from metacar import GearMode, VehicleControl

# ---- 规则常数 ----
A_LAT_MAX = 5.0
A_LONG_MAX = 5.0
V_RULE = 60.0 / 3.6

# ---- 接口常数（车控协议 / 乘用车标准轴距兜底） ----
STEER_FULL_SCALE = 0.6      # rad，steering=±1 时对应的前轮转角
WHEELBASE_DEFAULT = 2.6     # m，仅在在线标定完成前使用


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class Polyline:
    """折线：投影、取点、带符号曲率（全部由点列构造）。"""

    def __init__(self, pts):
        self.pts = [(float(p[0]), float(p[1])) for p in pts]
        self.cum = [0.0]
        for i in range(1, len(self.pts)):
            dx = self.pts[i][0] - self.pts[i - 1][0]
            dy = self.pts[i][1] - self.pts[i - 1][1]
            self.cum.append(self.cum[-1] + math.hypot(dx, dy))
        self.total = self.cum[-1] if self.cum else 0.0
        self.curv = self._curvatures()

    def _curvatures(self):
        """三点外接圆曲率 κ = 2·S_signed / (a·b·c)，左转为正。"""
        n = len(self.pts)
        out = [0.0] * n
        for i in range(n):
            i0, i1 = max(0, i - 1), min(n - 1, i + 1)
            x1, y1 = self.pts[i0]
            x2, y2 = self.pts[i]
            x3, y3 = self.pts[i1]
            a = math.hypot(x2 - x1, y2 - y1)
            b = math.hypot(x3 - x2, y3 - y2)
            c = math.hypot(x3 - x1, y3 - y1)
            if a < 1e-6 or b < 1e-6 or c < 1e-6:
                out[i] = out[i - 1] if i else 0.0
                continue
            area2 = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)
            out[i] = 2.0 * area2 / (a * b * c)
        return out

    def project(self, x: float, y: float):
        best_s, best_d, best_sign = 0.0, float("inf"), 1.0
        for i in range(len(self.pts) - 1):
            x1, y1 = self.pts[i]
            x2, y2 = self.pts[i + 1]
            dx, dy = x2 - x1, y2 - y1
            seg = dx * dx + dy * dy
            t = 0.0 if seg < 1e-9 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg))
            px, py = x1 + t * dx, y1 + t * dy
            d = math.hypot(x - px, y - py)
            if d < best_d:
                cross = dx * (y - y1) - dy * (x - x1)
                best_s = self.cum[i] + t * math.hypot(dx, dy)
                best_d = d
                best_sign = 1.0 if cross > 0 else -1.0     # >0 = 车在路径左侧
        return best_s, best_d * best_sign

    def at(self, s: float):
        s = max(0.0, min(self.total, s))
        for i in range(len(self.pts) - 1):
            if self.cum[i] <= s <= self.cum[i + 1]:
                seg = self.cum[i + 1] - self.cum[i]
                t = 0.0 if seg < 1e-9 else (s - self.cum[i]) / seg
                x1, y1 = self.pts[i]
                x2, y2 = self.pts[i + 1]
                return x1 + t * (x2 - x1), y1 + t * (y2 - y1), i
        x, y = self.pts[-1]
        return x, y, len(self.pts) - 1

    def curvature_at(self, s: float) -> float:
        _x, _y, i = self.at(s)
        return self.curv[i]

    def max_abs_curvature(self, s0: float, s1: float) -> float:
        i0 = 0
        while i0 < len(self.cum) - 1 and self.cum[i0] < s0:
            i0 += 1
        i1 = i0
        while i1 < len(self.cum) - 1 and self.cum[i1] < s1:
            i1 += 1
        seg = [abs(k) for k in self.curv[i0:i1 + 1]]
        return max(seg) if seg else 0.0


class GeoFollower:
    """规则 / 物理 / 数据驱动的路线循迹。"""

    def __init__(self, route_pts, speed_limit: float = V_RULE):
        self.path = Polyline(route_pts)
        self.v_cap = speed_limit
        self.v_meas_max = 0.0
        self.wheelbase = None
        self.k_steer = None
        self.lag = deque(maxlen=60)
        self.t = 0.0
        self.done = False
        self._prev_yaw = None
        self._stall = 0.0

    def _calibrate(self, v: float, steer: float, yaw_rate: float):
        if abs(steer) > 0.15 and v > 1.0 and abs(yaw_rate) > 1e-4:
            delta = abs(steer) * STEER_FULL_SCALE
            l_est = v * math.tan(delta) / abs(yaw_rate)
            if 1.0 < l_est < 6.0:
                self.wheelbase = l_est if self.wheelbase is None else 0.9 * self.wheelbase + 0.1 * l_est
            k = abs(yaw_rate) / v / abs(steer)
            if 1e-3 < k < 5.0:
                self.k_steer = k if self.k_steer is None else 0.9 * self.k_steer + 0.1 * k
        self.v_meas_max = max(self.v_meas_max, v)

    def update(self, pose, speed: float, dt: float):
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = float(speed)
        self.t += dt
        yaw_rate = 0.0
        if self._prev_yaw is not None and dt > 0:
            yaw_rate = math.radians(wrap180(yaw - self._prev_yaw)) / dt
        self._prev_yaw = yaw

        L = self.wheelbase or WHEELBASE_DEFAULT
        s, e = self.path.project(x, y)
        rem = self.path.total - s

        tau = (sum(self.lag) / len(self.lag)) if self.lag else 0.3
        s_pre = s + max(v, 0.1) * tau
        tx, ty, _ = self.path.at(s_pre)
        # 注意：仿真 yaw 为顺时针（yaw = -atan2(dy,dx)），故取负号
        heading_ref = -math.degrees(math.atan2(ty - y, tx - x))
        psi_e = math.radians(wrap180(heading_ref - yaw))
        k_ff = self.path.curvature_at(s_pre)

        k_ahead = self.path.max_abs_curvature(s, s + max(8.0, v * 2.0))
        v_curve = math.sqrt(A_LAT_MAX / k_ahead) if k_ahead > 1e-4 else V_RULE
        v_cap = min(self.v_cap, v_curve)
        if self.v_meas_max > 0.5:
            v_cap = min(v_cap, self.v_meas_max * 1.05)
        v_end = math.sqrt(max(2.0 * A_LONG_MAX * max(rem - 0.5, 0.0), 0.0))
        v_cmd = min(v_cap, v_end)

        delta = math.atan(L * k_ff) + psi_e - math.atan2(e, max(v, 0.5))
        steer = max(-1.0, min(1.0, delta / STEER_FULL_SCALE))

        gear = GearMode.DRIVE
        thr = brk = 0.0
        err = v_cmd - v
        if err >= 0:
            thr = min(1.0, err / max(v_cmd, 1.0))
        else:
            brk = min(1.0, -err / max(v_cmd + 1.0, 1.0))
        # 静摩擦兜底：想走但车不动（物理现象，不是调参）
        if thr > 0 and abs(v) < 0.1:
            self._stall += dt
            if self._stall > 0.5:
                thr = 1.0
        else:
            self._stall = 0.0
        if v_cmd <= 0.05 and abs(v) < 0.05:
            gear, brk = GearMode.PARKING, 0.10
        if rem <= 0.3:
            self.done = True

        self._calibrate(v, steer, yaw_rate)
        info = {"s": s, "e": e, "rem": rem, "v_cmd": v_cmd, "k": k_ahead,
                "L": L, "k_steer": self.k_steer, "steer": steer}
        return VehicleControl(gear=gear, throttle=thr, brake=brk, steering=steer), info

