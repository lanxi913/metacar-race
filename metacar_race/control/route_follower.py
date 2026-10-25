# -*- coding: utf-8 -*-
"""路线级循迹控制器（通用，无写死坐标）

2026-10-23 实测：场景元素 12「辅路驶入、驶出」总得分 **110.00**
（任务 100 + 加速度 10，扣分 0），用时 99 s（限时 200 s）。

适用：参考路径完全由 `static.route` 给出的场景（辅路、匝道、连续转弯等）。

## 为什么不写经验参数

| 量 | 来源 |
|---|---|
| 参考路径 | `static.route`（场景给的点列） |
| 目标速度 | `min(巡航, √(2a·剩余))` |
| 横向控制 | 纯跟踪，前视 `1.5 + 0.45·v` |
| 终点判定 | 距 `route[-1]` 小于 0.3 m 开始收尾 |

## 三个实测要点

1. **收尾不能硬停**：到点直接挂 P 会一帧把 4.8 m/s 打到 0（远超 5 m/s²），
   加速度分丢失；必须轻刹到 0.05 m/s 以下再挂 P 保持 5 s。
2. **转向限幅 0.8**：满舵 + 低速会顶死静摩擦。
3. **长路线别做曲线限速**：443 m 的路线 + 4.8 m/s 的车速，卡点全在时间上，
   多余的减速会直接超时；弯不急时按巡航速度跑。
"""
from __future__ import annotations

import math

from metacar import GearMode, VehicleControl

from metacar_race.config import (
    ROUTE_ACCEL_MAX,
    ROUTE_CRUISE_MS,
    ROUTE_HOLD_SECONDS,
    ROUTE_STEER_LIMIT,
    ROUTE_WHEELBASE,
)

MAX_STEER_RAD = 0.6


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class RouteFollower:
    """沿 static.route 循迹到终点。"""

    def __init__(self, static) -> None:
        self.pts = [(float(p.x), float(p.y)) for p in static.route]
        self.cum = [0.0]
        for i in range(len(self.pts) - 1):
            self.cum.append(self.cum[-1] + math.hypot(
                self.pts[i + 1][0] - self.pts[i][0], self.pts[i + 1][1] - self.pts[i][1]))
        self.total = self.cum[-1]
        self.goal = self.pts[-1]
        self.thr = 0.0
        self.brk = 0.0
        self.hold = 0.0
        self.phase = "DRIVE"

    def _project(self, x: float, y: float) -> float:
        """返回自车在路线上的弧长坐标 s。"""
        best = (1e18, 0.0)
        for i in range(len(self.pts) - 1):
            x0, y0 = self.pts[i]
            x1, y1 = self.pts[i + 1]
            dx, dy = x1 - x0, y1 - y0
            L2 = dx * dx + dy * dy
            if L2 <= 1e-9:
                continue
            t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
            px, py = x0 + t * dx, y0 + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best[0]:
                best = (d2, self.cum[i] + t * math.sqrt(L2))
        return best[1]

    def _at(self, s: float) -> tuple[float, float]:
        s = max(0.0, min(self.total, s))
        for i in range(len(self.pts) - 1):
            if self.cum[i] + (self.cum[i + 1] - self.cum[i]) >= s:
                L = max(self.cum[i + 1] - self.cum[i], 1e-9)
                t = (s - self.cum[i]) / L
                return (self.pts[i][0] + t * (self.pts[i + 1][0] - self.pts[i][0]),
                        self.pts[i][1] + t * (self.pts[i + 1][1] - self.pts[i][1]))
        return self.pts[-1]

    def update(self, pose, speed: float, dt: float) -> tuple[VehicleControl, dict]:
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        s = self._project(x, y)
        rem = math.hypot(self.goal[0] - x, self.goal[1] - y)

        ld = max(2.5, min(9.0, 1.5 + 0.45 * abs(speed)))
        txp, typ = self._at(min(s + ld, self.total))
        desired = -math.degrees(math.atan2(typ - y, txp - x))
        alpha = math.radians(wrap180(desired - yaw))
        delta = math.atan2(2 * ROUTE_WHEELBASE * math.sin(alpha), ld)
        steer = max(-ROUTE_STEER_LIMIT, min(ROUTE_STEER_LIMIT, delta / MAX_STEER_RAD))

        v_des = min(ROUTE_CRUISE_MS, math.sqrt(max(2 * ROUTE_ACCEL_MAX * max(rem - 0.4, 0.0), 0.0)))
        thr = brk = 0.0
        gear = GearMode.DRIVE
        vc_phase = "DRIVE"
        if rem <= 0.30 and abs(speed) <= 0.05:
            self.hold += dt
            gear, brk = GearMode.PARKING, 0.10
            vc_phase = "HOLD"
            if self.hold >= ROUTE_HOLD_SECONDS:
                vc_phase = "DONE"
        elif rem <= 1.2:
            brk = min(0.06, self.brk + 0.004)
            self.brk = brk
            vc_phase = "STOP"
        elif speed > v_des + 0.25:
            brk = min(0.10 if speed > v_des + 0.8 else 0.05, self.brk + 0.004)
            self.brk = brk
        else:
            target = min(1.0, max(0.0, (v_des - speed) / 0.8)) * 0.05
            self.thr = min(target, self.thr + 0.003) if target > self.thr else max(target, self.thr - 0.003)
            thr = self.thr
            self.brk = 0.0
        self.phase = vc_phase
        return (VehicleControl(gear=gear, throttle=thr, brake=brk, steering=steer),
                {"phase": vc_phase, "s": s, "rem": rem, "total": self.total})

    def done(self) -> bool:
        return self.phase == "DONE"

