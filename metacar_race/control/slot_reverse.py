# -*- coding: utf-8 -*-
"""斜列/平行车位的「解析 S 弯 + 倒车纯跟踪」，带坡度适配。

为什么需要它
------------
平行泊车控制器（`parallel_parking.py`）用"开环两段圆弧 + 按航向切段"，
平地连续 5 次 210 分，但在 **15° 坡道** 上实测：

    ARC1 横移 0.62 m，ARC2 却横移 4.43 m（规划只需 3.11 m）
    → 车冲到 x=-11.42（车位中心 -10.09），顶到路缘后 v=0 卡死

坡上开环圆弧的横向位移和规划对不上，改成**先解析算路径、再倒车纯跟踪**，
跟踪误差由反馈修正。

坐标系（仿真：yaw 顺时针）
--------------------------
    dx/dt = v·cos(yaw),  dy/dt = −v·sin(yaw),  dyaw/dt = v·tan(δ)/L
倒车 v<0：δ>0 ⇒ yaw 减小 ⇒ 车尾往 −x 走（实测如此）。

流程：APPROACH（直行到车位侧后方 staging）→ PLAN（解析两段圆弧 + 直线）
→ REVERSE（倒车纯跟踪）→ SETTLE（停稳保持）
"""
from __future__ import annotations

import os
import math
from dataclasses import dataclass

from metacar import GearMode, VehicleControl

WHEELBASE = 2.60
MAX_STEER_RAD = 0.60
STEER_CMD = 0.75            # 打舵量（满舵会顶静摩擦，0.75 是平地验证值）
V_APPROACH = 2.60           # 对位段目标速度 m/s
V_REVERSE = 0.75            # 倒车目标速度 m/s
LOOKAHEAD = 0.9             # 倒车纯跟踪前视距离 m（小一点才贴得住圆弧）
STAGE_MARGIN = -3.00        # staging 往后再退 3 m：给两段圆弧留够横移距离
STAGE_SHIFT = -1.67        # 实测两段弧会多走 2.27 m，把 staging 前移同样的量抵消
CT_GAIN = 0.35              # 横向误差反馈增益（1/m 量纲）


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


@dataclass
class SlotInfo:
    phase: str
    dx: float
    dy: float
    yaw_err: float
    prog: float


class SlotReverseParker:
    def __init__(self, slope_thr: float = 0.0, slope_brake: float = 0.0) -> None:
        self.L = WHEELBASE
        self.radius = WHEELBASE / math.tan(STEER_CMD * MAX_STEER_RAD)
        self.slope_thr = slope_thr        # 重力前馈油门（坡道用）
        self.slope_brake = slope_brake    # 坡上停住时的最小刹车
        self.phase = "APPROACH"
        self.path: list[tuple[float, float]] = []
        self.slot: tuple[float, float, float] | None = None
        self.stage: tuple[float, float] = (0.0, 0.0)
        self.idx = 0
        self.hold = 0.0
        self.lane_x: float | None = None
        self.breaks: list[int] = [0, 0, 0]
        self.stuck = 0.0
        self.fine_t = 0.0
        self.thr_lim = 0.0     # 油门斜率限制后的实际输出
        self.fine_sub = "Y"    # FINE 子阶段：Y=先修纵向，YAW=再修航向

    # ---------- 场景解析 ----------
    def _parse(self, obstacles, x: float) -> None:
        if self.slot is not None:
            return
        cars: list[tuple[float, float, float]] = []
        for o in obstacles:
            t = getattr(o, "type", None)
            if getattr(t, "value", None) == 30:
                self.slot = (float(o.pos_x), float(o.pos_y), float(o.ori_z))
            elif getattr(t, "name", "") == "CAR":
                cars.append((float(o.pos_x), float(o.pos_y), float(o.length)))
        if self.slot is not None:
            sx, sy, syaw = self.slot
            self.lane_x = x
            # 必须先开过车位（y 更小侧），再倒车 +Y 进空档；
            # 停在车位前方会变成"倒着开走"，方向就反了（实测 y 从 4.6 涨到 19）
            far = [c for c in cars if c[1] < sy]
            if far:
                stage_y = far[0][1] + STAGE_MARGIN
            else:
                stage_y = sy - 3.0
            self.stage = (x, stage_y)

    # ---------- 路径生成（从 staging 到车位中心） ----------
    def _plan(self, x0: float, y0: float, yaw0: float,
              sx: float, sy: float, syaw: float) -> list[tuple[float, float]]:
        r = self.radius
        delta = STEER_CMD * MAX_STEER_RAD
        rad = math.radians(yaw0)
        nx, ny = math.sin(rad), math.cos(rad)        # 车头右法向
        lateral = abs((sx - x0) * nx + (sy - y0) * ny)
        cos_t = max(-0.99, min(0.99, 1.0 - lateral / (2.0 * r)))
        theta = math.degrees(math.acos(cos_t))

        pts: list[tuple[float, float]] = []
        x, y, yaw = x0, y0, yaw0
        step = 0.05
        v = -V_REVERSE
        yaw_end1 = yaw0 - theta
        nav = 0.0
        len1 = 0
        while (yaw - yaw_end1) > 0.05 and nav < 200.0:
            yaw += math.degrees(v * math.tan(delta) / self.L * (step / abs(v)))
            x += v * math.cos(math.radians(yaw)) * (step / abs(v))
            y += -v * math.sin(math.radians(yaw)) * (step / abs(v))
            pts.append((x, y))
            nav += step
        len1 = len(pts)
        nav = 0.0
        while (yaw0 - yaw) > 0.05 and nav < 200.0:
            yaw += math.degrees(v * math.tan(-delta) / self.L * (step / abs(v)))
            x += v * math.cos(math.radians(yaw)) * (step / abs(v))
            y += -v * math.sin(math.radians(yaw)) * (step / abs(v))
            pts.append((x, y))
            nav += step
        len2 = len(pts) - len1
        syaw_rad = math.radians(syaw)
        fx, fy = math.cos(syaw_rad), -math.sin(syaw_rad)
        nav = 0.0
        while nav < 100.0:
            ahead = (sx - x) * fx + (sy - y) * fy
            if abs(ahead) < 0.15:
                break
            sgn = 1.0 if ahead > 0 else -1.0
            x += sgn * fx * step
            y += sgn * fy * step
            pts.append((x, y))
            nav += step
        # 记录换段位置（按点序号），倒车时用"走到哪一段"决定曲率前馈
        self.breaks = [len1, len1 + len2, len(pts)]
        return pts

    def _slew(self, thr_t: float) -> float:
        """油门斜率限制：加得慢、收得快。

        仿真的发动机对油门台阶有阶跃响应，直接给 0.20 会窜到 5+ m/s²，
        平台连报「加速度高于允许上限」。用 0.0015/帧 的斜坡爬上去就平顺。
        """
        if thr_t < self.thr_lim:
            self.thr_lim = thr_t
        else:
            self.thr_lim = min(thr_t, self.thr_lim + 0.0015)
        return self.thr_lim

    # ---------- 主循环 ----------
    def update(self, pose, speed: float, obstacles, dt: float):
        x, y = float(pose.pos_x), float(pose.pos_y)
        yaw = float(pose.ori_z)
        self._parse(obstacles, x)
        if self.slot is None:
            return (VehicleControl(gear=GearMode.DRIVE, brake=0.30),
                    SlotInfo("WAIT", math.inf, math.inf, math.inf, 0.0))
        sx, sy, syaw = self.slot
        dx, dy = x - sx, y - sy
        yaw_err = abs(wrap180(yaw - syaw))

        if self.phase == "APPROACH":
            stage_y = self.stage[1]
            rem = y - stage_y
            if rem <= 0.20:
                self.phase = "STOP"
                return (VehicleControl(gear=GearMode.DRIVE, brake=max(0.25, self.slope_brake)),
                        SlotInfo("STOP", dx, dy, yaw_err, 0.0))
            v_des = min(V_APPROACH, math.sqrt(max(2 * 0.6 * rem, 0.0)))
            if speed > v_des + 0.35:
                return (VehicleControl(gear=GearMode.DRIVE, brake=max(0.20, self.slope_brake)),
                        SlotInfo("APPROACH", dx, dy, yaw_err, 0.0))
            thr = 0.11 if speed < v_des else 0.05
            if self.slope_thr > 0:
                thr += self.slope_thr
            return (VehicleControl(gear=GearMode.DRIVE, throttle=self._slew(min(0.5, thr))),
                    SlotInfo("APPROACH", dx, dy, yaw_err, 0.0))

        if self.phase == "STOP":
            if abs(speed) > 0.06:
                return (VehicleControl(gear=GearMode.DRIVE, brake=max(0.25, self.slope_brake)),
                        SlotInfo("STOP", dx, dy, yaw_err, 0.0))
            self.path = self._plan(x, y, yaw, sx, sy, syaw)
            self.idx = 0
            self.phase = "REVERSE"
            return (VehicleControl(gear=GearMode.REVERSE, throttle=0.06, steering=0.0),
                    SlotInfo("REVERSE", dx, dy, yaw_err, 0.0))

        if self.phase == "REVERSE":
            if not self.path:
                self.phase = "SETTLE"
            best_i, best_d = self.idx, 1e9
            for i in range(self.idx, min(len(self.path), self.idx + 300)):
                px, py = self.path[i]
                d = (px - x) ** 2 + (py - y) ** 2
                if d < best_d:
                    best_d, best_i = d, i
            self.idx = best_i
            # ---- 曲率前馈 + 横向反馈（比纯跟踪稳：纯跟踪在等曲率圆弧上没有修正余量）----
            b1, b2, bn = self.breaks
            if best_i < b1:
                ff = +STEER_CMD          # 第一段圆弧（倒车、yaw 减小、车尾往 -x 走）
            elif best_i < b2:
                ff = -STEER_CMD          # 反向圆弧回正
            else:
                ff = 0.0                 # 沿车位轴线直线倒入
            i0 = min(best_i, len(self.path) - 2)
            px, py = self.path[i0]
            qx, qy = self.path[i0 + 1]
            tx, ty = qx - px, qy - py
            tn = math.hypot(tx, ty) or 1.0
            nx, ny = -ty / tn, tx / tn                     # 路径左法向
            e_ct = (x - px) * nx + (y - py) * ny           # 横向误差（正 = 在路径左侧）
            steer = max(-1.0, min(1.0, ff + CT_GAIN * e_ct))
            # 靠近车位中心就减速，并在 0.45 m 内停住（不再依赖"走到路径末端"）
            dist = math.hypot(dx, dy)
            # 终点减速剖面：把"停在车位中心"当成终点处理，
            # v = sqrt(2·a·剩余距离)，避免 1.5 m/s 冲进去后刹不住
            rem_c = max(-dy - 0.08, 0.0) if dy < 0 else 0.0
            v_target = min(V_REVERSE, math.sqrt(2.0 * 0.35 * rem_c) + 0.08)
            # 必须走完两段圆弧（best_i >= b2）才允许进精细对位，
            # 否则会在圆弧没走完时被截断，横向少走 1 m 以上
            # 冲过车位中心（dy>0.10）也要立刻交给精细对位，别继续往里挤
            if (dist < 1.5 or dy > 0.10) and yaw_err < 30.0:
                self.phase = "FINE"          # 最后 1.5 m 交给精细对位
                self.hold = 0.0
                return (VehicleControl(gear=GearMode.DRIVE, brake=0.25),
                        SlotInfo("FINE", dx, dy, yaw_err, 1.0))
            elif abs(speed) > V_REVERSE + 0.30:
                return (VehicleControl(gear=GearMode.REVERSE, brake=0.15, steering=steer),
                        SlotInfo("REVERSE", dx, dy, yaw_err, best_i / max(1, len(self.path))))
            # 最后一段沿车位轴线：车位中心在"车头方向"就用前进，否则倒车
            syaw_rad = math.radians(syaw)
            fx, fy = math.cos(syaw_rad), -math.sin(syaw_rad)
            ahead = (sx - x) * fx + (sy - y) * fy
            gear = GearMode.DRIVE if (best_i >= b2 and ahead > 0.10) else GearMode.REVERSE
            # 油门必须 ≥0.18 才推得动（实测 0.10~0.16 在坡上会卡住），用 PWM 控速
            thr = 0.20 if abs(speed) < v_target else 0.0
            # 最后 2 m 超速就主动刹（坡上下滑会让车越溜越快）
            if dist < 2.0 and abs(speed) > v_target + 0.15:
                return (VehicleControl(gear=gear, brake=0.20, steering=steer),
                        SlotInfo("REVERSE", dx, dy, yaw_err, best_i / max(1, len(self.path))))
            if self.slope_thr > 0 and gear is GearMode.REVERSE:
                thr += self.slope_thr
            # 坡上静止起步要更大油门，否则卡在静摩擦里（实测 v 一直为 0）
            # 注意：仿真里油门 0.30 反而会让车停住，0.12~0.18 才动
            if abs(speed) < 0.06:
                self.stuck += dt
                if self.stuck > 0.8:
                    thr = 0.16
            else:
                self.stuck = 0.0
            return (VehicleControl(gear=gear, throttle=self._slew(min(0.5, thr)),
                                   steering=steer),
                    SlotInfo("REVERSE", dx, dy, yaw_err, best_i / max(1, len(self.path))))

        # ---- FINE：最后 1.5 m 的精细对位 ----
        # 目标：沿车位轴向前后挪，同时用航向把横向误差压到 0；蠕行 0.12 m/s
        if self.phase == "FINE":
            self.fine_t += dt
            if os.environ.get("FINE_STOP"):
                self.phase = "SETTLE"
                self.hold = 0.0
                return (VehicleControl(gear=GearMode.DRIVE, brake=0.35),
                        SlotInfo("SETTLE", dx, dy, yaw_err, 1.0))
            dist = math.hypot(dx, dy)
            if self.fine_sub == "Y":
                # 1) 先把纵向 dy 修到 ±0.18：死区内直接停住，换向之前必须先停稳
                if abs(dy) < 0.18:
                    if abs(speed) < 0.06:
                        self.fine_sub, self.fine_t = "YAW", 0.0
                    return (VehicleControl(gear=GearMode.DRIVE, brake=0.25),
                            SlotInfo("FINE", dx, dy, yaw_err, 1.0))
                gear = GearMode.DRIVE if dy > 0 else GearMode.REVERSE
                if abs(speed) > 0.10:
                    return (VehicleControl(gear=gear, brake=0.20),
                            SlotInfo("FINE", dx, dy, yaw_err, 1.0))
                thr = 0.20 if abs(speed) < 0.10 else 0.0
                return (VehicleControl(gear=gear, throttle=self._slew(thr)),
                        SlotInfo("FINE", dx, dy, yaw_err, 1.0))
            # 2) 航向：前进打舵 + 倒车反舵，让 yaw 单向累积（每次只挪 0.3 m）
            if self.fine_sub == "YAW" and (yaw_err < 4.0 or self.fine_t > 12.0):
                self.fine_sub, self.fine_t = "X", 0.0
                return (VehicleControl(gear=GearMode.DRIVE, brake=0.30),
                        SlotInfo("FINE", dx, dy, yaw_err, 1.0))
            if self.fine_sub == "YAW":
                sign = 1.0 if wrap180(syaw - yaw) > 0 else -1.0
                phase_half = (self.fine_t % 2.4) < 1.2
                if phase_half:
                    gear, steer = GearMode.DRIVE, sign * 0.55
                else:
                    gear, steer = GearMode.REVERSE, -sign * 0.55
                if abs(speed) > 0.18:
                    return (VehicleControl(gear=gear, brake=0.20, steering=steer),
                            SlotInfo("FINE", dx, dy, yaw_err, 1.0))
                return (VehicleControl(gear=gear, throttle=self._slew(0.12), steering=steer),
                        SlotInfo("FINE", dx, dy, yaw_err, 1.0))
            # 3) 横向：同向打舵"前进+倒车"两次小动作，把 dx 拉到 0（航向基本不变）
            if abs(dx) < 0.25 or self.fine_t > 16.0:
                self.phase = "SETTLE"
                self.hold = 0.0
                return (VehicleControl(gear=GearMode.DRIVE, brake=0.30),
                        SlotInfo("SETTLE", dx, dy, yaw_err, 1.0))
            # 横移技巧：前进 +δ 与倒车 +δ 都往 -x 侧偏，且航向变化互相抵消
            sgn = 1.0 if dx > 0 else -1.0          # dx>0 ⇒ 需要往 -x 挪 ⇒ 用 +δ
            half = (self.fine_t % 2.6) < 1.3
            gear = GearMode.DRIVE if half else GearMode.REVERSE
            steer = sgn * 0.55
            if abs(speed) > 0.18:
                return (VehicleControl(gear=gear, brake=0.20, steering=steer),
                        SlotInfo("FINE", dx, dy, yaw_err, 1.0))
            return (VehicleControl(gear=gear, throttle=self._slew(0.12), steering=steer),
                    SlotInfo("FINE", dx, dy, yaw_err, 1.0))

        # SETTLE / DONE
        if abs(speed) > 0.06 and self.phase == "SETTLE":
            return (VehicleControl(gear=GearMode.DRIVE, brake=max(0.20, self.slope_brake)),
                    SlotInfo("SETTLE", dx, dy, yaw_err, 1.0))
        self.hold += dt
        if self.hold > 5.0:
            self.phase = "DONE"
        return (VehicleControl(gear=GearMode.PARKING, brake=max(0.30, self.slope_brake)),
                SlotInfo("SETTLE" if self.phase != "DONE" else "DONE", dx, dy, yaw_err, 1.0))

    def done(self) -> bool:
        return self.phase == "DONE"



























