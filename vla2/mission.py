# -*- coding: utf-8 -*-
"""VLA 停车区：**一圈 + 读到就偏出去 + 回到线上** 的合并状态机（骨架，未跑）。

复用：
  * 算法层：`vla_stack.RefPath / build_reference / Corridor / wrap180 / yaw_of` 与常数
  * 读号层：`vla_read.VoteBox / ReadEstimator / p_violation / backproject / to_world`

决策层五状态：follow → offset_out → pass → offset_in →（submit）
唯一被改造的量：**横向设定值 `e_lat_target`**（0 ↔ ±4.6 m），不换路线、不写时刻表、不倒车。
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

WS = Path(__file__).parent
for p in (WS, Path(r"D:\无人算法\metacar-race")):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from vla_stack import (  # noqa: E402
    A_ACC, BRK_MAX, K_CROSS, K_HEAD, MAX_STEER, STEER_LP, STEER_SLEW, THR_MAX,
    V_CAP, V_MIN, V_STRAIGHT, WHEELBASE, Corridor, RefPath, build_reference, wrap180,
    yaw_of,
)
from vla_read import (  # noqa: E402
    SUBMIT_MARGIN_S, TASK_SECONDS, CarObs, ReadEstimator, VoteBox, backproject,
    p_violation, to_world,
)

# —— 读号机动的几何/时间常数（全部可被现场量替换）——
D_REF = 6.5        # 读号距离（p_read 峰值区）
E_REF = 1.85       # 贴边距离（与 vla_stack 一致）
OFFSET_MAX = D_REF - E_REF          # = 4.65 → 取 4.6
SETPOINT_RATE = 0.35                # 横向设定值斜坡 m/s
SETPOINT_EPS = 0.25
TRIGGER_AHEAD_M = 22.0               # 触发提前量（4.6 m / 0.35 m/s × 2 m/s ≈ 26 m 路程）
PASS_LENGTH_M = 6.0                  # 侧向掠过长度
PASS_SPEED = 2.0
PRIZE_MIN = 0.25


@dataclass
class MissionCmd:
    v_t: float
    steer: float
    throttle: float
    brake: float
    e_lat_target: float
    state: str
    reason: list[str] = field(default_factory=list)


class Mission:
    def __init__(self, region, band=None):
        self.region = region
        self.band = band
        self.path = RefPath(build_reference(*region))
        self.perimeter = self.path.n * self.path.ds
        self.est = ReadEstimator()
        self.box = VoteBox()
        self.state = "follow"
        self.e_target = 0.0
        self.target_key: int | None = None
        self.s_along = 0.0
        self.i_prev: int | None = None
        self.pass_start_s: float | None = None
        self.state_t = 0.0
        self.steer_prev = 0.0
        self.thr_prev = 0.0

    # ---------- 算法层：投影与跟踪 ----------
    def _track(self, x, y, yaw):
        i, tx, ty, dist, e_lat = self.path.project(x, y)
        e_head = wrap180(yaw_of(tx, ty) - yaw)
        kappa = self.path.curvature_ahead(i)
        if self.i_prev is not None:
            self.s_along += ((i - self.i_prev) % self.path.n) * self.path.ds
        self.i_prev = i
        return i, e_lat, e_head, kappa

    def _prize(self, car: CarObs, ego) -> tuple[float, float]:
        d = math.hypot(car.x - ego[0], car.y - ego[1])
        return self.est.p_read(d) * (2.0 * car.p_v - 1.0), d

    def _pick_target(self, cars, ego, x, y, yaw):
        """选一个"前方且值得读"的候选，并给出它在我左侧还是右侧。"""
        best = None
        for c in cars:
            if c.label is not None or c.p_v <= 0.5:
                continue
            pz, d = self._prize(c, ego)
            if pz < PRIZE_MIN or d > 30.0:
                continue
            ahead = (c.x - x) * math.cos(math.radians(yaw)) + \
                    (c.y - y) * (-math.sin(math.radians(yaw)))
            if ahead < 6.0 or ahead > TRIGGER_AHEAD_M:
                continue
            side = (c.x - x) * (-(-math.sin(math.radians(yaw)))) + \
                   (c.y - y) * math.cos(math.radians(yaw))
            if best is None or pz > best[0]:
                best = (pz, c, side)
        return best

    # ---------- 决策层：五状态 ----------
    def tick(self, x, y, yaw, v, img=None, dets=None, t=0.0, dt=0.033):
        self.state_t += dt
        cars = list(self.box.data.values())
        i, e_lat, e_head, kappa = self._track(x, y, yaw)
        cor = Corridor.analyse(img)
        reason = [f"{self.state}"]

        # —— select：只在 follow 里挑目标 ——
        if self.state == "follow":
            ego = (x, y)
            best = self._pick_target(cars, ego, x, y, yaw)
            if best is not None:
                _pz, car, side = best
                self.target_key = car.key
                sign = -1.0 if side > 0 else 1.0        # 车在我左侧 → 设定值正
                self.e_target = sign * OFFSET_MAX
                self.state, self.state_t = "offset_out", 0.0
                reason.append(f"lock target#{car.key} p_v={car.p_v:.2f}")

        # —— 设定值斜坡 ——
        goal = self.e_target if self.state in ("offset_out", "pass") else 0.0
        step = SETPOINT_RATE * dt
        if abs(goal - self.e_target) > step and self.state in ("follow", "offset_in"):
            self.e_target += step * (1.0 if goal > self.e_target else -1.0)
        elif self.state in ("follow", "offset_in"):
            self.e_target = goal

        if self.state == "offset_out":
            if abs(e_lat - self.e_target) < SETPOINT_EPS:
                self.state, self.state_t = "pass", 0.0
                self.pass_start_s = self.s_along
            elif self.state_t > 4.0:
                self.state, self.state_t = "offset_in", 0.0
                reason.append("offset timeout → back")
        elif self.state == "pass":
            if self.pass_start_s is None:
                self.pass_start_s = self.s_along
            if self.s_along - self.pass_start_s > PASS_LENGTH_M:
                self.state, self.state_t = "offset_in", 0.0
                reason.append("pass done → back")
        elif self.state == "offset_in":
            if abs(e_lat) < 0.30 and abs(self.e_target) < 1e-3:
                self.state, self.state_t = "follow", 0.0
                self.target_key = None

        # —— L2 节奏：曲率限速（一切速度上限的起点）——
        r_loc = 1e3 if kappa < 1e-3 else 1.0 / kappa
        v_t = min(V_STRAIGHT, V_CAP, max(V_MIN, math.sqrt(max(0.6 * r_loc, 0.0))))
        if self.state in ("pass", "offset_out", "offset_in"):
            v_t = min(v_t, PASS_SPEED)

        # —— L0 安全：走廊否决（不倒车）——
        if cor.gap_bins < 12:
            v_t = min(v_t, 1.4)
            reason.append(f"corridor gap={cor.gap_bins} v≤1.4")
            if self.state in ("offset_out", "pass"):
                self.state, self.state_t = "offset_in", 0.0
                reason.append("blocked → back")
        if cor.gap_bins < 7:
            v_t = min(v_t, 0.8)

        # —— L1 贴合：把车压到 e_lat_target 上 ——
        e_lat_err = e_lat - self.e_target
        steer_ff = math.atan(WHEELBASE * kappa) / MAX_STEER
        steer_raw = steer_ff + K_HEAD * e_head + K_CROSS * e_lat_err
        steer_cmd = max(self.steer_prev - STEER_SLEW,
                        min(self.steer_prev + STEER_SLEW, steer_raw))
        steer = STEER_LP * self.steer_prev + (1.0 - STEER_LP) * steer_cmd
        steer = max(-0.95, min(0.95, steer))
        self.steer_prev = steer

        # —— 纵向：加速度空间 + 起步咬合 ——
        a_cmd = max(-A_ACC, min(A_ACC, 1.2 * (v_t - v)))
        thr = min(THR_MAX, a_cmd / 8.0) if a_cmd > 0 else 0.0
        brk = min(BRK_MAX, -a_cmd / 100.0 * 4.0) if a_cmd < 0 else 0.0
        if v < 0.4 and v_t > 0.8:
            thr = max(thr, 0.15)
        thr = max(0.0, min(self.thr_prev + 0.05, thr))
        self.thr_prev = thr

        # —— 读号（只在 pass 里做，每帧至多一次）——
        if self.state == "pass" and img is not None and dets:
            for b in dets:
                u = b["x"] + b["w"] / 2.0
                d, s = backproject(u, b["y"] + b["h"], type("V", (), {"hrow": 175, "K": 2200.0, "fov_h_deg": 70.0}))
                if d <= 0.5:
                    continue
                wx, wy = to_world(x, y, yaw, d, s)
                pv = p_violation(wx, wy, 4.4, 1.8, 0.0, self.region, self.band)
                key = int(round(wx)) * 1000 + int(round(wy))
                self.box.observe(key, wx, wy, 0.0, max(b["w"], b["h"]), pv)
                try:
                    from vla2.vision import read_label
                    lab, sc, _raw = read_label(img, roi=(b["x"], b["y"], b["w"], b["h"]))
                except Exception:
                    lab, sc = None, -1.0
                self.est.add(d, bool(lab and sc >= 0.60))
                if lab and sc >= 0.60:
                    self.box.add_read(key, lab, sc)
                    self.state, self.state_t = "offset_in", 0.0
                    reason.append(f"read {lab} {sc:.2f} @{d:.1f}m")
                    break

        return MissionCmd(v_t, steer, thr, brk, self.e_target, self.state, reason), dict(
            i=i, e_lat=e_lat, e_head=e_head, kappa=kappa, cor=cor,
            s_along=self.s_along)

    # ---------- 交卷判断 ----------
    def should_submit(self, t: float) -> tuple[bool, str]:
        pending = [c for c in self.box.data.values() if c.p_v > 0.5 and c.label is None]
        remain = TASK_SECONDS - t
        if not pending:
            return True, "信息饱和"
        if remain < SUBMIT_MARGIN_S + 5.0:
            return True, f"预算耗尽 剩{remain:.1f}s"
        if t > 0.90 * TASK_SECONDS:
            return True, "硬底线"
        return False, ""

    def report(self) -> list[str]:
        return self.box.violating()
