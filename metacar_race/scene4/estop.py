# -*- coding: utf-8 -*-
"""元素 8「车辆紧急停止」：端点坐标系停车律（v2，2026-10-25）。

为什么弃用旧口径（`s_end − s_now` 里程差）：
- 平台的 `route_s` 是**整米网格**，`d` 的分辨率就是 1 m，而停车窗口只有 0.2 m；
- 更致命的是旧码在 `d > 0.45` 时返回 `v_cap = sqrt(2·0.25·d)`——这是个**正的限速**
  （d=1.7 时 0.92 m/s），基线会老老实实去追它，StallGuard 再补 0.09~0.27 的油门，
  于是车在离终点 1.7 m 处以 ±0.4 m/s 抖了 9 s、蹭过 2 m，最后满刹停在终点后 0.272 m
  （elem8-r3：`route_s` 3040.94 → 3041.93 → 3042.91，扣 0.72）。

新口径直接照搬 main 已验证的「车辆紧急停止」案例（`metacar_race/control/emergency_stop.py`）：
用平台下发的 `end_point` 世界坐标建终点坐标系，`along = (end − pos)·forward` 是厘米级
真实距离，完全绕开 `route_s` 的网格误差。

控制律（离线试算见 `accel_window_probe/scratch_estop_sim.py`）：
1. **远段**（`along > ESTOP_FINE_ALONG_M`）：`allow = min(进场速度, sqrt(2·a·gap))`，
   快了就松油滑行、慢于限速就把 `v_cap = allow` 交给基线（不自己踩油门）；
2. **精细段**（`along ≤ 8 m`）：`a_need = v² / (2·gap)`，`brk = (a_need − 滑行) / 3.0`
   夹在 `[0, 0.28]`，按 `0.60/s` 爬升——**只用很小的刹车**，绝大多数跑次是 0.03 甚至 0；
3. **收尾**：停早了（`v < 0.25` 还有 `> 0.12 m` 就到）给 0.03 的显式蠕动油门，
   压掉 StallGuard；压到 `along ≤ 0.06` 就停住；
4. **保持**：`thr = 0`、**gear 保持 DRIVE**（不挂 P）、刹车从 0.10 用 1 s 爬到
   `ESTOP_HOLD_BRAKE_MAX(0.55)` 并一直保持（**不用 1.00**：实测 brk=1.00 顶在静止的车上
   会让速度逐帧抖 ±0.18 m/s，元素 8 的 |a| 因此冲到 −5.16，正好丢掉 +10）。
   不挂 P 的理由见下一条的 elem8912-r1 证据：挂 P 之后平台在切元素那一下做 P→D 重新咬合，
   速度会从 0 被跳到 ~6 m/s；保持 D 时（elem89-r1）没有这个跳变。
5. **零刹车起跑**（2026-10-25 第二轮）：平台把元素切到 9 之后**不立刻交棒**，
   先由本策略自己把车从 0 拉起来——`brake` 恒 0、油门按 `0.10/s` 从 0 爬到 0.30、
   目标 `ESTOP_LAUNCH_MS`，一直开到 `s_end + ESTOP_LAUNCH_M(24 m)` 才 `return None`。
   原因：m00443 结算里元素 9 唯一丢分就是交棒后基线给的 2 帧 `brk=0.35`（安全闸）
   加 0→0.56 油门阶跃（|a| 8.49）；本策略的 `Command` 在 driver 里最后覆盖，
   `brake=0.0` 能把安全闸那两帧压掉，斜坡油门也不会阶跃。

元素 1–7 的冻结常量一个都不碰，本文件也只读 `ctx`，不改 driver 的任何通用行为。
"""
from __future__ import annotations

import math

from . import config as C
from .strategies import Command, ElementContext, Strategy

try:
    from metacar import GearMode
except Exception:  # pragma: no cover
    GearMode = None  # type: ignore

try:
    from metacar_race.control.tracking import PurePursuit
except Exception:  # pragma: no cover
    PurePursuit = None  # type: ignore


def _xy(pt):
    """把 end_point（对象或 (x, y)）转成浮点二元组；认不出就 None。"""
    if pt is None:
        return None
    if isinstance(pt, (tuple, list)) and len(pt) >= 2:
        try:
            return float(pt[0]), float(pt[1])
        except (TypeError, ValueError):
            return None
    for ax, ay in (("x", "y"), ("pos_x", "pos_y")):
        x = getattr(pt, ax, None)
        y = getattr(pt, ay, None)
        if x is not None and y is not None:
            try:
                return float(x), float(y)
            except (TypeError, ValueError):
                return None
    return None


def _forward_at_endpoint(route, ex: float, ey: float):
    """在元素折线上找离 end_point 最近的一段，返回该段的方向单位向量。"""
    pts = [(float(p.x), float(p.y)) for p in route]
    if len(pts) < 2:
        return None
    best = None
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-9:
            continue
        t = ((ex - x0) * dx + (ey - y0) * dy) / seg2
        t = max(0.0, min(1.0, t))
        px, py = x0 + t * dx, y0 + t * dy
        dist = math.hypot(ex - px, ey - py)
        if best is None or dist < best[0]:
            best = (dist, dx, dy, math.sqrt(seg2))
    if best is None:
        return None
    _, dx, dy, norm = best
    if norm < 1e-9:
        return None
    return dx / norm, dy / norm


class StopAtEndStrategy(Strategy):
    """元素 8：端点坐标系里滑行到 `end_point` 前 `ESTOP_ALONG_TARGET_M` 停住并保持。"""

    key = "estop"

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        super().__init__(ctx, route, log)
        self.pursuit = None
        if PurePursuit is not None:
            try:
                self.pursuit = PurePursuit(self.route, lookahead=C.ESTOP_LOOKAHEAD_M)
            except Exception as exc:  # pragma: no cover
                self.pursuit = None
                self._log("[estop] 纯跟踪构造失败: %r" % (exc,))
        self.frame = None                 #: (end_x, end_y, fx, fy)
        self.mode = ""                    #: "endpoint" / "mileage"，首帧自检后确定
        self.entry_cap: float | None = None
        self.entry_along: float | None = None
        self.held_s = 0.0
        self.settled = False
        self.released = False
        self.creep_s = 0.0
        self.brk_cmd = 0.0
        self.hold_brake = 0.0
        self.launching = False
        self.launch_s = 0.0
        self.launch_thr = 0.0
        self.platform_at_arrive = None
        self.platform_now = ""
        self.last_meas = -1.0
        end = _xy(getattr(ctx, "end_point", None))
        if end is not None:
            fwd = _forward_at_endpoint(self.route, end[0], end[1])
            if fwd is not None:
                self.frame = (end[0], end[1], fwd[0], fwd[1])
        self.clearance = max(0.10, C.ESTOP_CLEARANCE_RATIO * float(ctx.ego_length or 4.7))
        self._log("[estop] 接管：s_end=%.1f，end_point=%s，目标 along=%.2f m"
                  % (self.s_end, "无" if self.frame is None else
                     "(%.1f, %.1f)" % (self.frame[0], self.frame[1]),
                     C.ESTOP_ALONG_TARGET_M))

    # -- 内部 -------------------------------------------------------------
    def _log(self, msg: str) -> None:
        if self.log is not None:
            try:
                self.log(msg)
            except Exception:
                pass

    def _steer(self, ctx: ElementContext):
        if self.pursuit is None:
            return None
        try:
            steer = float(self.pursuit.steering(ctx.x, ctx.y, ctx.yaw))
        except Exception:
            return None
        if not math.isfinite(steer):
            return 0.0
        return max(-C.MAX_STEER, min(C.MAX_STEER, steer))

    def measure(self, ctx: ElementContext) -> tuple[str, float]:
        """返回 (口径, 到停车点还差多少米)。首帧做一次自检。"""
        along = None
        if self.frame is not None:
            ex, ey, fx, fy = self.frame
            dx = ex - float(ctx.x)
            dy = ey - float(ctx.y)
            along = dx * fx + dy * fy
        if not self.mode:
            mile = self.s_end - float(ctx.s_now)
            if along is None or abs(along - mile) > C.ESTOP_SELFCHECK_M:
                self.mode = "mileage"
                self._log("[estop] end_point 自检失败（along=%s，里程差=%.2f），退回里程口径"
                          % ("无" if along is None else "%.2f" % along, mile))
            else:
                self.mode = "endpoint"
                self._log("[estop] end_point 自检通过：along=%.2f，里程差=%.2f（差 %.2f m）"
                          % (along, mile, along - mile))
        if self.mode == "endpoint" and along is not None:
            return "endpoint", along - C.ESTOP_ALONG_TARGET_M
        return "mileage", (self.s_end - float(ctx.s_now)) - self.clearance

    def allow_speed(self, d: float) -> float:
        """距停车点还有 `d` 米时允许的速度（滑行模型）。"""
        return math.sqrt(2.0 * C.ESTOP_APPROACH_MS2 * max(0.0, d))

    def _settled_now(self, gap: float, along: float, speed: float) -> bool:
        if gap <= C.ESTOP_SETTLED_GAP_M and speed <= C.ESTOP_SETTLED_MS:
            return True
        # 已经压到终点线前 0.06 m：别再往前蹭了
        if along <= C.ESTOP_SETTLED_ALONG_M and speed <= 0.15:
            return True
        # 停在 0.30 m 内、蠕行也推不动（死区）：算停住，避免一直给油门
        if (along <= C.ESTOP_STUCK_ALONG_M and speed <= 0.05
                and self.creep_s >= C.ESTOP_CREEP_LIMIT_S):
            return True
        return False

    def _platform_changed(self, ctx: ElementContext, along: float) -> bool:
        """平台元素名是否已从「车辆紧急停止」切到下一个元素。

        平台名只在**接近终点线**（`ESTOP_NEAR_ALONG_M`）时记一次基线，
        这样元素途中平台数据抖动（空串/重发）不会误判。
        """
        plat = str(getattr(ctx, "platform_element", "") or "")
        if self.platform_at_arrive is None and plat and along <= C.ESTOP_NEAR_ALONG_M:
            self.platform_at_arrive = plat
            self._log("[estop] 记下近线平台元素名：'%.12s'（along=%.2f）" % (plat, along))
        if not plat or self.platform_at_arrive is None or plat == self.platform_at_arrive:
            return False
        self.platform_now = plat
        return True

    def _launch(self, ctx: ElementContext, speed: float, along: float):
        """零刹车起跑：自己把车拉起来，别让基线在交棒瞬间给刹车/油门阶跃。

        只给纵向，**不给转向**（elem8912-r1 的崩溃修复，2026-10-25）：
        本元素切片的最后一个点在 `s_end + 18 m`（`slice_route` 的 tail_pad），
        起跑段却要开到 `s_end + 24 m`。跑到切片之外以后 PurePursuit 的目标点落在车后，
        转向直接打满（实测 f13230 `steer=-1.000`、`lat` +0.80 → +4.59 → +8.11 m），
        车冲出路面在 s=3058.49 停住、整场在 13438 帧结束。
        横向交回基线的 Stanley（它跟的是整条 route，知道 3067.8 处的辅路入口）。
        """
        dt = float(ctx.dt or 0.0)
        self.launch_s += dt
        remain = (self.s_end + C.ESTOP_LAUNCH_M) - float(ctx.s_now)
        if remain <= 0.0 or self.launch_s >= C.ESTOP_LAUNCH_MAX_S:
            self.released = True
            self._log("[estop] 起跑交棒：s=%.1f（s_end+%.0f），v=%.2f，用 %.2f s，平台 '%.12s'"
                      % (float(ctx.s_now), C.ESTOP_LAUNCH_M, speed, self.launch_s,
                         self.platform_now))
            return None
        target = max(1.0, min(C.ESTOP_LAUNCH_MS, self.entry_cap or C.ESTOP_LAUNCH_MS))
        if speed < target:
            self.launch_thr = min(C.ESTOP_LAUNCH_THR_MAX,
                                  self.launch_thr + C.ESTOP_LAUNCH_RAMP_PER_S * dt)
        else:
            self.launch_thr = max(0.0, self.launch_thr - 0.5 * dt)
        gear = GearMode.DRIVE if GearMode is not None else None
        return Command(steer=None, throttle=round(self.launch_thr, 3), brake=0.0, gear=gear,
                       note="estop launch v=%.2f thr=%.3f 余 %.1f m"
                            % (speed, self.launch_thr, remain))

    # -- 基类钩子 ---------------------------------------------------------
    def sync(self, x: float, y: float) -> None:
        if self.pursuit is not None:
            try:
                # 与 ramp.py 同一个坑：max_jump_m 传大值会让 sync 直接返回，索引不动。
                self.pursuit.sync(x, y)
            except Exception:
                pass

    def reset(self) -> None:
        self.entry_cap = None
        self.entry_along = None
        self.held_s = 0.0
        self.settled = False
        self.released = False
        self.creep_s = 0.0
        self.brk_cmd = 0.0
        self.hold_brake = 0.0
        self.platform_at_arrive = None
        self.platform_now = ""
        if self.pursuit is not None:
            try:
                self.pursuit.reset()
            except Exception:
                pass

    def step(self, ctx: ElementContext) -> Command | None:
        if self.released:
            return None
        dt = float(ctx.dt or 0.0)
        steer = self._steer(ctx)
        speed = abs(float(ctx.speed or 0.0))
        mode, gap = self.measure(ctx)
        along = gap + (C.ESTOP_ALONG_TARGET_M if mode == "endpoint" else self.clearance)

        if self.entry_cap is None:
            self.entry_cap = max(0.50, speed)
            self.entry_along = along

        # 0) 保持太久、平台名始终没变（单场景等）→ 兜底交棒
        if self.held_s >= C.ESTOP_RELEASE_MAX_S and not self.launching:
            self.released = True
            self._log("[estop] 兜底交棒：保持 %.2f s，平台元素仍为 '%.12s'"
                      % (self.held_s, str(getattr(ctx, "platform_element", "") or "")))
            return None

        # 1) 到点 → 保持（gear 保持 D、刹车只爬到 0.55）/ 平台切元素后零刹车起跑
        if self.settled or self._settled_now(gap, along, speed):
            self.settled = True
            self.held_s += dt
            if not self.launching and self._platform_changed(ctx, along):
                self.launching = True
                self.launch_s = 0.0
                self._log("[estop] 平台已切到 '%.12s'，保持 %.2f s，开始零刹车起跑（→ s_end+%.0f m）"
                          % (self.platform_now, self.held_s, C.ESTOP_LAUNCH_M))
            if self.launching:
                return self._launch(ctx, speed, along)
            self.hold_brake = min(C.ESTOP_HOLD_BRAKE_MAX,
                                  max(0.10, 0.10 + (C.ESTOP_HOLD_BRAKE_MAX - 0.10)
                                      * min(1.0, self.held_s / max(0.1, C.ESTOP_HOLD_RAMP_S))))
            # 保持段**不挂 P**（elem8912-r1 实测）：挂 P 之后平台在切元素那一下做 P→D
            # 重新咬合，速度从 0 在 0.7 s 内被跳到 5.96 m/s（f13092–f13113，当时我们
            # 的油门只有 0.04~0.10），与「换挡帧平台跳速度」同类，够丢一个元素的 +10。
            # elem89-r1（保持段 gear=DRIVE + 刹车 1.0）切元素时没有这个跳变。
            # 车本来就靠 0.55 的刹车停住，挂 D 不影响元素 8 的停车位置分（已 110）。
            gear = GearMode.DRIVE if GearMode is not None else None
            return Command(steer=steer, throttle=0.0, brake=round(self.hold_brake, 3),
                           gear=gear,
                           note="estop hold %.2fs along=%.2f（%s）"
                                % (self.held_s, along, mode))

        # 2) 精细段：a_need 反算刹车（上限 0.28，按 0.60/s 爬升）
        if along <= C.ESTOP_FINE_ALONG_M:
            a_need = speed * speed / (2.0 * max(gap, 0.05))
            want = (a_need - C.ESTOP_COAST_MS2) / C.ESTOP_BRAKE_GAIN_MS2
            want = max(0.0, min(C.ESTOP_FINE_BRAKE_MAX, want))
            if want > self.brk_cmd:
                self.brk_cmd = min(want, self.brk_cmd + C.ESTOP_BRAKE_RAMP_PER_S * dt)
            else:
                self.brk_cmd = max(want, self.brk_cmd - 3.0 * C.ESTOP_BRAKE_RAMP_PER_S * dt)
            if (speed < C.ESTOP_CREEP_TARGET_MS and gap > C.ESTOP_SETTLED_GAP_M
                    and self.creep_s < C.ESTOP_CREEP_LIMIT_S):
                self.creep_s += dt
                # 0.03 可能落在油门死区里：在 6 s 内线性加到 0.08
                thr = C.ESTOP_CREEP_THROTTLE + (C.ESTOP_CREEP_MAX_THROTTLE
                                                - C.ESTOP_CREEP_THROTTLE) * min(
                                    1.0, self.creep_s / max(0.1, C.ESTOP_CREEP_LIMIT_S))
                return Command(steer=steer, throttle=round(thr, 3), brake=0.0,
                               note="estop creep v=%.2f along=%.2f（%.1fs，thr=%.3f）"
                                    % (speed, along, self.creep_s, thr))
            return Command(steer=steer, throttle=0.0, brake=round(self.brk_cmd, 3),
                           note="estop fine v=%.2f along=%.2f brk=%.2f"
                                % (speed, along, self.brk_cmd))

        # 3) 远段：只给限速 / 松油滑行，油门归基线
        allow = min(self.entry_cap, self.allow_speed(gap))
        if speed > allow + C.COAST_MARGIN_MS:
            return Command(steer=steer, coast=True,
                           note="estop coast v=%.2f allow=%.2f along=%.1f"
                                % (speed, allow, along))
        stop_dist = speed * speed / (2.0 * max(0.5, C.ESTOP_COAST_MS2
                                               + C.ESTOP_BRAKE_GAIN_MS2 * C.ESTOP_FINE_BRAKE_MAX))
        if gap <= stop_dist * 1.05 + 1.0 and speed > 0.30:
            want = min(C.ESTOP_FINE_BRAKE_MAX, max(self.brk_cmd, 0.05))
            self.brk_cmd = min(want, self.brk_cmd + C.ESTOP_BRAKE_RAMP_PER_S * dt)
            return Command(steer=steer, throttle=0.0, brake=round(self.brk_cmd, 3),
                           note="estop guard v=%.2f along=%.1f brk=%.2f"
                                % (speed, along, self.brk_cmd))
        self.brk_cmd = max(0.0, self.brk_cmd - 3.0 * C.ESTOP_BRAKE_RAMP_PER_S * dt)
        return Command(steer=steer, v_cap=allow,
                       note="estop approach v=%.2f allow=%.2f along=%.1f"
                            % (speed, allow, along))


__all__ = ["StopAtEndStrategy"]
