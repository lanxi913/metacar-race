# -*- coding: utf-8 -*-
"""元素 9「辅路驶入、驶出」：路线级纯跟踪（2026-10-25 新增）。

参考案例：main 上的 `templates/scenes/zyyxnjydhcg/fudao_run.py` +
`metacar_race/control/route_follower.py`（单场景 443 m / 99 s / 110 分）。
那份实现的三条经验，这里逐条照搬：

1. **跟 `static.route`，不跟最近车道**——辅路那一片 roads 有 6 条、车道 21 条，
   而且跨路段不连续，按车道找会跳线；
2. **不要叠保守减速**——单场景的失败跑次全是"曲率限速 + 巡航限速"叠太多：
   v1 巡航 4.5 跑 106 s、v2 巡航 6.0 还差 2 m、v3 关掉弯道限速才 98.3 s，
   最后 v4 只留"规划减速度 2.0 + 终点判定"才 99 s 拿到 110；
3. **收尾不能硬停**——一帧 4.8→0 就是 5 m/s² 的加速度事件，必须轻刹到 0.05 m/s
   以下再保持；转向限幅 0.8，防止低速时方向盘顶死。

与单场景的差别（连续场景）：
* 元素 9 有 **1336.7 m / 限时 200 s**，平均要 ≥6.68 m/s；一挡墙 4.78 m/s 跑完要 280 s，
  必然超时；
* 元素 8 把车停住后变速箱退回一挡，而 driver 的升挡闸门 `_shifted_ok` 是**全局一次**
  （m03278，元素 1 已经用掉）⇒ 元素 9 只能自己补一次升挡 kick：油门 0.86→0.88，
  贴着一挡墙触发（元素 1 实测 0.86 顶 0.43 s 就升挡）。

纪律：本策略只接管 `steer` 与（kick 期间的）`throttle`，纵向巡航仍交基线；
`v_cap` 给 `vmax` 只是不让基线自己踩到 9 以上。元素 10 在 6 m 之外，提前 `return None`
把控制权还给基线，绝不抢占后面的元素。
"""
from __future__ import annotations

import math

from . import config as C
from .strategies import Command, ElementContext, Strategy

try:
    from metacar_race.control.tracking import PurePursuit
except Exception:  # pragma: no cover
    PurePursuit = None  # type: ignore


class FudaoStrategy(Strategy):
    """元素 9：贴 `static.route` 纯跟踪 + 一次升挡 kick + 终点前 6 m 交棒。"""

    key = "fudao"

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        super().__init__(ctx, route, log)
        self.pursuit = None
        if PurePursuit is not None:
            try:
                self.pursuit = PurePursuit(self.route, lookahead=C.FUDAO_LOOKAHEAD_M)
            except Exception as exc:  # pragma: no cover
                self.pursuit = None
                self._log("[fudao] 纯跟踪构造失败: %r" % (exc,))
        self.vmax = float(getattr(ctx, "vmax", 0.0) or 9.0)
        self.kick_until = -1e9
        self.kick_thr = 0.0
        self.kick_done = False
        self.last_kick = -1e9
        self.handed = False
        self.launch_thr = 0.0
        self.launch_done = False
        self._log("[fudao] 接管：%.1f → %.1f m（%.1f m），vmax=%.2f，kick=%s"
                  % (self.s_start, self.s_end, max(0.0, self.s_end - self.s_start),
                     self.vmax, "on" if C.FUDAO_KICK_ON else "off"))

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

    def _kick_active(self, ctx: ElementContext) -> bool:
        return float(ctx.sim_t) <= self.kick_until

    def _maybe_kick(self, ctx: ElementContext, speed: float, remain: float, dt: float) -> None:
        now = float(ctx.sim_t)
        # 已经跑过一挡墙 → 说明已经在二挡以上，kick 不再需要
        if speed > C.KICK_WALL_MS + 0.30:
            if not self.kick_done:
                self.kick_done = True
                self._log("[fudao] v=%.2f 已过一挡墙，无需 kick" % speed)
            return
        if self._kick_active(ctx):
            return
        if self.kick_done or not C.FUDAO_KICK_ON:
            return
        if remain < C.FUDAO_KICK_MIN_TAIL_M:
            return
        if now - self.last_kick < C.FUDAO_KICK_MIN_GAP_S:
            return
        if abs(speed - C.KICK_WALL_MS) > C.KICK_WALL_TOL_MS:
            return
        self.kick_until = now + C.FUDAO_KICK_MAX_S
        self.last_kick = now
        self.kick_thr = 0.0
        self._log("[fudao] 升挡 kick：v=%.2f，顶油门 %.2f 最多 %.1f s（剩 %.0f m）"
                  % (speed, C.FUDAO_KICK_THROTTLE, C.FUDAO_KICK_MAX_S, remain))

    # -- 基类钩子 ---------------------------------------------------------
    def sync(self, x: float, y: float) -> None:
        if self.pursuit is not None:
            try:
                self.pursuit.sync(x, y)
            except Exception:
                pass

    def reset(self) -> None:
        self.kick_until = -1e9
        self.kick_thr = 0.0
        self.kick_done = False
        self.last_kick = -1e9
        self.handed = False
        self.launch_thr = 0.0
        self.launch_done = False
        if self.pursuit is not None:
            try:
                self.pursuit.reset()
            except Exception:
                pass

    def step(self, ctx: ElementContext) -> Command | None:
        dt = float(ctx.dt or 0.0)
        speed = abs(float(ctx.speed or 0.0))
        remain = self.s_end - float(ctx.s_now)

        # 1) 终点前 6 m 交棒给基线（元素 10 紧接在后面）
        if remain <= C.FUDAO_HANDOVER_M:
            if not self.handed:
                self.handed = True
                self._log("[fudao] 剩 %.1f m，交棒给基线（元素 10 由基线接管）" % remain)
            return None

        steer = self._steer(ctx)

        # 2) 起跑缓升：元素 8 交回来时车只跑到 ~3 m/s（estop 零刹车起跑 24 m），
        #    直接交回基线会出现 0→0.5 的油门阶跃（elem89-r1 的元素 9 因此丢了 +10，
        #    |a| 峰值 8.49）。这里自己把油门按 0.10/s 爬到 0.30，到 7 m/s 再交回。
        if not self.launch_done:
            if speed >= C.FUDAO_LAUNCH_MS:
                self.launch_done = True
                self._log("[fudao] 起跑完成：v=%.2f 已到 %.1f，纵向交回基线"
                          % (speed, C.FUDAO_LAUNCH_MS))
            else:
                self.launch_thr = min(C.FUDAO_LAUNCH_THR_MAX,
                                      self.launch_thr + C.FUDAO_LAUNCH_RAMP_PER_S * dt)
                return Command(steer=steer, throttle=round(self.launch_thr, 3), brake=0.0,
                               note="fudao launch v=%.2f thr=%.2f" % (speed, self.launch_thr))

        # 3) 升挡 kick（只在一挡墙上触发，且只在长元素里）
        self._maybe_kick(ctx, speed, remain, dt)
        if self._kick_active(ctx):
            self.kick_thr = min(C.FUDAO_KICK_THROTTLE,
                                self.kick_thr + C.FUDAO_KICK_RAMP_PER_S * dt)
            if speed >= C.SHIFT_DONE_MS:
                self.kick_done = True
                self.kick_until = -1e9
                self._log("[fudao] 升挡完成：v=%.2f" % speed)
            else:
                return Command(steer=steer, throttle=round(self.kick_thr, 3), brake=0.0,
                               note="fudao kick v=%.2f thr=%.2f" % (speed, self.kick_thr))

        # 4) 常规：只给横向与限速，**纵向交回基线**。
        #    这是 elem12-r1（2026-10-26 11:24）拿 110 的版本；当天下午试过自己接管纵向
        #    （巡航律 8.8 m/s + 0.02/帧油门），元素 9 掉回 100（elem12-r2），已回退。
        cap = min(float(self.vmax or 9.0), 9.0)
        return Command(steer=steer, v_cap=cap,
                       note="fudao v=%.2f 剩 %.0f m" % (speed, remain))


__all__ = ["FudaoStrategy"]
