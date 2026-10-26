# -*- coding: utf-8 -*-
"""元素 7「匝道行驶」的策略：只做「弯道限速 + 贴线」，纵向不另起一套闭环。

为什么要单独一个策略（2026-10-25 复盘）：
- 元素 7 是 2312.5~2995.5 m 的 683 m 长弯道路段，之前全程走通用栈（profile 9 m/s
  + 纯跟踪），只跑到过 s=2553.5（elem6-r4 收场），弯道是否偏线从来没有被验证过；
- 通用栈的 Stanley 只在元素名含「环岛」时打开，匝道这种长弯没有任何横向/曲率保护；
- 但纵向绝不能再来第二套闭环（环岛那次抽搐的教训），所以这里只给
  `Command(steer=..., v_cap=...)`，油门/刹车仍由基线速度环输出。

曲率限速复用 `ring.RingProfile`：它对整条静态路线逐点最小二乘拟合曲率，
再按 `sqrt(a_lat·R)` 取上限并做反向滑行回推——和环岛用的是同一套已验证代码。
"""
from __future__ import annotations

import math
from typing import Optional

from . import config as C
from .ring import RingProfile
from .strategies import Command, ElementContext, Strategy

try:  # 复用已验证的纯跟踪
    from metacar_race.control.tracking import PurePursuit
except Exception:  # pragma: no cover
    PurePursuit = None  # type: ignore


class RampStrategy(Strategy):
    """元素 7：贴线 + 曲率限速，纵向交给基线。"""

    key = "ramp"

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        super().__init__(ctx, route, log)
        self.base_route = getattr(getattr(ctx, "static", None), "route", None) or route
        self.vmax = float(getattr(ctx, "vmax", 0.0) or 9.0)
        self.profile: Optional[RingProfile] = None
        self.pursuit = None
        self._build(ctx)

    def _build(self, ctx: ElementContext) -> None:
        if PurePursuit is not None:
            try:
                self.pursuit = PurePursuit(self.route, lookahead=C.RAMP_LOOKAHEAD_M)
            except Exception as exc:  # pragma: no cover
                self.pursuit = None
                self._log("纯跟踪构造失败，只给限速: %r" % (exc,))
        try:
            self.profile = RingProfile(
                self.base_route,
                self.s_start,
                self.s_end,
                a_lat=C.RAMP_LAT_BUDGET_MS2,
                v_max=self.vmax,
                v_floor=C.RAMP_V_FLOOR_MS,
                coast=C.RAMP_COAST_MS2,
            )
            if self.profile.ok:
                self._log(
                    "[ramp] 限速剖面 %d 点 / %.0f m，最低 %.2f m/s，R_min=%.1f m"
                    % (
                        len(self.profile.caps),
                        self.profile.length,
                        self.profile.floor(),
                        self.profile.r_min,
                    )
                )
            else:
                self._log("[ramp] 限速剖面不可用（点太少），只贴线")
                self.profile = None
        except Exception as exc:  # pragma: no cover
            self.profile = None
            self._log("[ramp] 限速剖面构造失败，只贴线: %r" % (exc,))

    def _log(self, msg: str) -> None:
        if self.log is not None:
            try:
                self.log(msg)
            except Exception:
                pass

    def sync(self, x: float, y: float) -> None:
        if self.pursuit is not None:
            try:
                # 注意：必须用默认 max_jump_m=8。传 1e9 会让 sync 直接 return，
                # 索引停在 0，纯跟踪就永远把目标点当成"在车后"打满方向（实测 steer=1.0）。
                self.pursuit.sync(x, y)
            except Exception:
                pass

    def reset(self) -> None:
        if self.pursuit is not None:
            try:
                self.pursuit.reset()
            except Exception:
                pass

    def step(self, ctx: ElementContext) -> Command | None:
        steer = None
        if self.pursuit is not None:
            try:
                steer = float(self.pursuit.steering(ctx.x, ctx.y, ctx.yaw))
                if not math.isfinite(steer):
                    steer = 0.0
                steer = max(-C.MAX_STEER, min(C.MAX_STEER, steer))
            except Exception:
                steer = None

        v_cap = self.vmax
        if self.profile is not None and self.profile.ok:
            try:
                cap = float(self.profile.cap_at(ctx.s_now))
                v_cap = max(C.RAMP_V_FLOOR_MS, min(self.vmax, cap))
            except Exception:
                v_cap = self.vmax
        # 终点前把速度收住，免得冲过 end_point（平台按近距检查点推进，冲过头就不结算）
        remain = self.s_end - ctx.s_now
        if remain <= C.RAMP_END_SLOW_M:
            v_cap = min(v_cap, max(C.RAMP_V_FLOOR_MS * 0.5, math.sqrt(2.0 * 0.6 * max(0.0, remain))))
        return Command(steer=steer, v_cap=v_cap, note="ramp vcap=%.2f" % (v_cap,))


__all__ = ["RampStrategy"]
