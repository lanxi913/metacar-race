# -*- coding: utf-8 -*-
"""元素 6「复杂路况超车」：先向左并一次 → 超过右侧两辆车 → 回中线贴着终点开。

口径来自用户 2026-10-25（m04571）：「进入复杂情况超车元素后向左变道一次，
确认超过右侧两个车后，变道开向刚刚找到的真正终点」。

**为什么终点这么关键**：平台推进元素靠的是 end_point 附近的近距离检查点
（elem6-r1 实测：0.48 / 0.57 / 0.65 / 0.69 / 0.79 m 全部触发、1.95 m 触发，
而 cont-r2 在横向还差 2.75 m 时穿过终点就**没有**触发）。元素不推进 →
该行 0 分、后面 7~12 号元素也全部不结算。所以这里的最后 12 m 硬锁中线，
保证穿过 end_point 时横向 ≤ 0.1 m。

**只接管方向盘**：纵向一律交回基线（第二个速度环必定抽搐，环岛 r14 那次
175 s 只走 165 m 已经证明）。速度靠基线按 vmax 走，横移速度由本模块的
余弦/线性斜坡限制在 20 m 行程内，横向加速度不叠加。
"""
from __future__ import annotations

from . import config as C
from .lateral import ShiftRoute
from .strategies import Command, ElementContext, Pt, Strategy

try:                                   # 纯跟踪在 control 层，包一层防御性导入
    from ..control.tracking import PurePursuit

    _PP_OK = True
except Exception as _exc:  # noqa: BLE001
    PurePursuit = None                 # type: ignore[assignment]
    _PP_OK = False
    _PP_ERR = repr(_exc)


class TwoCarOvertakeStrategy(Strategy):
    """CRUISE → SHIFT_LEFT → PASS → BACK → CENTER，全部按里程推进。"""

    key = "overtake"
    label = "左并一次、超两车后回中线"

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        self.log = log
        self.base = [Pt(float(getattr(p, "x", 0.0)), float(getattr(p, "y", 0.0)))
                     for p in route]
        self.shift = ShiftRoute(self.base)
        self.pursuit = None
        if _PP_OK and len(self.base) >= 2:
            self.pursuit = PurePursuit(self.shift.items,
                                       lookahead=C.OVERTAKE_LOOKAHEAD_M)
        self.off = 0.0
        self.passes = 0
        self.counted: set[int] = set()
        self.phase = "shift"
        self._last_note = ""
        self._say(f"[overtake] 切片 {len(self.base)} 点 / "
                  f"{self.span_length():.0f} m，盯 {C.OVERTAKE_TARGET_PASSES} 辆")

    # ---------------- 小工具 ----------------

    def _say(self, msg: str) -> None:
        if self.log:
            self.log(msg)

    def span_length(self) -> float:
        total = 0.0
        for a, b in zip(self.base, self.base[1:]):
            total += ((b.x - a.x) ** 2 + (b.y - a.y) ** 2) ** 0.5
        return total

    def sync(self, x: float, y: float) -> None:
        if self.pursuit is not None:
            self.pursuit.sync(x, y, max_jump_m=1e9)

    def reset(self) -> None:
        self.off = 0.0
        self.passes = 0
        self.counted.clear()
        self.phase = "shift"
        self._last_note = ""
        if self.pursuit is not None:
            self.pursuit.reset()

    # ---------------- 超车计数 ----------------

    def _count_passes(self, ctx: ElementContext) -> None:
        """把"本车道里已经被我们落到车后 PASS_BEHIND 米"的障碍记成已超车辆。"""
        for o in (ctx.obstacles or []):
            try:
                otype = int(o.get("type", -1))
                oid = int(o.get("id", -1))
                dist = float(o.get("dist", 1e9))
                lat = float(o.get("lat", 0.0))
            except Exception:  # noqa: BLE001
                continue
            if otype in C.IGNORE_TYPES or oid in self.counted:
                continue
            if dist > -C.OVERTAKE_PASS_BEHIND_M:
                continue
            # 只数"本车道"被越过的车。elem6-r2 实测：两台环境车稳定在 lat +3.52/+4.03
            # （恰是我们并过去的左道），旧判据 `lat > off - 1.0` 会把它们排除，永远数不到 2 辆。
            if abs(lat) > C.OVERTAKE_COUNT_LAT_M:
                continue
            self.counted.add(oid)
            self.passes += 1
            self._say(f"[overtake] 超过第 {self.passes} 辆"
                      f"（id={oid} type={otype} lat={lat:+.1f}）")

    def _lane_clear(self, ctx: ElementContext) -> bool:
        """原车道（|lat| ≤ COUNT_LAT_M）前方的 RETURN_CLEAR_M 内有没有车。

        elem6-r2 的教训：在 remain=70 m 就回中，正好回到一辆停在原车道的车后面
        （blk=3.9 m、1 s 内 8.5→0 m/s、纵向 −8.5 m/s²）。
        """
        for o in (ctx.obstacles or []):
            try:
                otype = int(o.get("type", -1))
                dist = float(o.get("dist", 1e9))
                lat = float(o.get("lat", 0.0))
            except Exception:  # noqa: BLE001
                continue
            if otype in C.IGNORE_TYPES:
                continue
            if abs(lat) > C.OVERTAKE_COUNT_LAT_M:
                continue
            if 0.0 < dist <= C.OVERTAKE_RETURN_CLEAR_M:
                return False
        return True

    # ---------------- 主步进 ----------------

    def step(self, ctx: ElementContext) -> Command | None:
        if self.pursuit is None:
            return None
        remain = float(ctx.s_end) - float(ctx.s_now)
        self._count_passes(ctx)

        done = self.passes >= C.OVERTAKE_TARGET_PASSES
        locked = remain <= C.OVERTAKE_CHECKPOINT_M
        clear = self._lane_clear(ctx)
        # 回中条件：①终点前 12 m 硬锁；②已超够两辆且原车道净空；
        # ③剩 32 m 且原车道净空。②③都要净空——否则保持左侧继续走（PR#35 200 分局
        # 的回中终点在距终点 28.5 m，比原来的 70 m 晚得多）。
        want_back = locked or (done and clear) or (remain <= C.OVERTAKE_RETURN_M and clear)
        target = 0.0 if want_back else C.OVERTAKE_LANE_OFF_M
        ramp = C.OVERTAKE_RETURN_RAMP_M if target < self.off else C.OVERTAKE_RAMP_M

        if locked:
            self.off = 0.0                       # 终点前 12 m 硬锁中线
        else:
            ds = max(float(ctx.speed), 0.0) * max(float(ctx.dt), 0.0)
            step_max = C.OVERTAKE_LANE_OFF_M * ds / max(ramp, 0.1)
            delta = target - self.off
            self.off += max(-step_max, min(step_max, delta))

        if abs(self.off) <= 0.05 and target <= 0.05:
            self.phase = "center"
        elif self.off >= C.OVERTAKE_LANE_OFF_M - 0.05:
            self.phase = "pass"
        elif target > 0.05:
            self.phase = "shift"
        else:
            self.phase = "back"

        self.pursuit.sync(ctx.x, ctx.y, max_jump_m=1e9)
        self.pursuit.route = self.shift.apply(self.off, self.pursuit.idx)
        steer = float(self.pursuit.steering(ctx.x, ctx.y, ctx.yaw))

        note = (f"overtake {self.phase} off={self.off:+.2f} "
                f"pass={self.passes}/{C.OVERTAKE_TARGET_PASSES} remain={remain:.0f}")
        if note != self._last_note:
            self._say("[elem] " + note)
            self._last_note = note
        return Command(steer=steer, note=note)
