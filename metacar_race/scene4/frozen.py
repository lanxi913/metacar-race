# -*- coding: utf-8 -*-
"""元素 1–6 的「冻结快照」：把已经拿到满分的部分钉住，后续只许改 7–12。

为什么要这一层（2026-10-25 用户要求）：
连续场景 4 的元素是**串行结算**的，1–6 已经全部拿满（车辆直行 100、连续转弯 110、
环岛 110、跟随 110、十字路口 210、复杂超车 210）。后面再做 7–12 时，任何一次
"顺手调一下参数"都可能把这六个的分数线踩掉，而且很难在跑局日志里看出来。
所以这里把「元素 1–6 的策略映射 + 影响它们的全部常量」固化成一份可 diff 的快照：

- `FROZEN_ELEMENTS`：冻结的元素名（含合成的「起步直行」）；
- `FROZEN_CONSTANTS`：影响 1–6 的常量白名单（只列名字，值在 `snapshot()` 里读）；
- `snapshot()`：当前生效值；`tools/_smoke_scene4_frozen.py` 拿它和
  `docs/scene4-frozen.json` 逐项比对，**任何漂移都算冒烟失败**。

注意：这一层只做「检测」，不改任何控制量——行为与加它之前完全一致。
"""
from __future__ import annotations

from typing import Any

from . import config as C

#: 元素 1–6 + 起步段：这些已经满分，冻结。
FROZEN_ELEMENTS: tuple[str, ...] = (
    C.START_SPAN_NAME,      # 合成段：起步即升挡
    "车辆直行",              # 元素 1：定油门 0.55
    "连续转弯",              # 元素 2：切内弯 1.60 m
    "环岛驶入、驶出",         # 元素 3：只接管方向盘 + 曲率限速
    "车辆跟随",              # 元素 4：走基线
    "十字路口通行",           # 元素 5：灯态队列 + 非绿→绿起步
    "复杂路况超车",           # 元素 6：左并 → 超两车 → 回中线
)

#: 冻结期的策略映射（`strategies.kind_of` 必须与之一致）。
FROZEN_KIND: dict[str, str] = {
    C.START_SPAN_NAME: "straight",
    "车辆直行": "straight",
    "连续转弯": "curve",
    "环岛驶入、驶出": "ring",
    "车辆跟随": "",
    "十字路口通行": "",
    "复杂路况超车": "overtake",
}

#: 影响元素 1–6 的常量白名单（7–12 专用常量不在此列，改它们不会触发告警）。
FROZEN_CONSTANTS: tuple[str, ...] = (
    # 通用纵向/加速度纪律
    "STRAIGHT_THROTTLE", "ACCEL_CMD_A", "ACCEL_CMD_B", "ACCEL_CMD_LIMIT",
    "ACCEL_CMD_SLEW", "THROTTLE_CAP_BONUS", "BRAKE_CAP_BONUS",
    "COAST_TRIM_STEP", "COAST_TRIM_MAX", "COAST_THROTTLE_CUT",
    "COAST_DECEL_MS2", "THROTTLE_RAMP_PER_S", "ELEMENT_END_SPEED_MS",
    "COAST_MARGIN_MS", "COAST_LOOKAHEAD_MIN_M", "BRAKE_ONLY_BELOW_MS",
    "ACCEL_GOV_WIN_S", "ACCEL_GOV_TARGET", "ACCEL_GOV_BRAKE_MAX",
    # 升挡闸门（起步段 + 元素 1）
    "SHIFT_THROTTLE", "SHIFT_DONE_MS", "SHIFT_MAX_S", "SHIFT_MIN_GAP_S",
    "SHIFT_MIN_SPAN_M", "SHIFT_TARGET_MS", "PRE_SHIFT_M",
    # 长元素速度下限（元素 2/3 都吃这条）
    "LONG_SPAN_MIN_MS", "LONG_SPAN_FLOOR_TAIL_M",
    # 环岛（元素 3）
    "RING_LAT_BUDGET_MS2", "RING_V_FLOOR_MS", "RING_STEER_RATE",
    "RING_STEER_LIMIT", "RING_LAT_VMAX_MS", "RING_SPEED_CAP_MS",
    "RING_PROFILE_COAST_MS2",
    # 切内弯（元素 2）
    "R_THRESH", "MAX_CUT_M", "MIN_CUT_M", "LOOKAHEAD_M", "SMOOTH_WIN",
    "CUT_OVERSHOOT_M", "CUT_SAFETY_M", "CUT_ABORT_M", "CUT_ABORT_HOLD_S",
    # 复杂超车（元素 6）
    "OVERTAKE_LANE_OFF_M", "OVERTAKE_RAMP_M", "OVERTAKE_PASS_BEHIND_M",
    "OVERTAKE_TARGET_PASSES", "OVERTAKE_COUNT_LAT_M", "OVERTAKE_RETURN_M",
    "OVERTAKE_RETURN_RAMP_M", "OVERTAKE_RETURN_CLEAR_M",
    "OVERTAKE_CHECKPOINT_M", "OVERTAKE_LOOKAHEAD_M",
    # 信号灯（元素 5）
    "LIGHT_DECIDE_M", "LIGHT_COAST_DECEL_MS2", "LIGHT_STOP_AT_M",
    "LIGHT_STOP_GAP_M", "LIGHT_STOP_TOL_M", "LIGHT_DECEL", "LIGHT_LOOK_M",
    "LIGHT_GROUP_CHAIN_M", "LIGHT_APPROACH_MS", "LIGHT_COMMIT_M",
    "LIGHT_WAIT_BRAKE_M", "LIGHT_WAIT_BRAKE", "LIGHT_WAIT_MAX_S",
    "LIGHT_CREEP_MS", "LIGHT_CREEP_M", "LIGHT_CREEP_THROTTLE",
    # 障碍/跟车（元素 4/5/6 都吃）
    "LANE_HALF_M", "BLOCK_HORIZON_M", "BLOCK_MARGIN_M", "BLOCK_SLOW_M",
    "BLOCK_STOP_M", "FOLLOW_DECEL", "TELEPORT_MIN_M", "IDX_GUARD_M",
)


def snapshot() -> dict[str, Any]:
    """当前生效的冻结值（JSON 可序列化）。"""
    values: dict[str, Any] = {}
    for name in FROZEN_CONSTANTS:
        if hasattr(C, name):
            values[name] = getattr(C, name)
    return {
        "elements": list(FROZEN_ELEMENTS),
        "kind": dict(FROZEN_KIND),
        "constants": values,
    }


def diff_against(expected: dict[str, Any]) -> list[str]:
    """对比快照，返回人类可读的漂移列表（空 = 一致）。"""
    now = snapshot()
    out: list[str] = []
    if list(expected.get("elements", [])) != now["elements"]:
        out.append("元素清单漂移：%r → %r" % (expected.get("elements"), now["elements"]))
    exp_kind = expected.get("kind", {})
    for name, kind in now["kind"].items():
        if exp_kind.get(name, "<missing>") != kind:
            out.append("策略映射漂移 %s：%r → %r" % (name, exp_kind.get(name), kind))
    exp_vals = expected.get("constants", {})
    for name, val in now["constants"].items():
        old = exp_vals.get(name, "<missing>")
        if isinstance(val, float) and isinstance(old, (int, float)):
            if abs(float(val) - float(old)) > 1e-9:
                out.append("常量漂移 %s：%r → %r" % (name, old, val))
        elif old != val:
            out.append("常量漂移 %s：%r → %r" % (name, old, val))
    return out


__all__ = ["FROZEN_ELEMENTS", "FROZEN_KIND", "FROZEN_CONSTANTS", "snapshot", "diff_against"]
