"""连续场景 4（lianxu4）的驱动框架。

分层的唯一目的是**把"为什么这么开"和"怎么把控制量发下去"分开**，这样每个元素的
行为都能单独解释、单独回滚——2026-10-25 那一轮"13 个模块一起改、一起跑"的代价
是四个问题交错出现，排查花了一整天。

    config        集中参数（唯一允许出现魔数的地方）
    track         路线几何：累计里程、投影、停止线/信号灯几何、车道宽
    obstacles     障碍投影、起步破挡、卡死/瞬移检测
    lateral       横向：Stanley、并线状态机、横移路线
    longitudinal  纵向：收油治理、指令限幅、上升限幅
    safety        信号灯闸门（含绿灯 ETA）、终点驻车
    platform      场景 IO 与产物（run.csv / obs.jsonl / events.jsonl / summary.json）
    strategies    元素策略：**只声明自己接管哪条通道**（None = 沿用基线）
    router        里程选元素 + 名称路由 + 瞬移复位 + 失败降级
    driver        主循环装配（以及唯一的一个速度环）

命令行入口仍然是 `tools/park_cs4.py`，它现在只是 `argv → DriverConfig → Driver.run()`。
旧的整体式实现（1519 行）留在 git 历史 `9bdb2e8:tools/park_cs4.py`，需要对照时直接
`git show`，不要在仓库里再放一份拷贝。
"""

from __future__ import annotations

from .config import DriverConfig
from .driver import Driver, run
from .router import ElementRouter, Span
from .strategies import Command, ElementContext, kind_of

__all__ = [
    "DriverConfig", "Driver", "run", "ElementRouter", "Span",
    "Command", "ElementContext", "kind_of",
]
