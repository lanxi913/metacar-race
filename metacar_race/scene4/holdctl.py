"""强制停车（HoldController）：一次停住、一次起步，中间不许顿挫。

## 为什么要有这个模块

2026-10-25 p4/p5 元素 8（车辆紧急停止）实测的"顿挫"不是仿真抖，而是三件事叠在
一起：

1. **刹车被当成调速手段**：`brk=0.35` 一帧就是 `a_cmd = 8·0 − 100·0.35 = −35`，
   实测 −13~−18 m/s²，平台直接判加速度事件（p4 元素 8 有 114 帧 `|a|>5`）。
2. **到位后还在推**：停在 `blk=5.9` 之后目标车速被算成 1.5→0.3，低速限幅
   `thr=0.28` 又把它推着往前蹭，一路蹭到 4.9 m，最后 0.1 m 位置误差。
3. **纵向环积分顶减速**：PI 攒的积分先顶住减速，目标断崖变 0 时再一拍到底，
   于是出现"刹—窜—刹"的双尖峰。

## 做法

把"停车"做成一个**显式状态机**，只有它一个地方能决定"停"：

    run → (前方有阻挡，按运动学剖面提前收速) decel
        → (进到目标停车点余量内，或已经慢到可以锁) hold
        → (前车开走且稳定一会儿) release（复刻场景起点起步：咬合油门一次到位）
        → run

`hold` 阶段：油门归零、只用 `HOLD_BRAKE=0.05`（a_cmd=−5，正好卡在预算线上）减速，
车速低于 `HOLD_LOCK_MS=0.12` 时挂 P 挡 + 刹车 1.0 锁死——P 挡后速度恒定 −0.00，
保持期零抖动（上下坡驻车场景实测）。`release` 阶段用一次性咬合油门，让离合咬合
那一帧**早发生**，不留到判分区中段。

## 谁可以用

元素 8（紧急停止）这种"必须停"的元素由策略发请求即可，不再自己踩刹车；
阻挡扫描（前车）也走同一条路径。默认 `mode="run"` 时输出全 `None`，
行为与改造前完全一致。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from . import config as C


@dataclass
class HoldCmd:
    """一帧的停车决策。`None` 表示这一项不干涉。"""

    mode: str = "run"          #: run / decel / hold / release
    v_cap: float | None = None  #: 目标车速上限（decel/hold 时给出）
    lock: bool = False          #: True = 这一帧强制停车（P 挡 + 满刹）
    bite: bool = False          #: True = 这一帧给咬合油门（起步那一下）
    d_free: float | None = None  #: 扣掉目标停车余量后的可用距离（诊断用）


class HoldController:
    """见模块 docstring。一帧一次 `update()`，返回 `HoldCmd`。"""

    def __init__(self) -> None:
        self.mode = "run"
        self.lock_on = False
        self.clear_t = 0.0
        self.bite_t = 0.0
        self.lead_stop_t = 0.0    #: 前车连续静止了多久（测量）
        self.holds = 0            #: 累计锁止次数（诊断）

    # ---- 生命周期 ---------------------------------------------------
    def reset(self) -> None:
        self.mode = "run"
        self.lock_on = False
        self.clear_t = 0.0
        self.bite_t = 0.0
        self.lead_stop_t = 0.0

    @property
    def locked(self) -> bool:
        return self.lock_on

    # ---- 每帧决策 ---------------------------------------------------
    def update(self, blk, stop_gap: float, extra: float,
               speed: float, dt: float, must_stop: bool = False,
               d_true: float | None = None) -> HoldCmd:
        cmd = HoldCmd()
        v_abs = abs(float(speed))
        d_free: float | None = None
        v_lead = 0.0
        if blk is not None:
            # 2026-10-25 p6 实测：`blk["dist"]` 是 route 投影、按 1 m 前缀量化，
            # 拿它判"够近了"必然骗早（p6 元素 8 在 5.87 m 就锁死，误差 0.8 m）。
            # 有实测中心距就用实测值，量化值只做兜底。
            d_meas = float(d_true) if d_true is not None else float(blk["dist"])
            d_free = d_meas - float(stop_gap)
            v_lead = abs(float(blk.get("v") or 0.0))
        cmd.d_free = d_free

        # 前车是不是"真的停住"（测量）：只有停住才允许为了它锁死自己
        if d_free is not None and d_free < C.BLOCK_SLOW_M and v_lead < C.HOLD_LEAD_STOP_MS:
            self.lead_stop_t += dt
        else:
            self.lead_stop_t = 0.0
        lead_stopped = self.lead_stop_t >= C.HOLD_LEAD_STOP_S

        # 1) 已锁：等前车真的走开（距离拉开 + 稳定一段时间，防抖）
        if self.lock_on:
            if d_free is None or d_free > C.HOLD_RELEASE_M:
                self.clear_t += dt
                if self.clear_t >= C.HOLD_RELEASE_S:
                    self.lock_on = False
                    self.mode = "release"
                    self.bite_t = C.LAUNCH_BITE_MS
            else:
                self.clear_t = 0.0

        # 2) 该锁了就锁：按"还差多少才能停在目标点"判（测量驱动），
        #    而不是"离目标点多近"。pred = 当前车速按 BLOCK_DECEL 还能滑多远；
        #    d_free − pred 就是再晚一点刹就会越过的距离，落到目标点上就锁。
        # 2026-10-25 p6 教训：**只有"规则要求必须停"的段才允许锁死**。p6 把"前车静止"
        # 也当成锁死条件，结果在元素 9 入口被一辆永远不动的车锁住 → 元素超时 → 平台
        # 把车瞬移回起点 → 再锁 → 死循环（元素 9 反复重跑 1122 s，全程 1540 s）。
        # 一般的前车只做"跟速 + 软刹"，不接管，撞不上也不会被锁死。
        if (not self.lock_on) and d_free is not None and bool(must_stop):
            at_target = d_free <= float(extra)
            pred = (v_abs * v_abs) / (2.0 * C.BLOCK_DECEL)
            stop_now = (d_free - pred) <= float(extra) + C.HOLD_STOP_TOL
            crawl = v_abs <= C.HOLD_LOCK_MS * 2.5 and d_free <= float(extra) + C.HOLD_LOCK_WINDOW
            if at_target or stop_now or crawl:
                self.lock_on = True
                self.mode = "hold"
                self.clear_t = 0.0
                self.holds += 1

        # 3) 锁着：不再规划速度，交给 driver 强制停车
        if self.lock_on:
            cmd.mode = "hold"
            cmd.lock = True
            cmd.v_cap = 0.0
            return cmd

        # 4) 刚放开：给一次性咬合油门，让离合咬合这一帧早发生
        if self.mode == "release":
            self.bite_t -= dt
            if self.bite_t <= 0.0 or v_abs > C.LAUNCH_BITE_MS * 3.0:
                self.mode = "run"
            else:
                cmd.mode = "release"
                cmd.bite = True
                return cmd

        # 5) 预判减速：运动学剖面，前车自己还在动就把它的速度算进来
        if d_free is not None and d_free < C.BLOCK_SLOW_M:
            v_kin = v_lead + math.sqrt(max(0.0, 2.0 * C.BLOCK_DECEL *
                                           max(d_free - float(extra), 0.0)))
            if d_free <= float(extra):
                v_kin = 0.0
            cmd.v_cap = v_kin
            cmd.mode = "decel"
        return cmd
