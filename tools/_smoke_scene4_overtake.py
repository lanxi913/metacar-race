# -*- coding: utf-8 -*-
"""元素 6 超车策略的离线自检（不需要仿真引擎）。

自造一条 300 m 直线 + 两辆静止车（x=60 / x=120，都在右侧车道 lat=0），
逐帧喂 8 m/s，检查：

1. 进元素后横向偏移爬到 `OVERTAKE_LANE_OFF_M`；
2. 两辆车都落到车后 → `passes == 2`；
3. 超完回中线，最后一段（剩 ≤ CHECKPOINT_M）横向必须 == 0；
4. 全程方向盘不超 ±1、偏移不超上限。

跑法：`.venv\\Scripts\\python.exe tools\\_smoke_scene4_overtake.py`
"""
from __future__ import annotations

import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from metacar_race.scene4 import config as C                      # noqa: E402
from metacar_race.scene4.overtake import TwoCarOvertakeStrategy  # noqa: E402
from metacar_race.scene4.strategies import (Command, ElementContext,  # noqa: E402
                                            Pt, kind_of)

FAILS: list[str] = []


def check(ok: bool, what: str) -> None:
    if not ok:
        FAILS.append(what)
    print(f"  [{'ok' if ok else 'FAIL'}] {what}")


def make_route(n: int = 300, step: float = 1.0):
    return [Pt(i * step, 0.0) for i in range(n)]


def run(route, cars_at=(60.0, 120.0), v: float = 8.0, dt: float = 1.0 / 30.0):
    s_end = (len(route) - 1) * (route[1].x - route[0].x)
    ctx0 = ElementContext(x=0.0, y=0.0, yaw=0.0, speed=v, dt=dt, sim_t=0.0,
                          s_now=0.0, s_start=0.0, s_end=s_end, vmax=9.0)
    strat = TwoCarOvertakeStrategy(ctx0, route, log=None)
    trace = []
    s = 0.0
    t = 0.0
    offs = []
    passes_at = None
    first_full = None
    back_to_zero = None
    idx = 0
    for _ in range(4000):
        if s > s_end + 1.0:
            break
        # 车沿 +x 直行，y 跟随偏移量
        off_now = strat.off
        obs = []
        for k, xc in enumerate(cars_at):
            d = xc - s
            if -80.0 <= d <= 200.0:
                obs.append({"id": 100 + k, "type": 6, "x": xc, "y": 0.0,
                            "v": 0.0, "len": 4.5, "wid": 1.8, "ori": 0.0,
                            "dist": round(d, 2), "lat": 0.0})
        ctx = ElementContext(x=s, y=off_now, yaw=0.0, speed=v, dt=dt, sim_t=t,
                             s_now=s, s_start=0.0, s_end=s_end, vmax=9.0,
                             obstacles=obs)
        cmd = strat.step(ctx)
        assert isinstance(cmd, Command), "step() 必须返回 Command"
        assert cmd.throttle is None and cmd.brake is None, "纵向必须交回基线"
        trace.append((s, strat.off, strat.passes, cmd.steer, strat.phase))
        offs.append(abs(strat.off))
        if first_full is None and strat.off >= C.OVERTAKE_LANE_OFF_M - 1e-6:
            first_full = s
        if passes_at is None and strat.passes >= C.OVERTAKE_TARGET_PASSES:
            passes_at = s
        if passes_at is not None and back_to_zero is None and strat.off <= 1e-6:
            back_to_zero = s
        s += v * dt
        t += dt
        idx += 1
    return trace, dict(s_end=s_end, first_full=first_full, passes_at=passes_at,
                       back_to_zero=back_to_zero)


def main() -> int:
    print("元素 6 超车策略离线自检")
    print(f"  kind_of('复杂路况超车') = {kind_of('复杂路况超车')!r}")
    check(kind_of("复杂路况超车") == "overtake", "元素名映射到 overtake 策略")

    route = make_route()
    trace, info = run(route)
    check(info["first_full"] is not None, "横向偏移爬到整条车道")
    check(info["first_full"] is not None
          and info["first_full"] <= C.OVERTAKE_RAMP_M + 3.0,
          f"横移在 {C.OVERTAKE_RAMP_M:.0f} m 内完成（实际 {info['first_full']:.1f} m）")
    check(info["passes_at"] is not None, "两辆车都被记成已超")
    check(info["passes_at"] is not None and info["passes_at"] >= 120.0,
          f"第二辆超完才回中线（s={info['passes_at']}）")
    check(info["back_to_zero"] is not None,
          f"回中线完成（s={info['back_to_zero']}）")
    check(info["back_to_zero"] is not None
          and info["back_to_zero"] <= info["s_end"] - C.OVERTAKE_CHECKPOINT_M,
          "回中线早于终点前 12 m")
    tail = [o for (s, o, _p, _st, _ph) in trace if s >= info["s_end"] - C.OVERTAKE_CHECKPOINT_M]
    check(len(tail) > 0 and max(abs(x) for x in tail) <= 1e-6,
          "终点前 12 m 横向严格为 0（穿过检查点）")
    check(max(abs(st) for (_s, _o, _p, st, _ph) in trace) <= 1.0,
          "方向盘始终在 ±1 内")
    check(max(abs(o) for (_s, o, _p, _st, _ph) in trace) <= C.OVERTAKE_LANE_OFF_M + 1e-6,
          "横向偏移不超上限")

    # 反例：终点太近（没有 70 m 余量）时也必须在中线穿过终点
    short = make_route(n=90, step=1.0)
    _tr, info2 = run(short, cars_at=(30.0, 60.0))
    tail2 = [o for (s, o, _p, _st, _ph) in _tr if s >= info2["s_end"] - C.OVERTAKE_CHECKPOINT_M]
    check(len(tail2) > 0 and max(abs(x) for x in tail2) <= 1e-6,
          "短元素（90 m）也在终点前锁回中线")

    print()
    if FAILS:
        print(f"失败 {len(FAILS)} 项：" + "；".join(FAILS))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
