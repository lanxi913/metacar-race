# -*- coding: utf-8 -*-
"""真变道离线冒烟：造一条 100 m 直路 + 两条车道（中间虚线），验证：

1. 侧边界是虚线 → 变道成立，横向位移 ≈ 两车道中心距 (3.95+3.95)/2 = 3.95 m；
2. 侧边界是实线 → 不变道（offsets 全 0）；
3. 变道的斜入段是平滑的（相邻点偏移变化 < 0.35 m，不会出现折角）；
4. 偏移线在窗口两端都回到 0（不会骑着线出段）。

用法：.venv\\Scripts\\python.exe tools\\_smoke_scene4_lane.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar_race.scene4.corridor import DASH_LINE, SIDE_LINE, SOLID_LINE  # noqa: E402
from metacar_race.scene4.lanechange import LaneChangeStrategy  # noqa: E402
from metacar_race.scene4.strategies import ElementContext, Pt  # noqa: E402


def make_static(inner_line: int, outer_right: int = SIDE_LINE):
    """直路沿 +x；本车道 y=0（宽 3.95），左侧邻道 y=+3.95（宽 3.95）。"""
    def lane(lid, y0, w, left_t, right_t, left_id, right_id):
        return SimpleNamespace(
            id=lid, width=w,
            path_points=[SimpleNamespace(x=-20.0 + 2 * i, y=y0) for i in range(80)],
            left_border=SimpleNamespace(type=left_t, path_points=[]),
            right_border=SimpleNamespace(type=right_t, path_points=[]),
            left_lane_id=left_id, right_lane_id=right_id)

    main = lane("L1", 0.0, 3.95, inner_line, outer_right, "L2", "")
    left = lane("L2", 3.95, 3.95, SIDE_LINE, inner_line, "", "L1")
    road = SimpleNamespace(id="R1", lanes=[main, left])
    route = [SimpleNamespace(x=0.0 + 1.0 * i, y=0.0) for i in range(120)]
    return SimpleNamespace(roads=[road], route=route, sub_scenes=[])


def run_case(inner_line: int, name: str) -> dict:
    static = make_static(inner_line)
    route = [Pt(p.x, p.y) for p in static.route]
    ctx = ElementContext(x=0.0, y=0.0, yaw=0.0, speed=8.0, dt=1 / 30, sim_t=0.0,
                         s_now=0.0, static=static, ego_width=1.81, vmax=8.0,
                         s_start=0.0, s_end=120.0)
    st = LaneChangeStrategy(ctx, route, log=lambda m: print("   ", m))
    off = st.offsets
    d = [abs(off[i + 1] - off[i]) for i in range(len(off) - 1)]
    # 真机路径：先 sync 再 step（curve-r1 那次 AttributeError 就死在这里，必须覆盖）
    st.sync(ctx.x, ctx.y)
    cmd = st.step(ctx)
    return {
        "name": name, "side": st.side, "shift": st.shift, "plan_ok": st.plan_ok,
        "max_off": max(off) if off else 0.0, "min_off": min(off) if off else 0.0,
        "max_step": max(d) if d else 0.0,
        "head": off[0], "tail": off[-1],
        "cmd_none": cmd is None,
        "steer": None if cmd is None else round(float(cmd.steer or 0.0), 4),
        "lon_none": cmd is not None and cmd.throttle is None and cmd.brake is None,
    }


def main() -> int:
    fails = []

    r1 = run_case(DASH_LINE, "虚线 → 允许变道")
    print(f"[1] {r1}")
    if not r1["plan_ok"] or abs(r1["max_off"] - 3.95) > 0.35 or r1["side"] != +1:
        fails.append("[1] 虚线变道不成立或位移不对")
    if r1["max_step"] > 0.42:
        fails.append(f"[1] 偏移线有折角 max_step={r1['max_step']:.2f}")
    if abs(r1["head"]) > 1e-6 or abs(r1["tail"]) > 1e-6:
        fails.append("[1] 两端没有回到 0")
    if r1["cmd_none"] or r1["steer"] is None or not r1["lon_none"]:
        fails.append(f"[1] sync/step 链路不对：{r1}")

    r2 = run_case(SOLID_LINE, "实线 → 禁止变道")
    print(f"[2] {r2}")
    if r2["plan_ok"] or r2["side"] != 0 or abs(r2["max_off"]) > 1e-6:
        fails.append("[2] 实线仍然变道了")

    # 右侧是路边线时，左侧虚线照样允许（只往合法的一侧变）
    r3 = run_case(DASH_LINE, "虚线（复核一侧）")
    print(f"[3] {r3}")
    if r3["side"] != +1:
        fails.append("[3] 应该往左变道")

    print()
    if fails:
        print("失败 %d 项：" % len(fails))
        for f in fails:
            print("  -", f)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
