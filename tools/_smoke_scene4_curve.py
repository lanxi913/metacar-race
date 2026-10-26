# -*- coding: utf-8 -*-
"""离线自检：弯道切内弯（`metacar_race/scene4/curve.py`）。

不连场景、不需要引擎。用一条自造的「直线 + 90° 左弯 + 直线」路线和一份假的车道图，
检查四件事：

1. 能不能认出这段弯、方向对不对（左弯 = 内侧为正）；
2. 切多深是不是被「车道净空 − 半个车宽 − 余量」夹住；
3. 偏移线在弯心真的往内挪了、在弯外确实没动；
4. 直线路段不产生任何偏移（不能因为没弯也在飘）。

用法：
    .venv\\Scripts\\python.exe tools\\_smoke_scene4_curve.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from metacar.models import LineType  # noqa: E402

from metacar_race.scene4.curve import (  # noqa: E402
    CUT_ABORT_HOLD_S, CUT_ABORT_M, CUT_OVERSHOOT_M, CUT_SAFETY_M, CurveCutStrategy,
    MAX_CUT_M,
)
from metacar_race.scene4.lateral import Pt  # noqa: E402
from metacar_race.scene4.strategies import ElementContext, kind_of  # noqa: E402

FAILS = []


def check(name: str, ok: bool, info: str = "") -> None:
    print(("  OK   " if ok else "  FAIL ") + name + (f"  {info}" if info else ""))
    if not ok:
        FAILS.append(name)


def build_route(straight1: float = 40.0, radius: float = 40.0,
                arc_deg: float = 90.0, straight2: float = 40.0) -> list[Pt]:
    """沿 +x 起步，然后左转 90°（半径 radius），再直行。"""
    pts = [Pt(x, 0.0) for x in [i * 1.0 for i in range(int(straight1) + 1)]]
    cx, cy = pts[-1].x, pts[-1].y + radius          # 圆心在左侧
    steps = int(arc_deg / 180.0 * math.pi * radius)
    for k in range(1, steps + 1):
        th = -math.pi / 2 + k / steps * (arc_deg * math.pi / 180.0)
        pts.append(Pt(cx + radius * math.cos(th), cy + radius * math.sin(th)))
    last = pts[-1]
    for k in range(1, int(straight2) + 1):
        pts.append(Pt(last.x, last.y + k))      # 左转 90° 后朝 +y，不能拐回去
    return pts


def fake_static(route: list[Pt], width: float):
    """一份只含一条车道的假静态数据（左虚线、右侧线）。"""
    lane = SimpleNamespace(
        id="0", width=width,
        path_points=[Pt(p.x, p.y) for p in route],
        left_border=SimpleNamespace(type=LineType.DASH_LINE,
                                    path_points=[Pt(p.x, p.y - 0.0) for p in route]),
        right_border=SimpleNamespace(type=LineType.SIDE_LINE,
                                     path_points=[Pt(p.x, p.y) for p in route]),
        left_lane_id="", right_lane_id="",
    )
    road = SimpleNamespace(id="0", lanes=[lane], stop_line=[], driving_type=1,
                           traffic_sign_type=0, begin_pos=None, end_pos=None)
    return SimpleNamespace(roads=[road], route=route, roads_raw=[road])


def lateral_of(path_pt: Pt, route: list[Pt]) -> float:
    """把偏移线的点投影回原路线，返回带符号的横向距离（正 = 左）。"""
    best = (1e9, 0.0)
    for a, b in zip(route, route[1:]):
        vx, vy = b.x - a.x, b.y - a.y
        L = math.hypot(vx, vy) or 1e-9
        ux, uy = vx / L, vy / L
        t = ((path_pt.x - a.x) * ux + (path_pt.y - a.y) * uy)
        t = max(0.0, min(L, t))
        px, py = a.x + ux * t, a.y + uy * t
        d = math.hypot(path_pt.x - px, path_pt.y - py)
        if d < best[0]:
            cross = ux * (path_pt.y - a.y) - uy * (path_pt.x - a.x)
            best = (d, 1.0 if cross > 0 else -1.0)
    return best[0] * best[1]


def main() -> int:
    print("[1] 元素名路由")
    check("连续转弯 → curve", kind_of("连续转弯") == "curve", kind_of("连续转弯"))
    check("车辆直行 → straight", kind_of("车辆直行") == "straight")
    check("车辆跟随 → 基线", kind_of("车辆跟随") == "")

    print("\n[2] 左弯识别与切弯深度（宽车道）")
    route = build_route()
    static = fake_static(route, width=5.20)
    logs: list[str] = []
    ctx = ElementContext(x=0.0, y=0.0, yaw=0.0, speed=6.0, dt=1 / 30, sim_t=0.0,
                         s_now=0.0, static=static, ego_width=1.81, ego_length=4.70,
                         lane_width=5.20, vmax=8.0, s_start=0.0, s_end=999.0)
    strat = CurveCutStrategy(ctx, route, log=logs.append)
    check("识别出 1 段弯", len(strat.regions) == 1, f"regions={strat.regions}")
    if strat.regions:
        r = strat.regions[0]
        # 净空 = 车道半宽 − 半车宽 − 余量；再扣掉实测超调与安全余量，最后夹到 MAX_CUT_M
        expect = min(MAX_CUT_M, 5.20 / 2 - 1.81 / 2 - 0.25 - CUT_OVERSHOOT_M - CUT_SAFETY_M)
        check("内侧方向 = 左", r["sign"] > 0, f"sign={r['sign']}")
        check("切弯深度被净空夹住", abs(r["amp"] - expect) < 0.05,
              f"amp={r['amp']:.3f} 期望≈{expect:.3f}")
        check("半径认得对（≈40 m）", abs(r["r_min"] - 40.0) < 6.0, f"Rmin={r['r_min']:.1f}")

    print("\n[3] 偏移线的形状")
    mid = len(route) // 2
    # 找弯心附近（弧中点）对应的索引
    arc_idx = [i for i, p in enumerate(route) if abs(math.hypot(p.x - 40.0, p.y - 40.0) - 40.0) < 0.5]
    if arc_idx:
        i_mid = arc_idx[len(arc_idx) // 2]
        d_mid = lateral_of(strat.path[i_mid], route)
        check("弯心处向内偏移", d_mid > 0.5, f"横向 {d_mid:+.2f} m")
    d_head = lateral_of(strat.path[0], route)
    d_tail = lateral_of(strat.path[-1], route)
    check("起点不偏移", abs(d_head) < 1e-6, f"{d_head:+.3f}")
    check("终点不偏移", abs(d_tail) < 1e-6, f"{d_tail:+.3f}")

    print("\n[4] 净空不够就不许切（3.95 m 标准车道 / 2.60 m 窄车道）")
    for width in (3.95, 2.60):
        st = fake_static(route, width=width)
        c2 = ElementContext(x=0.0, y=0.0, yaw=0.0, speed=6.0, dt=1 / 30, sim_t=0.0,
                            s_now=0.0, static=st, ego_width=1.81, ego_length=4.70,
                            lane_width=width, vmax=8.0, s_start=0.0, s_end=999.0)
        notes: list[str] = []
        s2 = CurveCutStrategy(c2, route, log=notes.append)
        check(f"{width:.2f} m 车道不切", not s2.regions,
              "净空 " + f"{width / 2 - 1.81 / 2 - 0.25:.2f} m")
        check(f"{width:.2f} m 车道给出不切理由", any("不切" in m for m in notes))

    print("\n[5] 纯直线：不许有任何偏移")
    straight = [Pt(float(i), 0.0) for i in range(120)]
    static3 = fake_static(straight, width=3.95)
    ctx3 = ElementContext(x=0.0, y=0.0, yaw=0.0, speed=6.0, dt=1 / 30, sim_t=0.0,
                          s_now=0.0, static=static3, ego_width=1.81, ego_length=4.70,
                          lane_width=3.95, vmax=8.0, s_start=0.0, s_end=999.0)
    strat3 = CurveCutStrategy(ctx3, straight, log=lambda _m: None)
    dev = max(abs(lateral_of(p, straight)) for p in strat3.path)
    check("直线段偏移恒为 0", dev < 1e-6, f"max={dev:.2e}")
    check("直线段不产生区域", not strat3.regions)

    print("\n[6] 运行期保护：实测横向超计划就取消该段切弯")
    if strat.regions and strat.pursuit is not None:
        r = strat.regions[0]
        i_mid2 = (r["i0"] + r["i1"]) // 2
        strat.pursuit.idx = i_mid2
        p = strat.base[i_mid2]
        # 基线左法向
        tx, ty = strat._tangent(strat.base, i_mid2)
        L = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / L, tx / L
        planned = strat.offsets[i_mid2]
        bad = planned + CUT_ABORT_M + 0.20
        c3 = SimpleNamespace(x=p.x + nx * bad, y=p.y + ny * bad,
                             yaw=0.0, speed=6.0, dt=1 / 30, sim_t=0.0, s_now=0.0)
        strat.step(c3)                                   # 第一次：只记时
        check("首帧不立刻取消", not strat.disabled, f"disabled={strat.disabled}")
        c3.sim_t = CUT_ABORT_HOLD_S + 0.10               # 持续超限
        strat.step(c3)
        check("持续超限后取消该段", bool(strat.disabled), f"disabled={strat.disabled}")
        aft = strat.offsets[min(i_mid2, len(strat.offsets) - 1)]
        check("取消后偏移归零", abs(aft) < 0.30, f"偏移 {aft:+.2f} m（原计划 {planned:+.2f}）")

    if logs:
        print("\n策略日志:")
        for line in logs:
            print("   " + line)

    print("\n" + ("全部通过" if not FAILS else f"失败 {len(FAILS)} 项: {FAILS}"))
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
