"""离线冒烟：不连场景，验证新框架的装配是否自洽。

跑法（仓库根）：

    .venv\\Scripts\\python.exe tools\\_smoke_scene4_framework.py

它做四件事：
1. 假 `static`（12 个元素、一条 4.7 km 的直线路线）→ 构造 `ElementRouter`，
   检查起步段/元素段切分与策略路由；
2. 沿里程扫一遍 `router.step()`，确认直行段接管油门 0.55、环岛段只给转向；
3. 瞬移复位：`router.reset()` 后下一帧能重建策略；
4. 纯函数自检：`limit_command` 限幅、`EndPark` 滑行驻车、`LightGate` 绿灯 ETA。
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from metacar_race.scene4 import config as C
from metacar_race.scene4.lateral import Pt
from metacar_race.scene4.longitudinal import limit_command
from metacar_race.scene4.router import ElementRouter
from metacar_race.scene4.safety import EndPark, LightGate
from metacar_race.scene4.strategies import ElementContext
from metacar_race.scene4.track import RouteTrack

FAILS: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  OK   " if ok else "  FAIL ") + name + (("  " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def fake_static(total_m: float = 4718.5):
    spans = [(8.2, 59.2, "车辆直行"), (60.1, 1083.0, "连续转弯"),
             (1084.0, 1582.6, "环岛驶入、驶出"), (1583.7, 1661.8, "车辆跟随"),
             (1662.8, 1985.3, "十字路口通行"), (1986.3, 2312.5, "复杂路况超车"),
             (2312.5, 2995.5, "匝道行驶"), (2996.5, 3042.9, "车辆紧急停止"),
             (3067.8, 4404.5, "辅路驶入、驶出"), (4405.5, 4422.1, "车辆变道"),
             (4423.1, 4644.3, "车辆切出"), (4645.4, 4718.5, "平行车道泊车")]
    route = [Pt(float(s), 0.0) for s in range(0, int(total_m) + 1, 2)]
    subs = [SimpleNamespace(name=nm, start_point=Pt(a, 0.0), end_point=Pt(b, 0.0))
            for a, b, nm in spans]
    return SimpleNamespace(route=route, roads=[], sub_scenes=subs)


def ctx_at(track: RouteTrack, s: float, speed: float = 6.0) -> ElementContext:
    i = track.sync(s, 0.0)
    return ElementContext(x=s, y=0.0, yaw=0.0, speed=speed, dt=1 / 30, sim_t=s / 6.0,
                          s_now=s, ego_length=4.7, ego_width=1.9, vmax=8.0)


def main() -> int:
    static = fake_static()
    route = static.route
    track = RouteTrack(route)
    print(f"[1] 路线 {track.n} 点 / {track.length:.1f} m")

    # 默认只接管「起步直行 / 环岛」，其余元素走基线；这里显式传 only=(...) 复现默认行为。
    router = ElementRouter(static, route, track, only=("起步直行", "环岛"), log=lambda m: None)
    names = router.names()
    check("起步段被合成在 s=0", names[0] == C.START_SPAN_NAME, names[0])
    check("元素数量 = 13（起步+12）", len(names) == 13, str(len(names)))

    print("[2] 逐段接管检查")
    cases = [(2.0, "straight", None, True),      # 起步段：纵向规划 + 横向接管
             (30.0, "", None, False),            # 元素1 车辆直行：不在默认 only 里，走基线
             (500.0, "", None, False),           # 元素2 连续转弯：默认不接管（见 [2b]）
             (1300.0, "ring", None, True),       # 元素3 环岛：只横向
             (1700.0, "", None, False)]          # 元素4 跟随：不接管
    for s, kind, thr, owned in cases:
        router.reset() if kind != "ring" else None
        cmd = router.step(ctx_at(track, s))
        if not owned:
            check(f"s={s:7.1f} 走基线", cmd is None, repr(cmd))
            continue
        ok = cmd is not None
        detail = "None" if cmd is None else f"thr={cmd.throttle} steer={cmd.steer}"
        check(f"s={s:7.1f} {kind} 接管", ok, detail)
        if ok and thr is not None:
            check(f"s={s:7.1f} 定油门 {thr}", abs(float(cmd.throttle) - thr) < 1e-6,
                  str(cmd.throttle))
        if ok and kind == "ring":
            check(f"s={s:7.1f} 环岛不接管纵向",
                  cmd.throttle is None and cmd.brake is None, detail)

    print("[2c] 起步油门从 0 渐加（滑行规划）")
    router.reset()
    seq = []
    for _ in range(90):                       # 90 帧 ≈ 3 s
        c = router.step(ctx_at(track, 1.0))
        seq.append(None if c is None else c.throttle)
    check("起步第一帧油门 ≈0", seq[0] is not None and float(seq[0]) <= 0.02,
          "None" if seq[0] is None else f"{float(seq[0]):.3f}")
    check("起步 3 s 后油门爬到 0.55 上限",
          seq[-1] is not None and abs(float(seq[-1]) - 0.55) < 0.06,
          "None" if seq[-1] is None else f"{float(seq[-1]):.3f}")
    check("爬升全程单调不减",
          all(a is not None and b is not None and float(b) >= float(a) - 1e-9
              for a, b in zip(seq, seq[1:])), "")

    print("[2b] 放开 only 后由 curve 接管连续转弯（只横向）")
    free = ElementRouter(static, route, track, only=None, log=lambda m: None)
    cmd = free.step(ctx_at(track, 500.0))
    check("only=None 时 s=500 被 curve 接管", cmd is not None,
          "None" if cmd is None else str(free.current.name))
    if cmd is not None:
        check("curve 只接管横向，纵向留空",
              cmd.throttle is None and cmd.brake is None and cmd.steer is not None,
              f"steer={cmd.steer} thr={cmd.throttle}")

    print("[3] 瞬移复位")
    router.step(ctx_at(track, 1300.0))
    router.reset()
    cmd = router.step(ctx_at(track, 1300.0))
    check("reset 后重建", cmd is not None and router.current is not None,
          "" if cmd is None else str(router.current.name))

    print("[4] 纯函数自检")
    # 注意：a_limit=4.5 时，A=8 折算下来油门上限就是 4.5/8=0.5625，比 thr_cap=0.62 更紧。
    # 这是**设计如此**：先夹油门上限，再夹指令加速度预算，取更严的那个。
    thr, brk, a = limit_command(1.0, 0.0, None, 0.62, 0.0, 4.5, 0.0, enabled=True)
    check("指令限幅：油门 1.0 → 4.5/8=0.5625", abs(thr - 0.5625) < 1e-6, f"thr={thr} a_cmd={a:.2f}")
    thr, brk, a = limit_command(0.5, 0.10, None, 1.0, 1.0, 99.0, 0.0, enabled=False)
    check("关掉限幅后原样透传", abs(thr - 0.5) < 1e-6 and abs(brk - 0.10) < 1e-6,
          f"thr={thr} brk={brk}")

    park = EndPark(hold_seconds=8.0, coast_only=True)
    p, t, b, done = park.update(4.8, 0.1)
    check("滑行驻车：还有速度时先不给刹车", (not p) and b == 0.0, f"park={p} brk={b}")
    for _ in range(200):
        p, t, b, done = park.update(0.1, 0.1)
    check("停稳后挂驻车挡", p and b >= 1.0, f"park={p} brk={b} done={done}")

    gate = LightGate(enabled=True, log=None)
    from metacar.models import TrafficLightState
    # 绿剩 1 s、车 8 m/s，而停止线只剩 20 m：到线要 2.5 s，来不及 → 必须按"停在线前"限速。
    # 注意限速是"从当前位置以 LIGHT_DECEL 减速刚好停住"的连续曲线，远处它可能高于当前车速
    # （那时不干预是对的，等靠近了曲线自然会压下来），所以案例要取"已经来不及"的距离。
    light = SimpleNamespace(id="L1", position=Pt(21.0, 0.0),
                            straight_state=TrafficLightState.GREEN,
                            straight_remaining_time=1.0)
    road = SimpleNamespace(stop_line=[Pt(20.0, -3.0), Pt(20.0, 3.0)], lanes=[])
    d = gate.update(groups=[SimpleNamespace(id="G1", traffic_lights=[light])],
                    roads=[road], x=0.0, y=0.0, yaw=0.0, speed=8.0)
    check("绿灯剩 1 s、8 m/s、线前 20 m → 限速到 6.4 以下", d.v_cap is not None and d.v_cap < 6.5,
          f"v_cap={d.v_cap} state={d.state} remain={d.remain}")
    d2 = gate.update(groups=[SimpleNamespace(id="G1", traffic_lights=[
        SimpleNamespace(id="L1", position=Pt(21.0, 0.0),
                        straight_state=TrafficLightState.GREEN,
                        straight_remaining_time=30.0)])],
        roads=[road], x=0.0, y=0.0, yaw=0.0, speed=8.0)
    check("绿灯还剩 30 s → 不干预", d2.v_cap is None, f"v_cap={d2.v_cap}")

    print()
    if FAILS:
        print(f"失败 {len(FAILS)} 项：" + "、".join(FAILS))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
