"""离线冒烟：元素 7「匝道行驶」、元素 8「车辆紧急停止」、元素 9「辅路驶入、驶出」。

跑法（仓库根）：

    .venv\\Scripts\\python.exe tools\\_smoke_scene4_ramp_estop.py

它做五件事：
1. 假 `static`（12 个元素、一条直路）→ 确认 `kind_of` 把「匝道行驶」「车辆紧急停止」
   「辅路驶入、驶出」分别映射到 `ramp` / `estop` / `fudao`；
2. `ElementRouter` 只接管 7/8 时，s=2500 落到 `RampStrategy`、s=3010 落到
   `StopAtEndStrategy`，其余元素不接管；
3. 匝道：转向在 ±1 内、`v_cap` ≤ vmax 且 ≥ 地板值（终点前收速是唯一例外）；
4. 紧急停止（v2 端点坐标系）：远段只给限速/滑行、精细段刹车 ≤ 0.28 且不阶跃、
   停早了给 0.03 蠕行、到点后油门清零刹车爬坡、平台元素名变了才交棒；
5. 辅路：纯跟踪给转向、不给油门、终点前 6 m 交棒、一挡墙上补一次 kick。
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
from metacar_race.scene4.router import ElementRouter
from metacar_race.scene4.strategies import ElementContext, kind_of
from metacar_race.scene4.track import RouteTrack

FAILS: list[str] = []
SPANS = [(8.2, 59.2, "车辆直行"), (60.1, 1083.0, "连续转弯"),
         (1084.0, 1582.6, "环岛驶入、驶出"), (1583.7, 1661.8, "车辆跟随"),
         (1662.8, 1985.3, "十字路口通行"), (1986.3, 2312.5, "复杂路况超车"),
         (2312.5, 2995.5, "匝道行驶"), (2996.5, 3042.9, "车辆紧急停止"),
         (3067.8, 4404.5, "辅路驶入、驶出"), (4405.5, 4422.1, "车辆变道"),
         (4423.1, 4644.3, "车辆切出"), (4645.4, 4718.5, "平行车道泊车")]


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  OK   " if ok else "  FAIL ") + name + (("  " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def fake_static(total_m: float = 4718.5):
    route = [Pt(float(s), 0.0) for s in range(0, int(total_m) + 1, 1)]
    subs = [SimpleNamespace(name=nm, start_point=Pt(a, 0.0), end_point=Pt(b, 0.0))
            for a, b, nm in SPANS]
    return SimpleNamespace(route=route, roads=[], sub_scenes=subs)


def ctx_at(track: RouteTrack, s: float, speed: float = 6.0, dt: float = 1 / 30,
           end_point=None, plat: str = "", sim_t: float | None = None) -> ElementContext:
    track.sync(s, 0.0)
    return ElementContext(x=s, y=0.0, yaw=0.0, speed=speed, dt=dt,
                          sim_t=(s / 6.0 if sim_t is None else sim_t),
                          s_now=s, ego_length=4.7, ego_width=1.9, vmax=9.0,
                          end_point=end_point, platform_element=plat)


def main() -> int:
    static = fake_static()
    route = static.route
    track = RouteTrack(route)

    print("[1] 元素名映射")
    check("匝道行驶 → ramp", kind_of("匝道行驶") == "ramp", kind_of("匝道行驶"))
    check("车辆紧急停止 → estop", kind_of("车辆紧急停止") == "estop", kind_of("车辆紧急停止"))
    check("辅路驶入、驶出 → fudao", kind_of("辅路驶入、驶出") == "fudao",
          kind_of("辅路驶入、驶出"))
    check("车辆变道仍返回空（未接管）", kind_of("车辆变道") == "", repr(kind_of("车辆变道")))

    print("[2] 路由：只接管 7/8")
    router = ElementRouter(static, route, track, vmax=9.0, ego_length=4.7, ego_width=1.9,
                           lane_width=3.75, only=("匝道行驶", "车辆紧急停止"),
                           log=lambda m: None)
    cmd = router.step(ctx_at(track, 2500.0, speed=8.0))
    typ = type(router.strategy).__name__ if getattr(router, "strategy", None) else "?"
    check("s=2500 由 RampStrategy 接管", "Ramp" in typ, typ)
    check("匝道给了转向", cmd is not None and cmd.steer is not None,
          "None" if cmd is None else "steer=%.3f" % cmd.steer)
    check("匝道转向在 ±1 内", cmd is not None and cmd.steer is not None and abs(cmd.steer) <= 1.0,
          "" if cmd is None else str(cmd.steer))
    check("匝道不接管油门/刹车", cmd is not None and cmd.throttle is None and cmd.brake is None,
          "None" if cmd is None else "thr=%s brk=%s" % (cmd.throttle, cmd.brake))
    check("匝道限速 ≤ vmax 且 ≥ 地板", cmd is not None and cmd.v_cap is not None
          and C.RAMP_V_FLOOR_MS - 1e-6 <= cmd.v_cap <= 9.0 + 1e-6,
          "v_cap=%.2f" % (cmd.v_cap or 0.0))

    print("[3] 匝道终点收速")
    cmd_end = router.step(ctx_at(track, 2990.0, speed=6.0))
    check("终点前 5.5 m 速度 ≤ 4.0", cmd_end is not None and (cmd_end.v_cap or 99) <= 4.0,
          "v_cap=%s" % (None if cmd_end is None else round(cmd_end.v_cap or 0.0, 2)))

    print("[4] 紧急停止 v2（端点坐标系）")
    router.reset()
    end_pt = Pt(3042.9, 0.0)
    cmd_far = router.step(ctx_at(track, 3000.0, speed=8.0, end_point=end_pt))
    typ2 = type(router.strategy).__name__ if getattr(router, "strategy", None) else "?"
    check("s=3000 由 StopAtEndStrategy 接管", "Stop" in typ2, typ2)
    check("远段超速先松油滑行（不刹车）",
          cmd_far is not None and cmd_far.coast is True and cmd_far.brake is None,
          "" if cmd_far is None else "coast=%s brk=%s" % (cmd_far.coast, cmd_far.brake))
    check("end_point 自检通过（走端点口径）", router.strategy.mode == "endpoint",
          "%s last=%.2f" % (router.strategy.mode, router.strategy.last_meas))

    cmd_slow = router.step(ctx_at(track, 3005.0, speed=1.00, end_point=end_pt))
    check("远段低于限速 → 只给 v_cap（不滑行不刹车）",
          cmd_slow is not None and cmd_slow.v_cap is not None
          and not cmd_slow.coast and cmd_slow.brake is None and cmd_slow.throttle is None,
          "" if cmd_slow is None else "v_cap=%s" % cmd_slow.v_cap)

    cmd_fine = router.step(ctx_at(track, 3039.0, speed=1.60, end_point=end_pt))
    check("精细段刹车 ≤ 0.28 且油门清零",
          cmd_fine is not None and cmd_fine.throttle == 0.0
          and (cmd_fine.brake or 0.0) <= C.ESTOP_FINE_BRAKE_MAX + 1e-9,
          "" if cmd_fine is None else "thr=%s brk=%s" % (cmd_fine.throttle, cmd_fine.brake))
    prev = (cmd_fine.brake or 0.0) if cmd_fine is not None else 0.0
    cmd_fine2 = router.step(ctx_at(track, 3039.0, speed=1.40, end_point=end_pt))
    now = (cmd_fine2.brake or 0.0) if cmd_fine2 is not None else 0.0
    check("刹车指令不阶跃（单帧增幅 ≤ 0.02）", abs(now - prev) <= 0.02 + 1e-9,
          "%.3f → %.3f" % (prev, now))

    cmd_early = router.step(ctx_at(track, 3040.0, speed=0.10, end_point=end_pt))
    check("停早了给 0.03 显式蠕行（压 stall）",
          cmd_early is not None and cmd_early.throttle == C.ESTOP_CREEP_THROTTLE
          and cmd_early.brake == 0.0,
          "" if cmd_early is None else "thr=%s brk=%s" % (cmd_early.throttle, cmd_early.brake))

    cmd_stop = router.step(ctx_at(track, 3042.82, speed=0.05, end_point=end_pt,
                                  plat="车辆紧急停止"))
    gear_name = getattr(getattr(cmd_stop, "gear", None), "name", "")
    check("到点：油门清零、保持 D（不挂 P，避免平台 P→D 跳速度）、刹车起于 0.10",
          cmd_stop is not None and cmd_stop.throttle == 0.0 and gear_name == "DRIVE"
          and cmd_stop.brake is not None
          and 0.09 <= cmd_stop.brake <= C.ESTOP_HOLD_BRAKE_MAX + 1e-9,
          "" if cmd_stop is None else "thr=%s brk=%s gear=%s"
          % (cmd_stop.throttle, cmd_stop.brake, gear_name))
    for _ in range(90):
        cmd_hold = router.step(ctx_at(track, 3042.82, speed=0.0, end_point=end_pt,
                                      plat="车辆紧急停止", sim_t=100.0))
    check("保持 3 s 后刹车停在 %.2f（不用满刹 1.0：会抖出 −5 m/s² 丢 +10）"
          % C.ESTOP_HOLD_BRAKE_MAX,
          cmd_hold is not None and abs((cmd_hold.brake or 0.0)
                                       - C.ESTOP_HOLD_BRAKE_MAX) < 1e-6,
          "" if cmd_hold is None else "brk=%s note=%s" % (cmd_hold.brake, cmd_hold.note))
    cmd_launch = router.step(ctx_at(track, 3042.82, speed=0.0, end_point=end_pt,
                                    plat="辅路驶入、驶出", sim_t=101.0))
    check("平台切元素 → 先零刹车起跑（不交棒、brake=0）",
          cmd_launch is not None and cmd_launch.brake == 0.0
          and (cmd_launch.throttle or 0.0) > 0.0,
          "None" if cmd_launch is None else "thr=%s brk=%s"
          % (cmd_launch.throttle, cmd_launch.brake))
    check("起跑段不给转向（横向交回基线，否则出切片后打满冲出路面）",
          cmd_launch is not None and cmd_launch.steer is None,
          "None" if cmd_launch is None else "steer=%s" % cmd_launch.steer)
    step_last = None
    last_s = 3042.82
    brk_bad: list[str] = []
    for k in range(0, 25):
        s = 3042.82 + k * 1.0
        step_last = router.step(ctx_at(track, s, speed=6.5, end_point=end_pt,
                                       plat="辅路驶入、驶出", sim_t=101.0 + k * 0.33))
        if step_last is None:
            break
        last_s = s
        if step_last.brake not in (0.0, None):
            brk_bad.append("%.1f:brk=%s" % (s, step_last.brake))
    check("起跑段全程零刹车（压掉安全闸那两帧 0.35）", not brk_bad, "; ".join(brk_bad[:3]))
    check("起跑至少 20 m 后才交棒（实测最后控制点 s=%.1f，s_end=3042.9）" % last_s,
          last_s - 3042.82 >= 20.0, "最后控制点 %.1f m" % last_s)
    check("交棒后不再抢控制", router.step(ctx_at(track, 3068.0, speed=6.5,
                                                 end_point=end_pt,
                                                 plat="辅路驶入、驶出")) is None)

    print("[5] 紧急停止：没有 end_point 时退回里程口径")
    router_m = ElementRouter(static, route, track, vmax=9.0, ego_length=4.7, ego_width=1.9,
                             lane_width=3.75, only=("车辆紧急停止",), log=lambda m: None)
    cmd_mile = router_m.step(ctx_at(track, 3042.70, speed=0.05))
    check("无 end_point 仍能停住（里程口径）",
          cmd_mile is not None and "hold" in (cmd_mile.note or "")
          and router_m.strategy.mode == "mileage",
          "%s %s" % (router_m.strategy.mode, "" if cmd_mile is None else cmd_mile.note))
    check("滑行模型 allow(45 m)=%.2f" % C.ESTOP_APPROACH_MS2,
          abs(router_m.strategy.allow_speed(45.0)
              - (2 * C.ESTOP_APPROACH_MS2 * 45.0) ** 0.5) < 1e-6)

    print("[6] 辅路（元素 9）")
    router2 = ElementRouter(static, route, track, vmax=9.0, ego_length=4.7, ego_width=1.9,
                            lane_width=3.75, only=("辅路驶入、驶出",), log=lambda m: None)
    cmd_f9 = router2.step(ctx_at(track, 3500.0, speed=8.0))
    typ3 = type(router2.strategy).__name__ if getattr(router2, "strategy", None) else "?"
    check("s=3500 由 FudaoStrategy 接管", "Fudao" in typ3, typ3)
    check("辅路给了转向", cmd_f9 is not None and cmd_f9.steer is not None,
          "None" if cmd_f9 is None else "steer=%.3f" % cmd_f9.steer)
    check("辅路不接管油门（巡航段）",
          cmd_f9 is not None and cmd_f9.throttle is None and cmd_f9.brake is None,
          "None" if cmd_f9 is None else "thr=%s brk=%s" % (cmd_f9.throttle, cmd_f9.brake))
    check("辅路限速 = vmax", cmd_f9 is not None and cmd_f9.v_cap == 9.0,
          "v_cap=%s" % (None if cmd_f9 is None else cmd_f9.v_cap))

    router3 = ElementRouter(static, route, track, vmax=9.0, ego_length=4.7, ego_width=1.9,
                            lane_width=3.75, only=("辅路驶入、驶出",), log=lambda m: None)
    cmd_l1 = router3.step(ctx_at(track, 3100.0, speed=2.0, sim_t=1.0))
    check("起跑缓升：低速时自己给油门、零刹车",
          cmd_l1 is not None and (cmd_l1.throttle or 0.0) > 0.0 and cmd_l1.brake == 0.0,
          "None" if cmd_l1 is None else "thr=%s brk=%s" % (cmd_l1.throttle, cmd_l1.brake))
    check("起跑缓升：油门不超过 %.2f" % C.FUDAO_LAUNCH_THR_MAX,
          cmd_l1 is not None and (cmd_l1.throttle or 0.0) <= C.FUDAO_LAUNCH_THR_MAX + 1e-9,
          "thr=%s" % (None if cmd_l1 is None else cmd_l1.throttle))
    cmd_l2 = router3.step(ctx_at(track, 3120.0, speed=C.FUDAO_LAUNCH_MS + 0.5, sim_t=4.0))
    check("到 %.1f m/s 后交回基线纵向（throttle=None）" % C.FUDAO_LAUNCH_MS,
          cmd_l2 is not None and cmd_l2.throttle is None,
          "None" if cmd_l2 is None else "thr=%s" % cmd_l2.throttle)

    router4 = ElementRouter(static, route, track, vmax=9.0, ego_length=4.7, ego_width=1.9,
                            lane_width=3.75, only=("辅路驶入、驶出",), log=lambda m: None)
    cmd_probe = router4.step(ctx_at(track, 3150.0, speed=6.0, sim_t=9.0))
    strat4 = getattr(router4, "strategy", None)
    if strat4 is not None:
        # 起跑缓升期间不会走到一挡墙的 kick（油门只到 0.30），这里直接模拟「已在巡航段」
        strat4.launch_done = True
    cmd_kick = router4.step(ctx_at(track, 3200.0, speed=C.KICK_WALL_MS, sim_t=10.0))
    check("一挡墙上补一脚 kick（throttle>0、brake=0）",
          cmd_kick is not None and (cmd_kick.throttle or 0.0) > 0.0 and cmd_kick.brake == 0.0,
          "" if cmd_kick is None else "thr=%s brk=%s" % (cmd_kick.throttle, cmd_kick.brake))
    cmd_after = router2.step(ctx_at(track, 3201.0, speed=C.SHIFT_DONE_MS + 0.1, sim_t=10.5))
    check("过了一挡墙就不再 kick",
          cmd_after is not None and cmd_after.throttle is None,
          "" if cmd_after is None else "thr=%s" % cmd_after.throttle)

    router2.reset()
    cmd_hand = router2.step(ctx_at(track, 4400.0, speed=8.0, sim_t=200.0))
    check("终点前 6 m 交棒（返回 None）", cmd_hand is None,
          "None" if cmd_hand is None else "cmd")

    print("[7] 其余元素不受影响")
    router.reset()
    cmd_other = router.step(ctx_at(track, 1700.0, speed=8.0))
    check("s=1700（十字路口）不接管", cmd_other is None, "None" if cmd_other is None else "cmd")

    print()
    if FAILS:
        print("失败 %d 项：%s" % (len(FAILS), ", ".join(FAILS)))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
