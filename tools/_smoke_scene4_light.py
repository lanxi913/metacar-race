"""多路口信号灯闸门的离线冒烟（2026-10-25 m04149 口径重写）。

新口径：**不判断车速够不够过线**，只认「非绿 → 绿」这一个状态跳变；
非绿就松油滑行到线前 `LIGHT_STOP_AT_M` 处停住，变绿的那一帧立刻放行。

场景：一条 2000 m 直线路线，5 条停止线（200/300/400/500/600 m），
只有 1/3/5 号路口给灯（2/4 号故意不给，验证「没找到灯就不限制」）。
"""
import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from metacar.models import TrafficLightState  # noqa: E402

from metacar_race.scene4 import config as C  # noqa: E402
from metacar_race.scene4.safety import LightGate  # noqa: E402
from metacar_race.scene4.track import RouteTrack  # noqa: E402

FAILS = []


def check(ok, name, extra=""):
    print(("OK   " if ok else "FAIL ") + name + (("  " + extra) if extra else ""))
    if not ok:
        FAILS.append(name)


def cap_of(gap: float) -> float:
    """safety 里的允许车速曲线：sqrt(2·0.35·(gap − 0.8))。"""
    return math.sqrt(2.0 * 0.35 * max(gap - 0.80, 0.0))


def main() -> int:
    route = [types.SimpleNamespace(x=float(i), y=0.0, z=0.0) for i in range(2001)]
    track = RouteTrack(route)

    roads = [types.SimpleNamespace(stop_line=[types.SimpleNamespace(x=s, y=0.0)])
             for s in (200.0, 300.0, 400.0, 500.0, 600.0)]

    def light(lid, s, state, remain):
        return types.SimpleNamespace(id=lid,
                                     position=types.SimpleNamespace(x=s, y=0.0, z=0.0),
                                     straight_state=state,
                                     straight_remaining_time=remain)

    def groups_with(l3_state, l3_remain, l1_state=TrafficLightState.GREEN):
        return [types.SimpleNamespace(id="g0", traffic_lights=[
            light("L1", 200.0, l1_state, 10.0),
            light("L3", 400.0, l3_state, l3_remain),
            light("L5", 600.0, TrafficLightState.RED, 20.0),
        ])]

    def new_gate():
        logs = []
        return LightGate(enabled=True, log=logs.append, track=track), logs

    # 1) 绿灯通过：灯是绿的就不限制，也不再算 ETA
    gate, _ = new_gate()
    g = groups_with(TrafficLightState.GREEN, 0.80)
    d = gate.update(groups=g, roads=roads, x=150.0, y=0.0, yaw=0.0, speed=9.0)
    check(abs((d.stop_gap or -1) - 50.0) < 1.0, "150m 盯住 200m 那条线",
          "gap=%.1f" % (d.stop_gap or -1))
    check(d.state == "GREEN", "读到绿灯", str(d.state))
    check(d.v_cap is not None and abs(d.v_cap - C.LIGHT_APPROACH_MS) < 1e-6 and not d.waiting,
          "绿灯也不抢行：进决策距离就把巡航压到 %.1f m/s（方案 B）" % C.LIGHT_APPROACH_MS,
          "cap=%s waiting=%s" % (d.v_cap, d.waiting))

    # 2) 没有灯的路口不限制
    gate, _ = new_gate()
    d2 = gate.update(groups=g, roads=roads, x=220.0, y=0.0, yaw=0.0, speed=8.0)
    check(abs((d2.stop_gap or -1) - 80.0) < 1.0, "过线后自动换下一条线（300m）",
          "gap=%s" % d2.stop_gap)
    check(d2.v_cap is None and not d2.waiting, "没有灯的路口不限制", "cap=%s" % d2.v_cap)

    # 3) 红灯：60 m 处松油滑行，cap = sqrt(2*0.35*(60-0.8))
    gate, _ = new_gate()
    g_red = groups_with(TrafficLightState.RED, 20.0)
    d3 = gate.update(groups=g_red, roads=roads, x=340.0, y=0.0, yaw=0.0, speed=8.0)
    check(d3.state == "RED" and d3.waiting, "60m 红灯 → 决定停、开始滑行",
          "state=%s waiting=%s" % (d3.state, d3.waiting))
    check(d3.v_cap is not None and abs(d3.v_cap - C.LIGHT_APPROACH_MS) < 1e-6,
          "60m 处 cap 被方案 B 压到 %.1f（滑行允许值 %.2f 更松）"
          % (C.LIGHT_APPROACH_MS, cap_of(60.0)),
          "cap=%s" % (None if d3.v_cap is None else round(d3.v_cap, 2)))

    # 4) 贴线红灯：holding、cap=0（路线点距 1 m，投影 s 会落到最近的点上）
    d4 = gate.update(groups=g_red, roads=roads, x=399.9, y=0.0, yaw=0.0, speed=0.3)
    check(d4.holding and d4.v_cap == 0.0 and not d4.released,
          "贴线红灯 → 停住（不越线）",
          "hold=%s cap=%s gap=%s released=%s"
          % (d4.holding, d4.v_cap, d4.stop_gap, d4.released))

    # 5) 红转绿：那一帧立即放行
    g_green = groups_with(TrafficLightState.GREEN, 3.0)
    d5 = gate.update(groups=g_green, roads=roads, x=399.9, y=0.0, yaw=0.0, speed=0.0)
    check(d5.released and not d5.waiting and d5.v_cap is None,
          "红 → 绿的那一帧：立即放行（不看剩余时间）",
          "released=%s waiting=%s cap=%s" % (d5.released, d5.waiting, d5.v_cap))

    # 6) 一直绿（没红过）不算放行
    gate6, _ = new_gate()
    d6 = gate6.update(groups=g_green, roads=roads, x=399.4, y=0.0, yaw=0.0, speed=0.0)
    check(not d6.released and not d6.waiting, "一直绿 → 既不等也不触发 released",
          "released=%s waiting=%s" % (d6.released, d6.waiting))

    # 7) 黄灯也算非绿：继续等
    gate7, _ = new_gate()
    d7 = gate7.update(groups=g_red, roads=roads, x=394.0, y=0.0, yaw=0.0, speed=2.0)
    d7b = gate7.update(groups=groups_with(TrafficLightState.YELLOW, 1.0), roads=roads,
                       x=394.0, y=0.0, yaw=0.0, speed=0.4)
    check(d7.waiting and d7b.waiting and not d7b.released,
          "红 → 黄 仍在等（不放行）", "waiting=%s released=%s" % (d7b.waiting, d7b.released))

    # 8) 130 m 决策距离：更远不预先减速（只留 200/600 两条线，排除中间无灯路口）
    gate8, _ = new_gate()
    roads2 = [roads[0], roads[4]]
    far = gate8.update(groups=g_red, roads=roads2, x=470.0, y=0.0, yaw=0.0, speed=8.0)
    check(abs((far.stop_gap or -1) - 130.0) < 1.0 and far.waiting,
          "130m 处进入决策（边界）", "gap=%s waiting=%s" % (far.stop_gap, far.waiting))
    check(far.v_cap is not None and abs(far.v_cap - C.LIGHT_APPROACH_MS) < 1e-6,
          "130m 边界同样先把巡航压到 %.1f m/s" % C.LIGHT_APPROACH_MS,
          "cap=%s" % far.v_cap)
    gate8b, _ = new_gate()
    nearer = gate8b.update(groups=g_red, roads=roads2, x=468.0, y=0.0, yaw=0.0, speed=8.0)
    check(nearer.v_cap is None and not nearer.waiting,
          "132m（窗口外）不预先减速", "cap=%s waiting=%s" % (nearer.v_cap, nearer.waiting))

    # 9) 红灯 10 m 处 cap 收到 ~2.5
    gate9, _ = new_gate()
    g9 = groups_with(TrafficLightState.RED, 20.0)
    d9 = gate9.update(groups=g9, roads=roads, x=390.0, y=0.0, yaw=0.0, speed=3.0)
    check(d9.v_cap is not None and abs(d9.v_cap - cap_of(10.0)) < 0.05,
          "10m 处 cap≈%.2f" % cap_of(10.0),
          "cap=%s" % (None if d9.v_cap is None else round(d9.v_cap, 2)))

    # 10) 全部路口过完 → 不限制
    gate10, _ = new_gate()
    d10 = gate10.update(groups=g_green, roads=roads, x=650.0, y=0.0, yaw=0.0, speed=8.0)
    check(d10.v_cap is None and d10.stop_gap is None, "全部路口过完 → 不限制", "")

    # 11) 路口组（m04276 方案 A）：3 条线只隔 12 m → 算一个路口，组首绿灯就承诺通过，
    #     组内后面两条线是红灯也不再停（r20 就是在路口中间被第 2/3 条线拦停）。
    roads3 = [types.SimpleNamespace(stop_line=[types.SimpleNamespace(x=s, y=0.0)])
              for s in (400.0, 412.0, 424.0)]

    def groups3(state0):
        return [types.SimpleNamespace(id="g9", traffic_lights=[
            light("A", 400.0, state0, 8.0),
            light("B", 412.0, TrafficLightState.RED, 8.0),
            light("C", 424.0, TrafficLightState.RED, 8.0),
        ])]

    gate11, _ = new_gate()
    d11 = gate11.update(groups=groups3(TrafficLightState.GREEN), roads=roads3,
                        x=360.0, y=0.0, yaw=0.0, speed=6.0)
    check(gate11._groups() == [(400.0, 424.0)], "30m 内 3 条线聚成 1 个路口组",
          str(gate11._groups()))
    check(not d11.waiting and gate11._committed is None,
          "组首绿灯、但还差 40 m → 不锁定（怕半路变红硬闯）",
          "waiting=%s committed=%s" % (d11.waiting, gate11._committed))
    gate11.update(groups=groups3(TrafficLightState.GREEN), roads=roads3,
                  x=380.0, y=0.0, yaw=0.0, speed=6.0)      # 距组首 20 m ≤ LIGHT_COMMIT_M
    check(gate11._committed == (400.0, 424.0), "20m 处绿灯 → 锁定整组一次通过",
          str(gate11._committed))
    d11b = gate11.update(groups=groups3(TrafficLightState.RED), roads=roads3,
                         x=413.0, y=0.0, yaw=0.0, speed=6.0)  # 组内第二条线的灯是红的
    check(not d11b.waiting and d11b.v_cap is None,
          "锁定后组内变红也不停、不限速（r20 就是被它拦停的）",
          "waiting=%s cap=%s gap=%s" % (d11b.waiting, d11b.v_cap, d11b.stop_gap))
    d11c = gate11.update(groups=groups3(TrafficLightState.GREEN), roads=roads3,
                         x=440.0, y=0.0, yaw=0.0, speed=6.0)
    check(d11c.stop_gap is None and d11c.v_cap is None, "整组走完后不再限制",
          "gap=%s cap=%s" % (d11c.stop_gap, d11c.v_cap))

    # 12) 组首红灯：仍然要停（组内承诺只对「已经放行」成立）
    gate12, _ = new_gate()
    d12 = gate12.update(groups=groups3(TrafficLightState.RED), roads=roads3,
                        x=380.0, y=0.0, yaw=0.0, speed=6.0)
    check(d12.waiting and d12.v_cap is not None and d12.v_cap <= 6.0,
          "组首红灯 → 照旧滑行等绿", "waiting=%s cap=%s" % (d12.waiting, d12.v_cap))

    print()
    if FAILS:
        print("失败 %d 项：%s" % (len(FAILS), " / ".join(FAILS)))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
