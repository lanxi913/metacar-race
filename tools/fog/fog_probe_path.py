# -*- coding: utf-8 -*-
"""港口迷雾：路径诊断——把"车到底撞在什么东西上"一次问清楚。

只做三件事，不碰任务接口：
1. 打印起点到目标直线走廊两侧 15 m 内的静态构件（谁挡在路上）；
2. 打印 fog_route 规划出来的航点（看它是不是让车切出路面）；
3. 用 LaneDriver 走一遍，每秒打印 (x,y) v s e thr brk steer heading，
   卡住时把 api.get_events() 里的事件全吐出来（collision 会在这里现形）。

用法（场景在跑时）：
  python -u fog_probe_path.py --out ...\\logs\\probe1 --from 618.5,120.8 --to 587.5,124.8 --seconds 90
"""
import argparse
import json
import math
import os
import time

from fog_map import RoadGraph, connect_retry, to_plain
import fog_route
from fog_run3 import pose_of
from fog_run5 import LaneDriver

from metacar.models import VehicleControl


def nearest_static(static, a, b, radius=15.0):
    """点到线段 ab 的距离 <= radius 的静态构件。"""
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy or 1e-9
    hits = []
    for o in static.get("static_objects") or []:
        p = o.get("position") or {}
        x, y = float(p.get("x", 0.0)), float(p.get("y", 0.0))
        t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
        d = math.hypot(x - (ax + t * vx), y - (ay + t * vy))
        if d <= radius:
            hits.append((d, o))
    hits.sort(key=lambda h: h[0])
    return hits


def steer_cal(api, note, car="Car_C", thr=0.22, secs=3.0, steer=0.6):
    """实测转向符号：给 +0.6 满舵 3 s，看航向是涨还是跌。"""
    def hold(thr_v, st_v, dur):
        t_end = time.time() + dur
        while time.time() < t_end:
            api.set_vehicle_controls({car: VehicleControl(throttle=thr_v, brake=0.0,
                                                          steering=st_v)})
            time.sleep(0.1)

    def stop():
        for _ in range(15):
            api.set_vehicle_controls({car: VehicleControl(throttle=0.0, brake=0.6,
                                                          steering=0.0)})
            time.sleep(0.1)

    for st in (steer, -steer):
        stop()
        p0, y0, v0 = pose_of(api, car)
        hold(thr, 0.0, 1.5)                                  # 先直行，清掉历史
        p1, y1, _ = pose_of(api, car)
        hold(thr, st, secs)
        p2, y2, _ = pose_of(api, car)
        dyaw = (y2 - y1 + 540.0) % 360.0 - 180.0
        note("[cal] steer=%+.1f  航向 %.1f -> %.1f  Δ=%+.1f°  (%.1f,%.1f)->(%.1f,%.1f) v=%.2f"
             % (st, y1, y2, dyaw, p1[0], p1[1], p2[0], p2[1], v0))
    stop()
    note("[cal] 结论：Δ 为正 = +1 是左舵（航向增大）；Δ 为负 = +1 是右舵")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--from", dest="frm", default="618.5,120.8")
    ap.add_argument("--to", dest="to", default="587.5,124.8")
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--car", default="Car_C")
    ap.add_argument("--cal", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    a = tuple(float(v) for v in args.frm.split(","))
    b = tuple(float(v) for v in args.to.split(","))

    api = connect_retry()
    log = open(os.path.join(args.out, "probe.log"), "w", encoding="utf-8")
    t0 = time.time()

    def note(msg):
        line = "[%6.1f] %s" % (time.time() - t0, msg)
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

    try:
        static = to_plain(api.get_scene_static_data())
        note("static_objects=%d roads=%d" % (len(static.get("static_objects") or []),
                                             len(static.get("roads") or [])))
        if args.cal:
            note("=== 转向符号标定 ===")
            steer_cal(api, note, car=args.car)
        note("=== 直线走廊 %.1f,%.1f -> %.1f,%.1f 两侧 15 m 内的构件 ===" % (a[0], a[1], b[0], b[1]))
        for d, o in nearest_static(static, a, b)[:25]:
            ori = o.get("orientation_degrees") or {}
            note("  d=%5.1f %-26s %-22s pos=(%7.1f,%7.1f) L=%.1f W=%.1f yaw=%.0f"
                 % (d, o.get("id"), o.get("category"),
                    float(o["position"]["x"]), float(o["position"]["y"]),
                    float(o.get("length_m") or 0), float(o.get("width_m") or 0),
                    float(ori.get("z", 0) or 0)))

        graph = RoadGraph(static)
        pos0 = pose_of(api, args.car)[0]
        note("=== 起点 %s (%.1f,%.1f) ===" % (args.car, pos0[0], pos0[1]))
        r = fog_route.route(graph, pos0, b)
        if r is None:
            note("!! 路由失败")
            return
        note("route wp=%d len=%.0f m" % (len(r.pts), r.length()))
        for i, p in enumerate(r.pts):
            note("  wp%02d (%7.1f,%7.1f) v=%.1f" % (i, p[0], p[1], r.speeds[min(i, len(r.speeds) - 1)]))

        drv = LaneDriver(r, 2.65, v_cap=3.0)
        note("driver pts=%d（倒角/压点之后） total=%.0f m" % (len(drv.pts), drv.path.total))
        stall_t = None
        last_log = 0.0
        while time.time() - t0 < args.seconds:
            pos, yaw, speed = pose_of(api, args.car)
            thr, brk, steer = drv.step(pos, yaw, speed, 0.1)
            api.set_vehicle_controls({args.car: VehicleControl(throttle=thr, brake=brk,
                                                              steering=steer)})
            now = time.time() - t0
            if now - last_log >= 1.0:
                last_log = now
                note("  (%7.2f,%7.2f) v=%5.2f s=%6.1f e=%+6.2f thr=%.2f brk=%.2f st=%+5.2f yaw=%6.1f i=%d"
                     % (pos[0], pos[1], speed, drv.s, drv.e, thr, brk, steer, yaw, drv.i))
            if abs(speed) < 0.1:
                if stall_t is None:
                    stall_t = time.time()
                elif time.time() - stall_t > 3.0:
                    note("!! 卡住 @(%.2f,%.2f)" % (pos[0], pos[1]))
                    try:
                        for ev in api.get_events(0):
                            note("   event %s" % json.dumps(to_plain(ev), ensure_ascii=False)[:200])
                    except Exception as exc:                      # noqa: BLE001
                        note("   events 读取失败 %s" % exc)
                    break
            else:
                stall_t = None
            if drv.done:
                note("到达路径终点")
                break
            time.sleep(0.1)
    finally:
        try:
            api.set_vehicle_controls({args.car: VehicleControl(throttle=0.0, brake=1.0)})
        except Exception:                                          # noqa: BLE001
            pass
        log.close()
        try:
            api.close()
        except Exception:                                          # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
