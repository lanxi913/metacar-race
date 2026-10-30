# -*- coding: utf-8 -*-
"""港口迷雾：完整任务流水线（当前实现到"路障扫描"，后续接力）。

阶段 0  信号 A→B→C（Car_C，实测 176 s 三个都成）
阶段 1  路障扫描：开向两片 `static_task_obstacle` 模糊区中心，10 m 内 `scan`
阶段 2  打印任务态（泊位是否解锁、路口归属），供下一步接力

用法：
  python -u fog_mission.py --out ...\\logs\\fog_m1 --seconds 560
"""
import argparse
import json
import math
import os
import time

from metacar.models import GearMode, VehicleControl
from fog_map import RoadGraph, connect_retry, to_plain
import fog_nav
import fog_route
from fog_run3 import pose_of, clip, KEY_RE, WHEELBASE_RATIO, MAX_STEER
from fog_run5 import LaneDriver

SIGNALS = [("Signal_A", (587.47, 124.78)),
           ("Signal_B", (766.5, -34.5)),
           ("Signal_C", (720.5, -96.0))]
STOP_FAR = 6.0
CENTER = (728.79, -20.22)   # 中央路口中心（实测 request_entry 在此 12 m 内才可申请）


class Ctx(object):
    def __init__(self, api, vids, graph, wb, out, t0, note):
        self.api = api
        self.vids = vids
        try:
            self.rev_goal = {}
        except Exception:
            pass
        self.graph = graph
        self.wb = wb
        self.out = out
        self.t0 = t0
        self.note = note


def back_off_roadblock(ctx, vid, seconds, clear_m=12.0):
    """扫完路障后把车倒出它的包围盒：不倒车直接开，会一路刮着 Roadblock_East 走。"""
    rb = (766.5, -88.2)
    for k in range(5):
        try:
            p, _y, _v = pose_of(ctx.api, vid)
        except Exception:                                      # noqa: BLE001
            return
        d = math.hypot(p[0] - rb[0], p[1] - rb[1])
        if d > clear_m:
            ctx.note("    %s 离路障 %.1f m，够了" % (vid, d))
            return
        ctx.note("    %s 离路障只有 %.1f m，倒车让开" % (vid, d))
        for v in ctx.vids:
            _ctrl(ctx, v, 0.0, 0.4 if v != vid else 0.0, 0.0)
        veh = VehicleControl(throttle=0.34, brake=0.0, steering=0.0)
        try:
            veh.gear = GearMode.REVERSE
        except Exception:                                      # noqa: BLE001
            pass
        ctx.ctl[vid] = veh
        t_end = time.time() + 2.4
        while time.time() < t_end and time.time() - ctx.t0 < seconds:
            _send(ctx, {})
            time.sleep(0.1)
        _ctrl(ctx, vid, 0.0, 0.6, 0.0)
        _send(ctx, {})
        reset_pose(vid)


def drive_to(ctx, vid, goal_xy, seconds, stop_xy=None, stop_m=STOP_FAR,
             poll=None, approach_xy=None):
    """开到 goal_xy；stop_xy+stop_m 给了就按距离停车。返回 (ok, pos, speed)。

    poll(pos, speed, ob) 每帧调用，返回非 None 时提前结束并把值作为第三项返回。
    """
    pos, _y, _v = pose_of(ctx.api, vid)
    r = fog_route.route(ctx.graph, pos, goal_xy)
    if r is None:
        ctx.note("  [nav] 无法规划到 (%.1f,%.1f)" % goal_xy)
        return False, pos, 0.0, None
    drv = LaneDriver(r, ctx.wb, v_cap=5.1, stop_xy=stop_xy, stop_m=stop_m)
    ctx.note("  [nav] -> (%.1f,%.1f) %d wp %.0f m"
             % (goal_xy[0], goal_xy[1], len(r), r.length()))
    got = None
    last = 0.0
    stall_t0 = None
    kick_t0 = None
    while time.time() - ctx.t0 < seconds:
        pos, yaw, speed = pose_of(ctx.api, vid)
        now = time.time()
        if kick_t0 is not None:
            if now - kick_t0 < 1.5:
                ctrls = {v: VehicleControl(throttle=0.0, brake=0.35, steering=0.0)
                         for v in ctx.vids if v != vid}
                ctrls[vid] = VehicleControl(throttle=0.55, brake=0.0, steering=steer)
                ctx.api.set_vehicle_controls(ctrls)
                time.sleep(0.1)
                continue
            kick_t0 = None
            stall_t0 = now
        if abs(speed) < 0.15:
            if stall_t0 is None:
                stall_t0 = now
            elif now - stall_t0 > 4.0:
                ctx.note("    %s [stall] 卡住 %.0fs @(%.1f,%.1f)，踢油门"
                         % (vid, now - stall_t0, pos[0], pos[1]))
                kick_t0 = now
                stall_t0 = None
                continue
        else:
            stall_t0 = None
        ctrls = {v: VehicleControl(throttle=0.0, brake=0.35, steering=0.0)
                 for v in ctx.vids if v != vid}
        thr, brk, steer = drv.step(pos, yaw, speed, 0.1)
        ctrls[vid] = VehicleControl(throttle=thr, brake=brk, steering=steer)
        ctx.api.set_vehicle_controls(ctrls)
        ob = to_plain(ctx.api.get_observation(vid))
        if poll is not None:
            got = poll(pos, speed, ob)
            if got is not None:
                break
        if time.time() - ctx.t0 - last > 8.0:
            last = time.time() - ctx.t0
            ctx.note("    %s (%.1f,%.1f) v=%.2f seg=%d/%d"
                     % (vid, pos[0], pos[1], speed, drv.i, len(drv.pts)))
        if drv.done:
            break
        time.sleep(0.1)
    if got is not None:
        # poll 命中（例如刚读到信号码）时不要先刹 2 s，立刻返回让调用方马上提交
        pos, _y, speed = pose_of(ctx.api, vid)
        return True, pos, speed, got
    # 停稳
    t_stop = time.time()
    while time.time() - t_stop < 2.0:
        ctx.api.set_vehicle_controls({v: VehicleControl(throttle=0.0, brake=0.4,
                                                        steering=0.0)
                                      for v in ctx.vids})
        time.sleep(0.15)
    pos, _y, speed = pose_of(ctx.api, vid)
    return drv.done, pos, speed, got


def hold_still(ctx, vid, seconds=6.0, speed_eps=0.06):
    """踩住刹车直到车速接近 0。刹车逐帧爬上去（一帧 0→0.6 会被判一次加速度超限）。"""
    t_end = time.time() + seconds
    sp = 9.9
    cur = 0.15
    while time.time() < t_end:
        pos, yaw, sp = pose_of(ctx.api, vid)
        hb = getattr(ctx, "hold_brk", None) or {}
        ctrls = {v: VehicleControl(throttle=0.0,
                                   brake=max(min(cur, 0.40), hb.get(v, 0.0)),
                                   steering=0.0)
                 for v in ctx.vids}
        ctrls[vid] = VehicleControl(throttle=0.0, brake=cur, steering=0.0)
        ctx.api.set_vehicle_controls(ctrls)
        cur = min(0.60, cur + 0.12)
        if abs(sp) < speed_eps:
            break
        time.sleep(0.12)
    return sp


def creep_to(ctx, vid, goal_xy, seconds, stop_m, v_cap=1.4):
    """直线低速靠近 goal_xy，直到距离 <= stop_m。"""
    while time.time() - ctx.t0 < seconds:
        pos, yaw, speed = pose_of(ctx.api, vid)
        d = math.hypot(goal_xy[0] - pos[0], goal_xy[1] - pos[1])
        if d <= stop_m:
            return True, pos
        want = math.degrees(math.atan2(goal_xy[1] - pos[1], goal_xy[0] - pos[0]))
        diff = (want - yaw + 540.0) % 360.0 - 180.0
        steer = clip(diff / 45.0, -1.0, 1.0) * MAX_STEER
        thr = 0.0 if speed > v_cap else 0.16
        brk = 0.0 if speed > 0.4 else 0.0
        if abs(diff) > 50.0:
            thr, brk = 0.0, 0.35
        hb = getattr(ctx, "hold_brk", None) or {}
        ctrls = {v: VehicleControl(throttle=0.0, brake=max(0.35, hb.get(v, 0.0)),
                                   steering=0.0)
                 for v in ctx.vids}
        ctrls[vid] = VehicleControl(throttle=thr, brake=brk, steering=steer)
        ctx.api.set_vehicle_controls(ctrls)
        time.sleep(0.1)
    pos, _y, _v = pose_of(ctx.api, vid)
    return False, pos


def sig_state(ctx, vid, sig_id):
    """返回 (距离, 状态, key)。"""
    ob = to_plain(ctx.api.get_observation(vid))
    for it in (ob.get("visible_objects") or []):
        if str(it.get("object_id") or "") == sig_id:
            m = KEY_RE.search(it.get("hint") or "")
            return (it.get("distance_m") or 999.0, it.get("state") or "",
                    (m.group(0).replace(" ", "") if m else ""))
    return (999.0, "", "")



GATE_BOX = (723.8, 733.8, -30.0, 10.0)   # x0,x1,y0,y1：交叉口 6 m 互斥区所在的闸口竖段


def grid_with_gate(ctx, inflate=2.4):
    """返回一个把闸口竖段标记为不可通行的栅格图（避免未授权穿越）。"""
    if getattr(ctx, "grid_gate", None) is None:
        st = to_plain(ctx.api.get_scene_static_data())
        g = fog_nav.Grid(st)
        x0, x1, y0, y1 = GATE_BOX
        try:
            blk = g.block            # bytearray(w*h)，1 = 不可通行
            i0 = max(0, int((x0 - g.x0) / g.res))
            i1 = min(g.w - 1, int((x1 - g.x0) / g.res))
            j0 = max(0, int((y0 - g.y0) / g.res))
            j1 = min(g.h - 1, int((y1 - g.y0) / g.res))
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    blk[j * g.w + i] = 1
        except Exception as exc:
            ctx.note("[nav] gate mask failed: %s" % exc)
        ctx.grid_gate = g
    return ctx.grid_gate


def gate_path(ctx, frm, to):
    g = grid_with_gate(ctx)
    pts = g.astar(frm, to)
    if pts:
        pts = fog_nav.smooth(pts, g)
    return pts or [frm, to]


def nav_path(ctx, frm, to, inflate=2.4, avoid=None, avoid_r=4.0):
    """用栅格 A* 规划（避开静态物），返回折线航点。"""
    if getattr(ctx, "grid", None) is None:
        st = to_plain(ctx.api.get_scene_static_data())
        ctx.grid = fog_nav.Grid(st)
    # avoid：动态障碍按圆形禁区抹掉——static_objects 里没有它，不抹的话 A* 会
    # 直接从路障身上穿过去（15:14 那局 Car_C 的 4 点路线压着 Roadblock_East 走，
    # 撞了 60+ 条、卡死 5 次）。ctx.dyn_avoid 是全局禁区表，每辆车、每次规划都带上。
    zones = list(avoid or []) + list(getattr(ctx, "dyn_avoid", None) or [])
    saved = None
    if zones:
        saved = bytes(ctx.grid.block)
        for z in zones:
            ax, ay = float(z[0]), float(z[1])
            rr = float(z[2]) if len(z) > 2 else avoid_r
            try:
                ctx.grid._disc(ax, ay, rr, block=True)
            except Exception:                              # noqa: BLE001
                pass
    try:
        pts = ctx.grid.astar(frm, to)
        if pts:
            pts = fog_nav.smooth(pts, ctx.grid)
    finally:
        if saved is not None:
            ctx.grid.block = bytearray(saved)
    return pts or [frm, to]


def stage_signals(ctx, seconds):
    """沿车道开到信号旁，**在车上还带着速度、刚进入 7 m 视野时立刻提交**
    （fog_run5 里唯一成功过的路径就是这条：一边滑行一边 submit，granted=True）。
    """
    ok = {}
    for sig_id, sig_xy in SIGNALS:
        if time.time() - ctx.t0 > seconds:
            ctx.note("time up before %s" % sig_id)
            break
        ctx.note("  --- %s -> (%.1f,%.1f)" % (sig_id, sig_xy[0], sig_xy[1]))
        state = {"tries": 0}

        def poll(pos, speed, ob, _sig=sig_id, _st=state):
            for it in (ob.get("visible_objects") or []):
                if str(it.get("object_id") or "") != _sig:
                    continue
                m = KEY_RE.search(it.get("hint") or "")
                key = m.group(0).replace(" ", "") if m else ""
                st = it.get("state") or ""
                d = it.get("distance_m") or 999.0
                if key and st == "key_visible":
                    if _st["tries"] < 4:
                        _st["tries"] += 1
                        res = to_plain(ctx.api.interact(_sig and "Car_C", _sig,
                                                        "submit_key", {"key": key}))
                        ctx.note("    submit %s %s d=%.1f v=%.2f -> %s %s"
                                 % (_sig, key, d, speed, res.get("granted"),
                                    res.get("message")))
                        if res.get("granted"):
                            _st["key"] = key
                            return key
            return None

        if sig_id == "Signal_B" and not getattr(ctx, "prepos", False):
            # C 去 B 的这 130 s 里，把一直闲着的 D 送到泊位 A 的见证位（约 170 m）
            ctx.prepos = True
            try:
                pos_c = pose_of(ctx.api, "Car_C")[0]
                rr = fog_route.route(ctx.graph, pos_c, sig_xy)
                jobs = [{"vid": "Car_C", "route": fog_route.Route(rr.pts, rr.speeds),
                         "v_cap": 4.8, "stop_xy": sig_xy, "stop_m": STOP_FAR,
                         "poll": poll, "limit": 300.0}]
                pd = pose_of(ctx.api, "Car_D")[0]
                jobs.append({"vid": "Car_D",
                             "pts": nav_path(ctx, pd, (631.0, -50.7)),
                             "v_cap": 4.2, "stop_xy": (631.0, -50.7),
                             "stop_m": 2.0, "limit": 170.0})
                ctx.note("[pre] C 去 B 的同时把 D 送到泊位 A 见证位")
                got_b = drive_parallel(ctx, jobs, seconds)
                if got_b.get("Car_C"):
                    state["key"] = got_b["Car_C"]
            except Exception as exc:                               # noqa: BLE001
                ctx.note("[pre] 并行送去见证位失败 %s" % exc)
        else:
            drive_to(ctx, "Car_C", sig_xy, seconds, stop_xy=sig_xy, stop_m=STOP_FAR,
                     poll=poll)
        if state.get("key"):
            ok[sig_id] = state["key"]
        else:
            # 滑行中没提交成功：原地停住，再按状态重试几次
            for attempt in range(4):
                d, st, key = sig_state(ctx, "Car_C", sig_id)
                ctx.note("    重试 %d: %s d=%.1f state=%s key=%s"
                         % (attempt + 1, sig_id, d, st, key or "-"))
                if d < 900 and st == "key_visible":
                    creep_to(ctx, "Car_C", sig_xy, seconds, max(3.4, d - 0.8),
                             v_cap=1.0)
                elif d > 7.0:
                    creep_to(ctx, "Car_C", sig_xy, seconds, max(4.0, d - 1.2),
                             v_cap=1.0)
                hold_still(ctx, "Car_C", seconds=1.5)
                d, st, key = sig_state(ctx, "Car_C", sig_id)
                if key:
                    res = to_plain(ctx.api.interact("Car_C", sig_id, "submit_key",
                                                    {"key": key}))
                    ctx.note("    submit %s %s d=%.1f -> %s %s"
                             % (sig_id, key, d, res.get("granted"),
                                res.get("message")))
                    if res.get("granted"):
                        ok[sig_id] = key
                        break
            else:
                ctx.note("  !! %s 仍未通过" % sig_id)
        ts = to_plain(ctx.api.get_task_state())
        ctx.note("  signals=%s/%s phase=%s"
                 % (ts.get("verified_signal_count"), ts.get("total_signal_count"),
                    ts.get("phase")))
    return ok


def _segments_in_zone(graph, zone):
    """返回落在矩形内的道路线段（按车道中心线，begin→end）。"""
    x0, y0, x1, y1 = zone
    segs = []
    for rid, road in graph.roads.items():
        a, b = road["begin"], road["end"]
        mx, my = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
        if x0 - 2 <= mx <= x1 + 2 and y0 - 2 <= my <= y1 + 2:
            if road["len"] >= 3.0:
                segs.append((tuple(a), tuple(b)))
    return segs


def _chain_segments(segs, start):
    """贪心把线段串成一条折线（每条线段沿自身方向走，天然在车道上）。"""
    cur = start
    rest = list(segs)
    pts = [tuple(start)]
    while rest:
        best = None
        for i, (a, b) in enumerate(rest):
            for p, q in ((a, b), (b, a)):
                d = math.hypot(p[0] - cur[0], p[1] - cur[1])
                if best is None or d < best[0]:
                    best = (d, i, p, q)
        _d, i, p, q = best
        rest.pop(i)
        pts.append(tuple(p))
        pts.append(tuple(q))
        cur = q
    return pts


def drive_polyline(ctx, vid, pts, seconds, poll=None, v_cap=3.8):
    """沿给定折线（不做路网规划）开过去，边开边 poll。"""
    if len(pts) < 2:
        pts = [pts[0], (pts[0][0] + 0.1, pts[0][1])]
    dense = []
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        n = max(1, int(seg / 2.0))
        for k in range(n):
            dense.append((ax + (bx - ax) * k / n, ay + (by - ay) * k / n))
    dense.append(tuple(pts[-1]))
    route = fog_route.Route(dense, [v_cap] * len(dense))
    drv = LaneDriver(route, ctx.wb, v_cap=v_cap)
    got = None
    last = 0.0
    stall_t0 = None
    kick_t0 = None
    while time.time() - ctx.t0 < seconds:
        pos, yaw, speed = pose_of(ctx.api, vid)
        now0 = time.time()
        if kick_t0 is not None:
            if now0 - kick_t0 < 1.6:
                ctrls = {v: VehicleControl(throttle=0.0, brake=0.35, steering=0.0)
                         for v in ctx.vids if v != vid}
                ctrls[vid] = VehicleControl(throttle=0.62, brake=0.0, steering=0.20)
                ctx.api.set_vehicle_controls(ctrls)
                time.sleep(0.1)
                continue
            kick_t0 = None
            stall_t0 = now0
        if abs(speed) < 0.15:
            if stall_t0 is None:
                stall_t0 = now0
            elif now0 - stall_t0 > 3.5:
                ctx.note("    %s [stall] @(%.1f,%.1f) seg=%d/%d 踢油门"
                         % (vid, pos[0], pos[1], drv.i, len(drv.pts)))
                kick_t0 = now0
                stall_t0 = None
                continue
        else:
            stall_t0 = None
        ctrls = {v: VehicleControl(throttle=0.0, brake=0.35, steering=0.0)
                 for v in ctx.vids if v != vid}
        thr, brk, steer = drv.step(pos, yaw, speed, 0.1)
        ctrls[vid] = VehicleControl(throttle=thr, brake=brk, steering=steer)
        ctx.api.set_vehicle_controls(ctrls)
        ob = to_plain(ctx.api.get_observation(vid))
        if poll is not None:
            got = poll(pos, speed, ob)
            if got is not None:
                break
        if time.time() - ctx.t0 - last > 10.0:
            last = time.time() - ctx.t0
            ctx.note("    %s (%.1f,%.1f) v=%.2f seg=%d/%d"
                     % (vid, pos[0], pos[1], speed, drv.i, len(drv.pts)))
        if drv.done:
            break
        time.sleep(0.1)
    return got


def stage_scan(ctx, seconds):
    ts = to_plain(ctx.api.get_task_state())
    if not ts.get("roadblock_active"):
        ctx.note("[scan] roadblock 未激活，跳过")
        return False
    zones = []
    for o in ts.get("shared_task_objects") or []:
        if o.get("object_type") != "static_task_obstacle":
            continue
        unc = o.get("uncertainty_region") or {}
        mn, mx = unc.get("minimum") or {}, unc.get("maximum") or {}
        if not mn or not mx:
            continue
        zones.append((o.get("object_id"), (mn["x"], mn["y"], mx["x"], mx["y"])))
    hit = {}

    def poll(pos, speed, ob, _hit=hit):
        for it in (ob.get("visible_objects") or []):
            oid = str(it.get("object_id") or "")
            if it.get("object_type") == "roadblock" or "Roadblock" in oid:
                d = it.get("distance_m") or 999
                _hit["id"] = oid
                _hit["d"] = d
                if d <= 10.5:
                    return oid
            else:
                key = (it.get("object_type") or "", oid)
                if key not in _hit.setdefault("seen", {}):
                    _hit["seen"][key] = (it.get("position") or {}, it.get("distance_m"))
                    ctx.note("    [seen] %-18s %-12s d=%.0f %s"
                             % (oid, it.get("object_type"),
                                it.get("distance_m") or -1,
                                json.dumps(it.get("position") or {},
                                           ensure_ascii=False)[:56]))
        return None

    # 车刚验完 C 号就在灯塔旁边，路障可能已经在视野里：先就地处理，省一趟
    pos0, _y, _v = pose_of(ctx.api, "Car_C")
    ob0 = to_plain(ctx.api.get_observation("Car_C"))
    for it in (ob0.get("visible_objects") or []):
        oid = str(it.get("object_id") or "")
        if it.get("object_type") == "roadblock" or "Roadblock" in oid:
            d0 = it.get("distance_m") or 999.0
            rpos = it.get("position") or {}
            ctx.note("[scan] 已在视野: %s d=%.1f" % (oid, d0))
            if d0 > 9.5 and rpos.get("x") is not None:
                creep_to(ctx, "Car_C", (rpos["x"], rpos["y"]), seconds, 8.5, v_cap=1.2)
                hold_still(ctx, "Car_C", seconds=1.5)
            for attempt in range(3):
                res = to_plain(ctx.api.interact("Car_C", oid, "scan"))
                ctx.note("[scan] scan(%d) %s -> %s"
                         % (attempt + 1, oid,
                            json.dumps(res, ensure_ascii=False)[:200]))
                if res.get("granted"):
                    ts2 = to_plain(ctx.api.get_task_state())
                    ctx.note("[scan] unlocked=%s/%s"
                             % (ts2.get("unlocked_destination_count"),
                                ts2.get("total_destination_count")))
                    for o in ts2.get("shared_task_objects") or []:
                        if o.get("object_type") == "roadblock":
                            ctx.note("[scan] %s road_ids=%s pos=%s"
                                     % (o.get("object_id"), o.get("road_ids"),
                                        json.dumps(o.get("position") or {},
                                                   ensure_ascii=False)[:70]))
                    dump_state(ctx, "after_scan")
                    return True
                hold_still(ctx, "Car_C", seconds=1.0)
            break

    # 先扫离当前位置近的那一片
    pos0, _y, _v = pose_of(ctx.api, "Car_C")
    zones.sort(key=lambda z: math.hypot(0.5 * (z[1][0] + z[1][2]) - pos0[0],
                                        0.5 * (z[1][1] + z[1][3]) - pos0[1]))
    for zid, zone in zones:
        segs = _segments_in_zone(ctx.graph, zone)
        if not segs:
            continue
        pos, _y, _v = pose_of(ctx.api, "Car_C")
        seq = _chain_segments(segs, pos)
        total = sum(math.hypot(seq[i + 1][0] - seq[i][0], seq[i + 1][1] - seq[i][1])
                    for i in range(len(seq) - 1))
        ctx.note("[scan] %s 车道 %d 段 / 折线 %d 点 / 约 %.0f m"
                 % (zid, len(segs), len(seq), total))
        drive_polyline(ctx, "Car_C", seq, seconds, poll=poll, v_cap=3.2)
        if hit.get("id") and (hit.get("d") or 999) <= 10.5:
            ctx.note("[scan] 发现 %s d=%.1f，先停稳再扫"
                     % (hit["id"], hit.get("d") or -1))
            hold_still(ctx, "Car_C", seconds=2.0)
            res = {"granted": False}
            for attempt in range(3):
                res = to_plain(ctx.api.interact("Car_C", hit["id"], "scan"))
                ctx.note("[scan] scan(%d) -> %s"
                         % (attempt + 1, json.dumps(res, ensure_ascii=False)[:200]))
                if res.get("granted"):
                    break
                hold_still(ctx, "Car_C", seconds=1.0)
            ctx.note("[scan] scan -> %s" % json.dumps(res, ensure_ascii=False)[:300])
            ts2 = to_plain(ctx.api.get_task_state())
            ctx.note("[scan] phase=%s unlocked=%s/%s"
                     % (ts2.get("phase"), ts2.get("unlocked_destination_count"),
                        ts2.get("total_destination_count")))
            for o in ts2.get("shared_task_objects") or []:
                if o.get("object_type") == "roadblock":
                    ctx.note("[scan] %s road_ids=%s position=%s"
                             % (o.get("object_id"), o.get("road_ids"),
                                json.dumps(o.get("position") or {},
                                           ensure_ascii=False)[:80]))
            dump_state(ctx, "after_scan")
            return True
        ctx.note("[scan] %s 这一片没扫到路障（已见 %d 类对象）"
                 % (zid, len(hit.get("seen") or {})))
    return False


def stage_scout(ctx, seconds):
    """阶段 2：两辆运输车驶向各自泊位模糊区，边走边记录见到的所有对象
    （Intersection_Center 只要进 30 m 视野就会被记下来，用来定位中央路口）。"""
    seen = {}
    targets = []
    ts = to_plain(ctx.api.get_task_state())
    for o in ts.get("shared_task_objects") or []:
        if o.get("object_type") != "destination":
            continue
        unc = o.get("uncertainty_region") or {}
        mn, mx = unc.get("minimum") or {}, unc.get("maximum") or {}
        if not mn or not mx:
            continue
        targets.append((o.get("object_id"),
                        (0.5 * (mn["x"] + mx["x"]), 0.5 * (mn["y"] + mx["y"]))))
    targets.sort(key=lambda t: 0 if "A" in t[0] else 1)
    pairs = list(zip(["Car_A", "Car_B"], [t[1] for t in targets]))
    ctx.note("[scout] 目标: %s" % ", ".join("%s->(%.0f,%.0f)" % (c, p[0], p[1])
                                            for c, p in pairs))

    def poll(pos, speed, ob, _car=None):
        for it in (ob.get("visible_objects") or []):
            oid = str(it.get("object_id") or "")
            if oid in seen:
                continue
            seen[oid] = it
            ctx.note("    [seen:%s] %-20s %-12s d=%.0f pos=%s"
                     % (_car, oid, it.get("object_type"),
                        it.get("distance_m") or -1,
                        json.dumps(it.get("position") or {}, ensure_ascii=False)[:64]))
        return None

    for car, goal in pairs:
        if time.time() - ctx.t0 > seconds:
            break
        ctx.note("[scout] %s -> (%.0f,%.0f) 观察泊位" % (car, goal[0], goal[1]))
        drive_to(ctx, car, goal, seconds, stop_xy=goal, stop_m=9.0,
                 poll=lambda p, s, ob, _c=car: poll(p, s, ob, _c))
        hold_still(ctx, car, seconds=1.5)
        ts2 = to_plain(ctx.api.get_task_state())
        ctx.note("[scout] unlocked=%s/%s"
                 % (ts2.get("unlocked_destination_count"),
                    ts2.get("total_destination_count")))
        for o in ts2.get("shared_task_objects") or []:
            if o.get("object_type") not in ("destination", "roadblock"):
                continue
            ctx.note("[scout]   %-14s %-12s %s pos=%s"
                     % (o.get("object_id"), o.get("object_type"),
                        o.get("information_level"),
                        json.dumps(o.get("position") or {}, ensure_ascii=False)[:60]))
    i = 0
    for oid, it in seen.items():
        if "Intersection" in oid or it.get("object_type") in ("intersection",):
            ctx.note("[scout] *路口* %s pos=%s d=%.1f"
                     % (oid, json.dumps(it.get("position") or {},
                                        ensure_ascii=False)[:80],
                        it.get("distance_m") or -1))
    dump_state(ctx, "after_scout")
    return True


def stage_probe(ctx, seconds):
    """阶段 P：沿东西向服务路（y≈-46）试探中央路口位置。
    只做两件事：①把 30 m 视野里出现的所有对象类型打出来；
    ②每隔 8 m 试一次 request_entry，从 message 里读出“在不在入口车道/多远”。"""
    api = ctx.api
    ctx.note("[probe] Car_A -> (660,-46) 走廊")
    drive_to(ctx, "Car_A", (660.0, -46.2), seconds, stop_xy=(660.0, -46.2), stop_m=2.0)
    xs = [660, 668, 676, 684, 692, 700, 708, 716, 724, 732, 740]
    for x in xs:
        if time.time() - ctx.t0 > seconds:
            break
        pose = pose_of(api, "Car_A")
        ctx.note("[probe] 现在 (%.1f,%.1f) yaw=%.0f v=%.2f"
                 % (pose[0][0], pose[0][1], pose[1], pose[2]))
        ob = to_plain(api.get_observation("Car_A"))
        names = []
        for it in (ob.get("visible_objects") or []):
            names.append("%s(%s,%.0fm)" % (it.get("object_id"), it.get("object_type"),
                                           it.get("distance_m") or -1))
        ctx.note("[probe]   vis: %s" % (", ".join(names) if names else "空"))
        ts = to_plain(api.get_task_state())
        ctx.note("[probe]   owner=%r phase=%s" % (ts.get("intersection_owner"), ts.get("phase")))
        try:
            r = api.interact("Car_A", "Intersection_Center", "request_entry")
            ctx.note("[probe]   request_entry granted=%s msg=%s" %
                     (getattr(r, "granted", None), getattr(r, "message", r)))
        except Exception as exc:
            ctx.note("[probe]   request_entry 异常: %s: %s" % (type(exc).__name__, exc))
        drive_to(ctx, "Car_A", (x + 8.0, -46.2), seconds,
                 stop_xy=(x + 8.0, -46.2), stop_m=1.5, v_cap=3.2)
    dump_state(ctx, "after_probe")
    return True


def _report(ctx, carrier, name, verbose=False):
    """对目标 name（Berth_A/Berth_B/Obstacle_x）提交一次观测凭证。"""
    ob = to_plain(ctx.api.get_observation(carrier))
    if verbose:
        vis = [str(v.get("object_id")) for v in (ob.get("visible_objects") or [])]
        ctx.note("[obs] %s visible=%s" % (carrier, vis[:8]))
        if not any(str(v.get("object_id")) == name for v in (ob.get("visible_objects") or [])):
            return False
    for it in (ob.get("visible_objects") or []):
        if str(it.get("object_id") or "") != name:
            continue
        tok = it.get("observation_token") or ""
        _p, _yw, _sp = pose_of(ctx.api, carrier)
        ctx.note("[obs] %s 看到 %s d=%.1f tok=%s yaw=%.0f @(%.1f,%.1f)"
                 % (carrier, name, it.get("distance_m") or -1, tok[:12],
                    _yw, _p[0], _p[1]))
        if not tok:
            ctx.note("[obs] %s 看到 %s 但无凭证" % (carrier, name))
            return False
        try:
            r = ctx.api.interact(carrier, name, "report_observation",
                                 {"observation_token": tok})
        except Exception as exc:
            ctx.note("[obs] 异常 %s: %s" % (type(exc).__name__, exc))
            return False
        ctx.note("[obs] %s report -> granted=%s msg=%s"
                 % (carrier, getattr(r, "granted", None), getattr(r, "message", "")))
        return bool(getattr(r, "granted", False))
    return False



def corridor_run(ctx, vid, x_c, y_end, v_tgt=1.2, poll=None, seconds=620.0):
    """闸口走廊专用控制器：从当前位置平滑归到中心线 x=x_c，
    然后一直直行到 y=y_end。路径从车的实际位置起算，避免起点在车后导致转头。
    """
    pos, _yaw, _v = pose_of(ctx.api, vid)
    pts = [(pos[0], pos[1]), (x_c, y_end)]
    ctx.note("[corr] %s (%.1f,%.1f) -> x=%.2f -> y=%.1f @%.1f m/s"
             % (vid, pos[0], pos[1], x_c, y_end, v_tgt))
    return drive_polyline(ctx, vid, pts, seconds, v_cap=v_tgt, poll=poll)


def stage_cross(ctx, seconds):
    """运输车通行中央路口：从南侧进车道向北穿过（m17 实测可行方向）。"""
    note = ctx.note
    api = ctx.api
    note("=== 阶段 3：两辆运输车通行中央路口 ===")
    # 南侧进车道（实测：x≈729 向北行驶能穿过）
    south_in = [(728.8, -58.0), (728.8, -50.0), (728.8, -44.0), (728.8, -42.0)]
    # 现场真相（static_objects）：路口是一条宽 5.0 m 的南北通道，两侧各有
    # Port_PassageBarrier（x=726.26 与 x=731.32，z=90°，L=8.8）。车宽 2.45 m，必须走正中 x=728.79，
    # 偏 1.3 m 就贴墙卡死（m29/m30/m31 都卡在 x=730.0）。
    north_out = [(728.8, -34.0), (728.8, -28.0), (728.8, -20.0), (728.8, -12.0),
                 (728.8, -4.0), (728.8, 6.0)]
    # 地图显示北侧唯一延续车道在 x=734.0（y 38.9~62.8）；
    # y≈4.7 是东西向的 V_MAIN（东段 x>=743.3 / 西段 x<=717.4）。
    cands = [(734.0, 40.0), (734.0, 50.0), (745.0, 4.7), (730.9, 30.0),
             (730.9, 20.0), (728.8, 4.0)]
    for car in ("Car_A", "Car_B"):
        if time.time() - ctx.t0 > seconds:
            return
        pos0, _y, _v = pose_of(ctx.api, car)
        detour = nav_path(ctx, pos0, south_in[-1])
        ctx.note("[cross] %s 栅格规划 %d 点 → (%.1f,%.1f)"
                 % (car, len(detour), south_in[-1][0], south_in[-1][1]))
        drive_polyline(ctx, car, detour, seconds, v_cap=4.0)  # grid route
        granted = [False]

        def poll(pos, speed, _a=None):
            if granted[0] or speed < 0.8:
                return None
            try:
                r = to_plain(api.interact(car, "Intersection_Center", "request_entry"))
            except Exception as e:
                return None
            msg = str(r.get("message", ""))
            if r.get("granted"):
                granted[0] = True
                note("[cross] %s GRANTED @(%.1f,%.1f) v=%.2f -> %s"
                     % (car, pos[0], pos[1], speed, msg))
                return "granted"
            if "eligible" not in msg and "not_available" not in msg:
                note("[cross] %s request -> %s" % (car, msg))
            return None

        corridor_run(ctx, car, 728.79, 8.0, v_tgt=1.2, poll=poll, seconds=seconds)
        if not granted[0]:
            note("[cross] %s 未获批，跳过" % car)
            continue
        # 获批后直行往北，边走边在每个 5 m 标高上试 release，用来定位“已连接出口”
        tried = [None]

        def relpoll(pos, speed, _a=None):
            y = pos[1]
            mark = int(math.floor(y / 5.0))
            if tried[0] == mark:
                return None
            tried[0] = mark
            try:
                r = to_plain(api.interact(car, "Intersection_Center", "release"))
            except Exception as e:
                r = {"granted": False, "message": "ERR %s" % e}
            note("[rel] %s @(%.1f,%.1f) mark=%d -> %s | %s"
                 % (car, pos[0], pos[1], mark * 5, r.get("granted"), r.get("message")))
            return "released" if r.get("granted") else None

        # 闸口两墙在 x=726.26/731.32，中心线 x=728.79；车宽 2.45 只能走中心线
        # 先回到闸口中心线（x=728.79）再直行往北，避开两端 3.69 m 的斜段墙
        gate_line = [(728.79, -31.0), (728.79, -24.0), (728.79, -15.0), (728.79, -6.0),
                     (728.79, 4.0), (728.79, 20.0), (728.79, 46.0)]
        got_rel = drive_polyline(ctx, car, gate_line, seconds, v_cap=1.8, poll=relpoll)
        ok = (got_rel == "released")
        if not ok:
            for pt in [(734.0, 42.0), (745.0, 4.7), (715.0, 4.7)]:
                drive_to(ctx, car, pt, seconds, stop_xy=pt, stop_m=3.0)
                for _ in range(2):
                    try:
                        r = to_plain(api.interact(car, "Intersection_Center", "release"))
                    except Exception as e:
                        r = {"granted": False, "message": "ERR %s" % e}
                    note("[cross] %s release@(%.1f,%.1f) -> %s | %s"
                         % (car, pt[0], pt[1], r.get("granted"), r.get("message")))
                    if r.get("granted"):
                        ok = True
                        break
                    time.sleep(0.6)
                if ok:
                    break
        note("[cross] %s 通行 %s" % (car, "成功" if ok else "失败"))


def reverse_turn(ctx, vid, seconds, max_s=9.0, thresh_deg=70.0):
    """倒车搬头：朝向差太大时用 R 档把车折过来（车头姿不变反而更容易转）。"""
    note = ctx.note
    t_start = time.time()
    pos, yaw, speed = pose_of(ctx.api, vid)
    note("    %s [rev] @(%.1f,%.1f) yaw=%.0f" % (vid, pos[0], pos[1], yaw))
    while time.time() - t_start < max_s and time.time() - ctx.t0 < seconds:
        pos, yaw, speed = pose_of(ctx.api, vid)
        drv = ctx.rev_goal.get(vid)
        if drv is None:
            break
        gx, gy = drv
        want = math.degrees(math.atan2(gy - pos[1], gx - pos[0]))
        err = (want - yaw + 540.0) % 360.0 - 180.0
        if abs(err) < thresh_deg:
            break
        steer = max(-0.55, min(0.55, -0.02 * err))
        veh = VehicleControl(throttle=0.30, brake=0.0, steering=steer)
        try:
            veh.gear = GearMode.REVERSE
        except Exception:
            pass
        ctrls = {v: VehicleControl(throttle=0.0, brake=0.35, steering=0.0) for v in ctx.vids}
        ctrls[vid] = veh
        try:
            ctx.api.set_vehicle_controls(ctrls)
        except Exception as exc:
            note("    %s [rev] set failed %s" % (vid, exc))
            break
        time.sleep(0.1)
    ctrls = {v: VehicleControl(throttle=0.0, brake=0.45, steering=0.0) for v in ctx.vids}
    try:
        ctx.api.set_vehicle_controls(ctrls)
    except Exception:
        pass
    time.sleep(0.6)


def drive_multi(ctx, jobs, seconds, v_cap=4.2, stall_s=3.5):
    """多车并行：jobs = [(vid, pts, v_cap_or_None, stop_m)]，同时驾驶多辆车。"""
    note = ctx.note
    routes = []
    for vid, pts, cap, stop_m in jobs:
        # 从车的实时位置接上去（否则起点在车后方会往回开）
        try:
            _pos = pose_of(ctx.api, vid)[0]
            pts = [_pos] + [q for q in pts if math.hypot(q[0] - _pos[0], q[1] - _pos[1]) > 3.0]
            note("    %s 起点(%.1f,%.1f) -> %d wp" % (vid, _pos[0], _pos[1], len(pts)))
        except Exception as exc:
            note("    %s 取位姿失败 %s" % (vid, exc))
        if len(pts) < 2:
            pts = [pts[0], (pts[0][0] + 0.1, pts[0][1])]
        try:
            _p, _yaw, _v = pose_of(ctx.api, vid)
            _want = math.degrees(math.atan2(pts[1][1] - _p[1], pts[1][0] - _p[0]))
            _err = (_want - _yaw + 540.0) % 360.0 - 180.0
            if abs(_err) > 110.0:
                ctx.rev_goal[vid] = pts[1]
                reverse_turn(ctx, vid, seconds)
                ctx.rev_goal.pop(vid, None)
        except Exception as exc:
            note("    %s [rev] skip %s" % (vid, exc))
        dense = []
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            n = max(1, int(seg / 2.0))
            for k in range(n):
                dense.append((ax + (bx - ax) * k / n, ay + (by - ay) * k / n))
        dense.append(tuple(pts[-1]))
        cap_v = cap or v_cap
        route = fog_route.Route(dense, [cap_v] * len(dense))
        routes.append({"vid": vid, "end": dense[-1], "rev": None, "drv": LaneDriver(route, ctx.wb, v_cap=cap_v),
                       "cap": cap_v, "stop_m": stop_m, "stall": None, "kick": None,
                       "done": False, "kick_dir": 0.0})
    while time.time() - ctx.t0 < seconds:
        ctrls = {}
        alive = 0
        for r in routes:
            vid = r["vid"]
            pos, yaw, speed = pose_of(ctx.api, vid)
            if r["done"]:
                ctrls[vid] = VehicleControl(throttle=0.0, brake=0.45, steering=0.0)
                continue
            now0 = time.time()
            if r["kick"] is not None:
                if now0 - r["kick"] < 1.6:
                    if r.get("rev") is not None and now0 - r.get("rev", 0.0) < 0.9:
                        veh = VehicleControl(throttle=0.35, brake=0.0, steering=-r["kick_dir"] or 0.3)
                        try:
                            veh.gear = GearMode.REVERSE
                        except Exception:
                            pass
                        ctrls[vid] = veh
                    else:
                        ctrls[vid] = VehicleControl(throttle=0.60, brake=0.0, steering=r["kick_dir"])
                    alive += 1
                    continue
                r["kick"] = None
                r["stall"] = now0
            thr, brk, steer = r["drv"].step(pos, yaw, speed, 0.1)
            if abs(speed) < 0.15:
                if r["stall"] is None:
                    r["stall"] = now0
                elif now0 - r["stall"] > stall_s:
                    r["kick"] = now0
                    r["kick_n"] = r.get("kick_n", 0) + 1
                    if r["kick_n"] % 3 == 0:
                        # 轮次到了就倒一把搬头
                        r["kick_dir"] = 0.0
                        r["rev"] = now0
                    else:
                        r["kick_dir"] = 0.25 if (r["kick_n"] % 2 == 1) else -0.25
                    note("    %s [stall] @(%.1f,%.1f) 踢油门" % (vid, pos[0], pos[1]))
                    ctrls[vid] = VehicleControl(throttle=0.60, brake=0.0, steering=r["kick_dir"])
                    alive += 1
                    continue
            else:
                r["stall"] = None
            if r["drv"].done:
                end_xy = r["end"]
                d_end = math.hypot(pos[0] - end_xy[0], pos[1] - end_xy[1])
                if d_end <= r["stop_m"] + 0.5:
                    r["done"] = True
                    ctrls[vid] = VehicleControl(throttle=0.0, brake=0.5, steering=0.0)
                    note("    %s 到达 (%.1f,%.1f) d_end=%.1f" % (vid, pos[0], pos[1], d_end))
                    continue
            ctrls[vid] = VehicleControl(throttle=thr, brake=brk, steering=steer)
            alive += 1
        for vid in ctx.vids:
            if vid not in ctrls:
                ctrls[vid] = VehicleControl(throttle=0.0, brake=0.35, steering=0.0)
        ctx.api.set_vehicle_controls(ctrls)
        if alive == 0:
            break
        time.sleep(0.1)
    for vid in ctx.vids:
        pass


def precise_point(ctx, name, fallback):
    """lv3 后任务对象会给出精确 position；没有就用 fallback（矩形中心）。"""
    ts = to_plain(ctx.api.get_task_state())
    for it in (ts.get("shared_task_objects") or []):
        if str(it.get("object_id") or "") != name:
            continue
        pos = it.get("position") or {}
        try:
            x = float(pos.get("x") or 0.0)
            y = float(pos.get("y") or 0.0)
        except Exception:
            return fallback
        if abs(x) > 1e-3 or abs(y) > 1e-3:
            return (x, y)
    return fallback


def creep_into(ctx, vid, target, seconds, tol=1.6, rounds=5):
    """分段蜷行到目标点附近（tol 内）。

    ⚠️ 已经进到 2 m 以内就**别再纠偏**：19:28 那局 Car_A 在 d=0.97 m 时还去纠，
    1 m 的目标点落在车后 → 纯跟踪直接掉头，8 s 后甩到 (622.9,−61.8) 撞进
    EastBulkCargoWarehouse。位姿精度交给后面的 `park_and_hold`（它只要求停稳）。
    """
    for k in range(rounds):
        pos = pose_of(ctx.api, vid)[0]
        d = math.hypot(pos[0] - target[0], pos[1] - target[1])
        ctx.note("[creep] %s d=%.2f (%.1f,%.1f)" % (vid, d, pos[0], pos[1]))
        if d <= max(tol, 2.0):
            if d > tol:
                ctx.note("[creep] %s 已在 %.1f m 内，不再纠偏（避免掉头甩出去）" % (vid, d))
            return True
        if time.time() - ctx.t0 > seconds - 5.0:
            return False
        try:
            drive_polyline(ctx, vid, [(pos[0], pos[1]), (target[0], target[1])],
                           seconds, v_cap=0.7)
        except Exception as exc:                               # noqa: BLE001
            ctx.note("[creep] 异常 %s" % exc)
            return False
    return False


def park_and_hold(ctx, vid, seconds, need_s=2.0, eps=0.05):
    """挂 P 档 + 刹死，直到连续 need_s 秒 |v| < eps（交付前的“稳定位姿”硬要求）。"""
    t_end = min(time.time() + 25.0, ctx.t0 + seconds)
    ok_since = None
    while time.time() < t_end:
        veh = VehicleControl(throttle=0.0, brake=1.0, steering=0.0)
        try:
            veh.gear = GearMode.PARKING
        except Exception:
            pass
        hb = getattr(ctx, "hold_brk", None) or {}
        ctrls = {v: VehicleControl(throttle=0.0, brake=max(0.5, hb.get(v, 0.0)),
                                   steering=0.0) for v in ctx.vids}
        ctrls[vid] = veh
        try:
            ctx.api.set_vehicle_controls(ctrls)
        except Exception:
            pass
        time.sleep(0.1)
        _pos, _yaw, sp = pose_of(ctx.api, vid)
        if abs(sp) < eps:
            if ok_since is None:
                ok_since = time.time()
            elif time.time() - ok_since >= need_s:
                ctxx = "hold %.1fs |v|<%.2f" % (need_s, eps)
                ctx.note("[park] %s %s" % (vid, ctxx))
                return True
        else:
            ok_since = None
    ctx.note("[park] %s 未能稳定" % vid)
    return False


def level_of(ctx, name):
    """读任务对象（泊位等）的 information_level，返回 0/1/2/3。"""
    ts = to_plain(ctx.api.get_task_state())
    for it in (ts.get("shared_task_objects") or []):
        if str(it.get("object_id") or "") == name:
            lv = str(it.get("information_level") or "")
            if "lv3" in lv:
                return 3
            if "lv2" in lv:
                return 2
            if "lv1" in lv:
                return 1
            return 0
    return 0



def face_target(ctx, vid, tgt_xy, seconds, gap=4.5):
    """\u628a\u8f66\u5934\u8f6c\u5411\u76ee\u6807\uff08\u5f00\u5230\u6cbf\u7740\u6307\u5411\u76ee\u6807\u65b9\u5411 4.5 m \u7684\u70b9\uff09\u3002"""
    try:
        pv, yaw_v = pose_of(api, vid)[:2]
        dx, dy = tgt_xy[0] - pv[0], tgt_xy[1] - pv[1]
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return
        q = (pv[0] + dx / n * gap, pv[1] + dy / n * gap)
        bear = math.degrees(math.atan2(dy, dx))
        err = abs((bear - yaw_v + 180.0) % 360.0 - 180.0)
        note("[face] %s yaw=%.0f \u76ee\u6807\u65b9\u4f4d=%.0f \u8bef\u5dee=%.0f" % (vid, yaw_v, bear, err))
        if err > 35.0:
            drive_polyline(ctx, vid, [q], seconds, v_cap=1.2)
    except Exception as exc:
        note("[face] %s \u5f02\u5e38 %s" % (vid, exc))


def back_out(ctx, vid, seconds, dist_m=8.0, v_target=1.3):
    """切 R 档倒车直退 dist_m 米（出泊位用）。"""
    api = ctx.api
    p0 = pose_of(api, vid)[0]
    while time.time() - ctx.t0 < seconds:
        p, _yaw, spd = pose_of(api, vid)
        moved = math.hypot(p[0] - p0[0], p[1] - p0[1])
        if moved >= dist_m:
            break
        if abs(spd) > v_target:
            ctl = VehicleControl(throttle=0.0, brake=0.30, steering=0.0, gear=GearMode.REVERSE)
        else:
            ctl = VehicleControl(throttle=0.10, brake=0.0, steering=0.0, gear=GearMode.REVERSE)
        try:
            api.set_vehicle_control(ctl, vehicle_id=vid)
        except Exception:
            break
        time.sleep(0.1)
    try:
        api.set_vehicle_control(VehicleControl(throttle=0.0, brake=0.6, gear=GearMode.PARKING), vehicle_id=vid)
    except Exception:
        pass
    pp = pose_of(api, vid)[0]
    ctx.note("[deliver] %s 倒车出位到 (%.1f,%.1f)" % (vid, pp[0], pp[1]))


def stage_deliver(ctx, seconds):
    """阶段 4：泊位交付。A 先（Car_A 交、Car_C 陪同观测），
    剩余时间再做 B（Car_B 交、Car_C 陪同观测）。"""
    api = ctx.api
    note = ctx.note

    # Car_D 从起点往东接上 x=638.4 的南行货运走廊，直到泊位 A 南侧
    west_cc = [(638.4, 120.8), (638.4, 60.0), (638.4, -20.0), (638.4, -48.0), (643.0, -52.0),
               (641.0, -59.0)]
    # 出闸口后车头朝北：先往北上 V_MAIN(y≈4.7)，再左转西，再左转南走 x=714.6，避开 90° 急弯
    west_ca = [(728.8, -9.9), (728.8, 3.0), (724.0, 4.7), (718.0, 4.7), (714.6, 0.0),
               (714.6, -20.0), (714.6, -33.2), (712.0, -46.0), (704.2, -46.2), (680.0, -46.2),
               (655.0, -46.2), (648.7, -46.2), (644.0, -50.0), (638.4, -53.2), (636.0, -58.0),
               (633.5, -62.0)]

    # 4.1 两车并行：Car_C 从灯塔、Car_A 从闸口北口，同时赶去泊位 A
    try:
        drive_multi(ctx, [("Car_D", west_cc, 4.2, 3.0), ("Car_A", west_ca, 1.8, 4.0)], seconds)
    except Exception as exc:
        note("[deliver] 并行赶赴 A 异常 %s: %s" % (type(exc).__name__, exc))

    # 4.2 等两车都进到观测距离（<=25 m）再观测
    bx, by = 630.97, -65.72
    for _ in range(60):
        try:
            pa = pose_of(ctx.api, "Car_A")[0]
            pd = pose_of(ctx.api, "Car_D")[0]
        except Exception:
            break
        da = math.hypot(pa[0] - bx, pa[1] - by)
        dd = math.hypot(pd[0] - bx, pd[1] - by)
        if da <= 25.0 and dd <= 25.0:
            note("[deliver] 两车就位 A: Car_A d=%.1f Car_D d=%.1f 间距=%.1f"
                 % (da, dd, math.hypot(pa[0] - pd[0], pa[1] - pd[1])))
            break
        time.sleep(2.0)
    for k in range(8):
        a = _report(ctx, "Car_A", "Berth_A")
        c = _report(ctx, "Car_D", "Berth_A", verbose=True)
        lv = level_of(ctx, "Berth_A")
        ts = to_plain(api.get_task_state())
        note("[deliver] Berth_A obs A=%s D=%s level=lv%s unlocked=%s/%s"
             % (a, c, lv, ts.get("unlocked_destination_count"), ts.get("total_destination_count")))
        if lv >= 3 or ts.get("unlocked_destination_count"):
            break
        # 没到 lv3：两车都重新对准泊位（车头指向它）后再报
        bx, by = 630.97, -65.72
        for vid, standoff in (("Car_A", 6.0), ("Car_D", 8.0)):
            try:
                pv, yaw_v = pose_of(api, vid)[:2]
                _d = math.hypot(pv[0] - bx, pv[1] - by)
                _bear = math.degrees(math.atan2(by - pv[1], bx - pv[0]))
                _err = abs((_bear - yaw_v + 180.0) % 360.0 - 180.0)
                if _d < 28.0 and _err < 55.0:
                    note("[deliver] %s 已在 %.0f m/误差 %.0f°，不再就位" % (vid, _d, _err))
                    continue
                dx, dy = pv[0] - bx, pv[1] - by
                r = math.hypot(dx, dy) or 1.0
                ux, uy = dx / r, dy / r
                if vid == "Car_D" and abs(bx - 834.47) < 1.0:
                    g1 = (838.5, -52.0)
                    g2 = (838.5, -56.0)
                else:
                    g1 = (bx + ux * (standoff + 3.0), by + uy * (standoff + 3.0))
                    g2 = (bx + ux * standoff, by + uy * standoff)
                note("[deliver] %s 就位 (%.1f,%.1f)->(%.1f,%.1f)" % (vid, g1[0], g1[1], g2[0], g2[1]))
                drive_polyline(ctx, vid, [g1, g2], seconds,
                               v_cap=(1.4 if abs(bx - 834.47) < 1.0 else 2.2))
            except Exception as exc:
                note("[deliver] %s 就位异常 %s" % (vid, exc))
        try:
            hold_still(ctx, "Car_D", seconds=0.8)
        except Exception:
            pass
        pa = pose_of(api, "Car_A")[0]
        pd = pose_of(api, "Car_D")[0]
        note("[deliver] 视点间距=%.1f" % math.hypot(pa[0]-pd[0], pa[1]-pd[1]))

    tgt_a = precise_point(ctx, "Berth_A", (630.97, -65.72))
    note("[deliver] Berth_A 精确点 (%.2f,%.2f)" % (tgt_a[0], tgt_a[1]))
    try:
        pos0 = pose_of(api, "Car_A")[0]
        q = nav_path(ctx, pos0, tgt_a, inflate=2.0)
        note("[deliver] 栅格路径 -> %d wp" % len(q))
        drive_polyline(ctx, "Car_A", q, seconds, v_cap=1.6)
    except Exception as exc:
        note("[deliver] 栅格入位异常 %s" % exc)
    try:
        creep_into(ctx, "Car_A", tgt_a, seconds, tol=0.9)
    except Exception as exc:
        note("[deliver] 进泊位 A 异常 %s" % exc)
    park_and_hold(ctx, "Car_A", seconds)
    got_a = False
    for attempt in range(4):
        try:
            r = api.interact("Car_A", "Berth_A", "deliver")
        except Exception as exc:
            note("[deliver] deliver A 异常 %s: %s" % (type(exc).__name__, exc))
            break
        note("[deliver] Car_A deliver A(%d) granted=%s msg=%s"
             % (attempt + 1, getattr(r, "granted", None), getattr(r, "message", "")[:90]))
        if getattr(r, "granted", False):
            got_a = True
            break
        park_and_hold(ctx, "Car_A", seconds, need_s=1.2)
    dump_state(ctx, "after_deliver_A")
    if got_a:
        try:
            back_out(ctx, "Car_A", seconds, dist_m=9.0)
        except Exception as exc:
            note("[deliver] Car_A 倒车出位异常 %s" % exc)

    if not args_deliver_b():
        return True

    # Car_B 在闸口北口，先往北开出 6 m 互斥区再转向东
    try:
        drive_polyline(ctx, "Car_B", [(728.8, -9.9), (728.8, 6.0), (731.5, 8.0)], seconds, v_cap=2.0)
    except Exception as exc:
        note("[deliver] Car_B 出闸口异常 %s" % exc)

    # 4.4 泊位 B：Car_B 从闸口北口经绕行去 B，Car_C 同时从 A 出发
    # Car_B 直接进车位（它是交付车）；Car_D 只停在车位正北的走廊上，避免堆在 Car_B 后面
    north_b = [(728.8, 6.0), (743.3, 4.7), (790.0, 4.7), (790.0, -30.0),
               (812.0, -40.0), (830.0, -56.0), (834.0, -63.8)]
    from_a = [(636.0, -58.5), (638.4, -40.0), (638.4, 4.7), (700.0, 4.7), (790.0, 4.7),
              (800.0, -10.0), (812.0, -30.0), (838.5, -49.5), (838.5, -57.0)]
    try:
        drive_multi(ctx, [("Car_B", north_b, 4.2, 3.0), ("Car_D", from_a, 4.2, 3.0)], seconds)
        face_target(ctx, "Car_B", (834.47, -65.72), seconds)
        face_target(ctx, "Car_D", (834.47, -65.72), seconds)
    except Exception as exc:
        note("[deliver] 并行赶赴 B 异常 %s: %s" % (type(exc).__name__, exc))
    for k in range(8):
        b = _report(ctx, "Car_B", "Berth_B")
        c = _report(ctx, "Car_D", "Berth_B", verbose=True)
        lv = level_of(ctx, "Berth_B")
        ts = to_plain(api.get_task_state())
        note("[deliver] Berth_B obs B=%s D=%s level=lv%s unlocked=%s/%s"
             % (b, c, lv, ts.get("unlocked_destination_count"), ts.get("total_destination_count")))
        if lv >= 3:
            break
        if k <= 0 and abs(bx - 834.47) < 1.0:
            try:
                hold_still(ctx, "Car_B", seconds=1.2)
                hold_still(ctx, "Car_A", seconds=1.2)
            except Exception:
                pass
            continue
        bx, by = 834.47, -65.72
        for vid, standoff in (("Car_B", 6.0), ("Car_D", 8.5)):
            _pv0 = pose_of(api, vid)[0]
            if math.hypot(_pv0[0] - bx, _pv0[1] - by) > 35.0:
                note("[deliver] %s 还远（%.0f m），本轮不就位"
                     % (vid, math.hypot(_pv0[0] - bx, _pv0[1] - by)))
                continue
            try:
                pv = pose_of(api, vid)[0]
                dx, dy = pv[0] - bx, pv[1] - by
                r = math.hypot(dx, dy) or 1.0
                ux, uy = dx / r, dy / r
                g1 = (bx + ux * (standoff + 3.0), by + uy * (standoff + 3.0))
                g2 = (bx + ux * standoff, by + uy * standoff)
                note("[deliver] %s 就位 B (%.1f,%.1f)->(%.1f,%.1f)" % (vid, g1[0], g1[1], g2[0], g2[1]))
                drive_polyline(ctx, vid, [g1, g2], seconds, v_cap=1.4)
            except Exception as exc:
                note("[deliver] %s 就位 B 异常 %s" % (vid, exc))
        try:
            hold_still(ctx, "Car_A", seconds=0.8)
        except Exception:
            pass
    tgt_b = precise_point(ctx, "Berth_B", (834.47, -65.72))
    note("[deliver] Berth_B 精确点 (%.2f,%.2f)" % (tgt_b[0], tgt_b[1]))
    try:
        posb = pose_of(api, "Car_B")[0]
        qb = nav_path(ctx, posb, tgt_b, inflate=2.0)
        note("[deliver] B 栅格路径 -> %d wp" % len(qb))
        drive_polyline(ctx, "Car_B", qb, seconds, v_cap=1.6)
    except Exception as exc:
        note("[deliver] B 栅格入位异常 %s" % exc)
    try:
        creep_into(ctx, "Car_B", tgt_b, seconds, tol=0.9)
    except Exception as exc:
        note("[deliver] 进泊位 B 异常 %s" % exc)
    park_and_hold(ctx, "Car_B", seconds, need_s=2.5)
    for attempt in range(6):
        try:
            r = api.interact("Car_B", "Berth_B", "deliver")
        except Exception as exc:
            note("[deliver] deliver B 异常 %s: %s" % (type(exc).__name__, exc))
            break
        note("[deliver] Car_B deliver B(%d) granted=%s msg=%s"
             % (attempt + 1, getattr(r, "granted", None), getattr(r, "message", "")[:90]))
        if getattr(r, "granted", False):
            break
        hold_still(ctx, "Car_B", seconds=1.5)
    dump_state(ctx, "after_deliver_B")
    return got_a


def args_deliver_b():
    import os as _os
    return _os.environ.get("FOG_DELIVER_B", "0") == "1"


def dump_state(ctx, tag=""):
    ts = to_plain(ctx.api.get_task_state())
    ob = to_plain(ctx.api.get_observation("Car_C"))
    path = os.path.join(ctx.out, "state_%s.json" % (tag or "final"))
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"task": ts, "obs": ob}, fh, ensure_ascii=False, indent=1)
    ctx.note("phase=%s signals=%s/%s unlocked=%s/%s roadblock=%s owner=%s"
             % (ts.get("phase"), ts.get("verified_signal_count"),
                ts.get("total_signal_count"), ts.get("unlocked_destination_count"),
                ts.get("total_destination_count"), ts.get("roadblock_active"),
                ts.get("intersection_owner")))
    for o in ts.get("shared_task_objects") or []:
        ctx.note("  obj %-12s %-20s %s" % (o.get("object_id"), o.get("object_type"),
                                           o.get("information_level")))
    return ts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=560.0)
    ap.add_argument("--probe", type=int, default=0)
    ap.add_argument("--cross", type=int, default=1)
    ap.add_argument("--deliver", type=int, default=0)
    ap.add_argument("--probe2", type=int, default=0)
    ap.add_argument("--probe3", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    api = connect_retry()
    log = open(os.path.join(args.out, "run.log"), "w", encoding="utf-8")
    t0 = time.time()

    def note(msg):
        line = "[%6.1f] %s" % (time.time() - t0, msg)
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

    vids = []
    try:
        static = to_plain(api.get_scene_static_data())
        graph = RoadGraph(static)
        specs = {v["vehicle_id"]: v for v in static.get("vehicles", [])}
        vids = list(api.get_vehicle_ids())
        wb = WHEELBASE_RATIO * float(specs["Car_C"].get("length_m") or 4.81)
        ctx = Ctx(api, vids, graph, wb, args.out, t0, note)
        note("vehicles=%s" % vids)

        note("=== 阶段 0：三个现场信号 ===")
        got = stage_signals(ctx, args.seconds)
        note("signals ok=%s" % json.dumps(got, ensure_ascii=False))
        dump_state(ctx, "after_signals")

        note("=== 阶段 1：路障扫描 ===")
        scanned = stage_scan(ctx, args.seconds)
        dump_state(ctx, "after_scan")

        if args.probe3:
            note("=== 阶段 P3：打印路口对象完整信息 ===")
            stage_probe3(ctx, args.seconds)
        elif args.probe2:
            note("=== 阶段 P2：中央通道逐步试申请/release ===")
            stage_probe2(ctx, args.seconds)
        elif args.probe:
            note("=== 阶段 P：沿 y=-46 走廊试探中央路口 ===")
            stage_probe(ctx, args.seconds)
        elif scanned:
            if args.cross:
                note("=== 阶段 3：两辆运输车通行中央路口 ===")
                stage_cross(ctx, args.seconds)
            if args.deliver:
                note("=== 阶段 4：泊位观测与交付 ===")
                stage_deliver(ctx, args.seconds)
        try:
            for cmd in ("get_score", "get_run_report", "get_task_state"):
                try:
                    raw = api._request({"command": cmd})
                except Exception as exc:                           # noqa: BLE001
                    note("[score] %s 失败 %s" % (cmd, exc))
                    continue
                note("[score] %s -> %s" % (cmd, json.dumps(raw, ensure_ascii=False)[:700]))
        except Exception as exc:                               # noqa: BLE001
            pass
    finally:
        try:
            api.set_vehicle_controls({v: VehicleControl(throttle=0.0, brake=0.6,
                                                        steering=0.0) for v in vids})
        except Exception:                              # noqa: BLE001
            pass
        log.close()
        try:
            api.close()
        except Exception:                              # noqa: BLE001
            pass


# ============================================================================
# 以下是我在 2026-10-29/30 迭代出来的执行层（覆盖上面原版同名函数）。
# 放在文件末尾、`if __name__ == "__main__": main()` 之前，靠"后定义覆盖先定义"生效。
# 关键改动：防撞层、卡住立刻倒车、信号弧线补读、出口 0.5 m 细扫、过闸后直接去泊位。
# ============================================================================
from fog_run3 import pose_of, KEY_RE, WHEELBASE_RATIO, MAX_STEER, clip, reset_pose
from fog_safety import Safety, SAFE_GAP, WARN_PATH_M


def _dyn_register(ctx, xy, r=8.0, tag=""):
    """把动态障碍（路障这类 static_objects 里没有的东西）记成圆形禁区。"""
    lst = getattr(ctx, "dyn_avoid", None)
    if lst is None:
        lst = ctx.dyn_avoid = []
    for (x, y, _rr) in lst:
        if math.hypot(x - xy[0], y - xy[1]) < 3.0:
            return False
    lst.append((float(xy[0]), float(xy[1]), float(r)))
    ctx.note("[avoid] 记入动态禁区 %s (%.1f,%.1f) r=%.0f m，之后所有路线绕开它"
             % (tag, xy[0], xy[1], r))
    return True


def _dyn_xy_of(ctx, name):
    """按名字找任务对象坐标（路障只在这里有精确位置）。"""
    try:
        ts = to_plain(ctx.api.get_task_state())
    except Exception:                                          # noqa: BLE001
        return None
    for o in ts.get("shared_task_objects") or []:
        if str(o.get("object_id") or "") != name:
            continue
        p = o.get("position") or {}
        if p.get("x") is not None:
            return (float(p["x"]), float(p["y"]))
        unc = o.get("uncertainty_region") or {}
        mn, mx = unc.get("minimum") or {}, unc.get("maximum") or {}
        if mn and mx:
            return (0.5 * (mn["x"] + mx["x"]), 0.5 * (mn["y"] + mx["y"]))
    return None


def sync_dyn_avoid(ctx, r=8.0):
    """任务状态里的路障全部记成禁区。扫描一结束就调用，后面每条路线都会绕。"""
    try:
        ts = to_plain(ctx.api.get_task_state())
    except Exception:                                          # noqa: BLE001
        return
    for o in ts.get("shared_task_objects") or []:
        if o.get("object_type") != "roadblock":
            continue
        p = o.get("position") or {}
        if p.get("x") is None:
            continue
        _dyn_register(ctx, (float(p["x"]), float(p["y"])), r,
                      str(o.get("object_id")))


def _ctx_poll_events(self):
    """把上一帧之后的事件读出来，违规立刻落日志（不读就不知道车为什么卡住）。"""
    try:
        evs = self.api.get_events(getattr(self, "ev_seq", 0))
    except Exception as exc:                               # noqa: BLE001
        self.note("[events] 读取失败 %s" % exc)
        return
    for e in evs:
        d = to_plain(e)
        seq = int(d.get("sequence") or 0)
        if seq > getattr(self, "ev_seq", 0):
            self.ev_seq = seq
        kind = d.get("event_type") or ""
        msg = str(d.get("message"))
        if kind in ("violation", "warning", "penalty"):
            self.note("[violation] #%s %s" % (seq, msg[:110]))
        # 撞到"实体障碍"= 那条路被堵了：把它记成禁区，别再往同一个地方撞
        if "与实体障碍" in msg:
            nm = msg.split("与实体障碍", 1)[1].split("发生碰撞", 1)[0].strip()
            seen = getattr(self, "_blk_seen", None)
            if seen is None:
                seen = self._blk_seen = set()
            if nm and nm not in seen:
                seen.add(nm)
                xy = _dyn_xy_of(self, nm)
                if xy is not None:
                    _dyn_register(self, xy, 8.0, nm)
                else:
                    self.note("[avoid] %s 是静态构件（已在栅格里），不用再记" % nm)


def _ctx_poll_throttled(self, every=2.0):
    now = time.time()
    if now - getattr(self, "_ev_t", 0.0) < every:
        return
    self._ev_t = now
    self.poll_events()


Ctx.poll_events = _ctx_poll_events
Ctx.poll_events_throttled = _ctx_poll_throttled


STOP_FAR = 4.5          # 读码要 ≤7 m，停 6.7 m 太勉强


def _dense(pts, step=2.0):
    out = []
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        n = max(1, int(seg / step))
        for k in range(n):
            out.append((ax + (bx - ax) * k / n, ay + (by - ay) * k / n))
    out.append(tuple(pts[-1]))
    return out


def _send(ctx, ctrls):
    """合并式下发：只覆盖当前线程"占用"的车，其余保留上一帧指令。"""
    for vid, veh in ctrls.items():
        ctx.ctl[vid] = veh
    try:
        ctx.api.set_vehicle_controls(dict(ctx.ctl))
    except Exception as exc:                                   # noqa: BLE001
        ctx.note("[ctl] 发送失败 %s" % exc)


def _ctrl(ctx, vid, thr=0.0, brk=0.0, steer=0.0, gear=None):
    veh = VehicleControl(throttle=thr, brake=brk, steering=steer)
    if gear is not None:
        veh.gear = gear
    ctx.ctl[vid] = veh


def _brake_others(ctx, keep):
    """刹住没在开的车。ctx.hold_brk 里登记过的车用更强的刹车：

    19:44 那局 Car_C 早该停住了，可 0.35 的刹车在缓坡上按住不，它慢慢溜到
    路障边上又蹭了 6 次（514 s 还在刷 collision）。
    """
    hold = getattr(ctx, "hold_brk", None) or {}
    for v in ctx.vids:
        if v == keep:
            continue
        _ctrl(ctx, v, 0.0, float(hold.get(v, 0.35)), 0.0)


def park_car(ctx, vid, brk=1.0):
    """把一辆车彻底停住并登记力度（之后 _brake_others 会一直用这个力度按住）。"""
    hold = getattr(ctx, "hold_brk", None)
    if hold is None:
        hold = ctx.hold_brk = {}
    hold[vid] = float(brk)
    _ctrl(ctx, vid, 0.0, float(brk), 0.0)
    _send(ctx, {})
    ctx.note("[park] %s 驻车（刹车 %.1f）" % (vid, brk))


def settle(ctx, brk=0.50, seconds=0.6):
    """踩刹车收尾：刹车从 0.12 逐帧爬上去。

    一帧 0→0.45/0.60 的刹车阶跃就够平台判一次 `[excessive_acceleration]`
    （17:38 那局 C 在 170/178 s、D 在 223/352 s 都是这么丢的）。
    """
    t_end = min(time.time() + seconds, ctx.t0 + 600.0)
    cur = 0.12
    hb = getattr(ctx, "hold_brk", None) or {}
    while time.time() < t_end:
        for v in ctx.vids:
            # 已登记驻车的车（Car_C）保持大力刹车，别被这里的斜坡覆盖 ——
            # 一覆盖它就从 1.0 掉到 0.12，车一晃一顿，一次一晃一条加速度违规。
            _ctrl(ctx, v, 0.0, max(cur, hb.get(v, 0.0)), 0.0)
        _send(ctx, {})
        cur = min(brk, cur + 0.10)
        time.sleep(0.1)


def unstick(ctx, vid, seconds, back_s=1.2, thr=0.30, steer=0.0):
    """车走不动了：先往反方向退一点（用户 29 日 23:07 定的规矩）。

    油门按 0.06/帧 爬上去、收尾只踩 0.45 —— 原来 0→0.30 一脚踏出去、收手时
    0→0.6 一脚刹住，每帧 Δv 都能顶到 5 m/s² 以上，这就是
    `[excessive_acceleration]` 的主要来源。
    """
    for v in ctx.vids:
        _ctrl(ctx, v, 0.0, 0.35 if v != vid else 0.0, 0.0)
    _send(ctx, {})
    t_end = min(time.time() + back_s, ctx.t0 + seconds)
    p0 = pose_of(ctx.api, vid)[0]
    cur = 0.06
    while time.time() < t_end:
        veh = VehicleControl(throttle=cur, brake=0.0, steering=steer)
        try:
            veh.gear = GearMode.REVERSE
        except Exception:                                      # noqa: BLE001
            pass
        ctx.ctl[vid] = veh
        _send(ctx, {})
        cur = min(thr, cur + 0.06)
        time.sleep(0.1)
    for v in ctx.vids:
        _ctrl(ctx, v, 0.0, 0.45, 0.0)
    _send(ctx, {})
    time.sleep(0.45)
    p1 = pose_of(ctx.api, vid)[0]
    reset_pose(vid)
    moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    ctx.note("    %s [unstick] 倒车 %.2f m -> (%.1f,%.1f)" % (vid, moved, p1[0], p1[1]))
    return moved


def back_off_roadblock(ctx, vid, seconds, clear_m=12.0):
    """扫完路障后把车倒出它的包围盒：不倒车直接开，会一路刮着 Roadblock_East 走。"""
    rb = (766.5, -88.2)
    for k in range(5):
        try:
            p, _y, _v = pose_of(ctx.api, vid)
        except Exception:                                      # noqa: BLE001
            return
        d = math.hypot(p[0] - rb[0], p[1] - rb[1])
        if d > clear_m:
            ctx.note("    %s 离路障 %.1f m，够了" % (vid, d))
            return
        ctx.note("    %s 离路障只有 %.1f m，倒车让开" % (vid, d))
        for v in ctx.vids:
            _ctrl(ctx, v, 0.0, 0.4 if v != vid else 0.0, 0.0)
        veh = VehicleControl(throttle=0.34, brake=0.0, steering=0.0)
        try:
            veh.gear = GearMode.REVERSE
        except Exception:                                      # noqa: BLE001
            pass
        ctx.ctl[vid] = veh
        t_end = time.time() + 2.4
        while time.time() < t_end and time.time() - ctx.t0 < seconds:
            _send(ctx, {})
            time.sleep(0.1)
        _ctrl(ctx, vid, 0.0, 0.6, 0.0)
        _send(ctx, {})
        reset_pose(vid)


def drive_to(ctx, vid, goal_xy, seconds, stop_xy=None, stop_m=STOP_FAR,
             poll=None, approach_xy=None):
    """开到 goal_xy（含路线体检 + 逐帧防撞 + 卡住倒车）。"""
    safe = getattr(ctx, "safety", None)
    pos, _y, _v = pose_of(ctx.api, vid)
    r = fog_route.route(ctx.graph, pos, goal_xy)
    if r is None:
        ctx.note("  [nav] 无法规划到 (%.1f,%.1f)" % goal_xy)
        return False, pos, 0.0, None
    if safe is not None:
        d_clear, at_c, oid_c = safe.path_clearance(r.pts, skip_tail=stop_m + 2.0,
                                                    skip_head=6.0)
        if d_clear < WARN_PATH_M:
            ctx.note("  [safe] 路线最近 %.2f m（%s @(%.1f,%.1f)）→ 改用栅格绕行"
                     % (d_clear, oid_c, at_c[0], at_c[1]))
            det = nav_path(ctx, pos, goal_xy)
            if det and len(det) >= 2:
                r2 = fog_route.Route(_dense(det), [3.0] * len(_dense(det)))
                if safe.path_clearance(r2.pts, skip_tail=stop_m + 2.0)[0] > d_clear:
                    r = r2
    drv = LaneDriver(r, ctx.wb, v_cap=4.6, stop_xy=stop_xy, stop_m=stop_m)
    ctx.note("  [nav] -> (%.1f,%.1f) %d wp %.0f m"
             % (goal_xy[0], goal_xy[1], len(r.pts), r.length()))
    got = None
    last = 0.0
    last_guard = 0.0
    stall_t0 = None
    n_unstick = 0
    while time.time() - ctx.t0 < seconds:
        pos, yaw, speed = pose_of(ctx.api, vid)
        now = time.time()
        ctx.poll_events_throttled()
        if abs(speed) < 0.15:
            if stall_t0 is None:
                stall_t0 = now
            elif now - stall_t0 > 1.2:
                n_unstick += 1
                if n_unstick > 5:
                    ctx.note("    %s 倒车 5 次仍走不动，放弃这一段" % vid)
                    break
                ctx.note("    %s [stuck] %.1fs 不动 @(%.1f,%.1f)，倒车脱困（第 %d 次）"
                         % (vid, now - stall_t0, pos[0], pos[1], n_unstick))
                unstick(ctx, vid, seconds, back_s=1.1 + 0.2 * n_unstick,
                        steer=0.25 if n_unstick % 2 else -0.25)
                drv.reset_heading()
                stall_t0 = None
                continue
        else:
            stall_t0 = None
        _brake_others(ctx, vid)
        thr, brk, steer = drv.step(pos, yaw, speed, 0.1)
        if safe is not None:
            thr, brk, steer, (gap, oid) = safe.guard(
                pos[0], pos[1], yaw, speed, thr, brk, steer,
                horizon=0.6 if getattr(ctx, "no_bias", False) else 1.0,
                allow_bias=not getattr(ctx, "no_bias", False))
            if gap < SAFE_GAP and now - last_guard > 3.0:
                last_guard = now
                ctx.note("    %s [safe] 净空 %.2f m（%s）→ 限速/让开" % (vid, gap, oid))
        _ctrl(ctx, vid, thr, brk, steer)
        _send(ctx, {})
        ob = to_plain(ctx.api.get_observation(vid))
        if poll is not None:
            got = poll(pos, speed, ob)
            if got is not None:
                break
        if time.time() - ctx.t0 - last > 8.0:
            last = time.time() - ctx.t0
            ctx.note("    %s (%.1f,%.1f) v=%.2f seg=%d/%d"
                     % (vid, pos[0], pos[1], speed, drv.i, len(drv.pts)))
        if drv.done:
            break
        time.sleep(0.1)
    if got is not None:
        pos, _y, speed = pose_of(ctx.api, vid)
        return True, pos, speed, got
    settle(ctx, 0.50, seconds=0.9)
    pos, _y, speed = pose_of(ctx.api, vid)
    return drv.done, pos, speed, got


def drive_polyline(ctx, vid, pts, seconds, poll=None, v_cap=3.8):
    """沿给定折线开（含防撞 + 卡住倒车）。"""
    safe = getattr(ctx, "safety", None)
    if len(pts) < 2:
        pts = [pts[0], (pts[0][0] + 0.1, pts[0][1])]
    if safe is not None:
        d_clear, at_c, oid_c = safe.path_clearance(pts, skip_tail=2.0)
        ctx.note("  [safe] 折线 %d 点，最近 %.2f m（%s @(%.1f,%.1f)）"
                 % (len(pts), d_clear, oid_c, at_c[0], at_c[1]))
    dense = _dense(pts)
    drv = LaneDriver(fog_route.Route(dense, [v_cap] * len(dense)), ctx.wb, v_cap=v_cap)
    got = None
    last = 0.0
    last_guard = 0.0
    stall_t0 = None
    n_unstick = 0
    while time.time() - ctx.t0 < seconds:
        pos, yaw, speed = pose_of(ctx.api, vid)
        now = time.time()
        ctx.poll_events_throttled()
        if abs(speed) < 0.15:
            if stall_t0 is None:
                stall_t0 = now
            elif now - stall_t0 > 1.2:
                n_unstick += 1
                if n_unstick > 5:
                    ctx.note("    %s 倒车 5 次仍走不动，放弃" % vid)
                    break
                ctx.note("    %s [stuck] @(%.1f,%.1f) seg=%d/%d 倒车脱困（第 %d 次）"
                         % (vid, pos[0], pos[1], drv.i, len(drv.pts), n_unstick))
                unstick(ctx, vid, seconds, back_s=1.1 + 0.2 * n_unstick,
                        steer=0.25 if n_unstick % 2 else -0.25)
                drv.reset_heading()
                stall_t0 = None
                continue
        else:
            stall_t0 = None
        _brake_others(ctx, vid)
        thr, brk, steer = drv.step(pos, yaw, speed, 0.1)
        if safe is not None:
            thr, brk, steer, (gap, oid) = safe.guard(
                pos[0], pos[1], yaw, speed, thr, brk, steer,
                horizon=0.6 if getattr(ctx, "no_bias", False) else 1.0,
                allow_bias=not getattr(ctx, "no_bias", False))
            if gap < SAFE_GAP and now - last_guard > 3.0:
                last_guard = now
                ctx.note("    %s [safe] 净空 %.2f m（%s）→ 限速/让开" % (vid, gap, oid))
        _ctrl(ctx, vid, thr, brk, steer)
        _send(ctx, {})
        ob = to_plain(ctx.api.get_observation(vid))
        if poll is not None:
            got = poll(pos, speed, ob)
            if got is not None:
                break
        if time.time() - ctx.t0 - last > 10.0:
            last = time.time() - ctx.t0
            ctx.note("    %s (%.1f,%.1f) v=%.2f seg=%d/%d"
                     % (vid, pos[0], pos[1], speed, drv.i, len(drv.pts)))
        if drv.done:
            break
        time.sleep(0.1)
    # 收尾也斜坡刹车：一帧 0→0.35/0.45 会判一次加速度超限（17:38 / 18:07 两局都有）
    if got is None:
        settle(ctx, 0.50, seconds=0.8)
    return got


def arc_scan(ctx, vid, sig_id, seconds, rounds=4, arc_s=2.4):
    """边走小弧边读观测，一出现带 KEY 的 hint 就地提交（不依赖航向）。"""
    for k in range(rounds):
        if time.time() - ctx.t0 > seconds - 5.0:
            return ""
        sdir = 0.6 if k % 2 == 0 else -0.6
        t_end = time.time() + arc_s
        thr_a = 0.08                      # 斜坡起步：0.22 一脚踩下去就是一次加速度违规
        while time.time() < t_end:
            _ctrl(ctx, vid, thr_a, 0.0, sdir)
            _send(ctx, {})
            thr_a = min(0.22, thr_a + 0.03)
            d, st, key = sig_state(ctx, vid, sig_id)
            if key:
                res = to_plain(ctx.api.interact(vid, sig_id, "submit_key", {"key": key}))
                ctx.note("    弧线提交 %s %s d=%.1f -> %s %s"
                         % (sig_id, key, d, res.get("granted"), res.get("message")))
                if res.get("granted"):
                    return key
            time.sleep(0.1)
        ctx.note("    弧线 %d/%d 没读到 %s" % (k + 1, rounds, sig_id))
    return ""


def stage_signals(ctx, seconds):
    """三个现场信号：滑行中提交，失败就小弧扫描补。"""
    ok = {}
    for sig_id, sig_xy in SIGNALS:
        if time.time() - ctx.t0 > seconds:
            ctx.note("time up before %s" % sig_id)
            break
        ctx.note("  --- %s -> (%.1f,%.1f)" % (sig_id, sig_xy[0], sig_xy[1]))
        state = {"tries": 0}

        def poll(pos, speed, ob, _sig=sig_id, _st=state):
            for it in (ob.get("visible_objects") or []):
                if str(it.get("object_id") or "") != _sig:
                    continue
                m = KEY_RE.search(it.get("hint") or "")
                key = m.group(0).replace(" ", "") if m else ""
                st = it.get("state") or ""
                d = it.get("distance_m") or 999.0
                if key and st == "key_visible":
                    if _st["tries"] < 4:
                        _st["tries"] += 1
                        res = to_plain(ctx.api.interact("Car_C", _sig, "submit_key",
                                                        {"key": key}))
                        ctx.note("    submit %s %s d=%.1f v=%.2f -> %s %s"
                                 % (_sig, key, d, speed, res.get("granted"),
                                    res.get("message")))
                        if res.get("granted"):
                            _st["key"] = key
                            return key
            return None

        drive_to(ctx, "Car_C", sig_xy, seconds, stop_xy=sig_xy, stop_m=STOP_FAR,
                 poll=poll)
        if state.get("key"):
            ok[sig_id] = state["key"]
        else:
            for attempt in range(3):
                d, st, key = sig_state(ctx, "Car_C", sig_id)
                ctx.note("    重试 %d: %s d=%.1f state=%s key=%s"
                         % (attempt + 1, sig_id, d, st, key or "-"))
                if d < 900 and d > 3.4:
                    creep_to(ctx, "Car_C", sig_xy, seconds, max(3.4, d - 0.8), v_cap=1.0)
                hold_still(ctx, "Car_C", seconds=1.2)
                d, st, key = sig_state(ctx, "Car_C", sig_id)
                if key:
                    res = to_plain(ctx.api.interact("Car_C", sig_id, "submit_key",
                                                    {"key": key}))
                    ctx.note("    submit %s %s d=%.1f -> %s %s"
                             % (sig_id, key, d, res.get("granted"), res.get("message")))
                    if res.get("granted"):
                        ok[sig_id] = key
                        break
            if sig_id not in ok:
                key = arc_scan(ctx, "Car_C", sig_id, seconds)
                if key:
                    ok[sig_id] = key
                else:
                    ctx.note("  !! %s 仍未通过" % sig_id)
        ts = to_plain(ctx.api.get_task_state())
        ctx.note("  signals=%s/%s phase=%s"
                 % (ts.get("verified_signal_count"), ts.get("total_signal_count"),
                    ts.get("phase")))
    return ok


def cross_one(ctx, car, seconds):
    """一辆车过闸口：到南侧待命 → 申请 → 穿闸 → 出口 release（0.5 m 细扫 + 横向搜索）。"""
    note = ctx.note
    api = ctx.api
    pos0, _y, _v = pose_of(ctx.api, car)
    detour = nav_path(ctx, pos0, (728.8, -55.0))
    note("[cross] %s 栅格规划 %d 点 → (728.8,-55.0)" % (car, len(detour)))
    drive_polyline(ctx, car, detour, seconds, v_cap=4.0)
    granted = [False]

    def poll(pos, speed, _a=None):
        if granted[0] or speed < 0.8:
            return None
        try:
            r = to_plain(api.interact(car, "Intersection_Center", "request_entry"))
        except Exception:                                      # noqa: BLE001
            return None
        if r.get("granted"):
            granted[0] = True
            note("[cross] %s GRANTED @(%.1f,%.1f) v=%.2f" % (car, pos[0], pos[1], speed))
            return "granted"
        return None

    ctx.no_bias = True
    # 闸口两墙 x=726.26 / 731.32，正中是 728.79。之前这里写 729.0（右偏 0.21 m），
    # 17:38 那局 Car_B 就在 (729.4,-17) 蹭到 Port_PassageBarrier_1_2。
    corridor_run(ctx, car, 728.79, 8.0, v_tgt=1.6, poll=poll, seconds=seconds)
    ctx.no_bias = False
    if not granted[0]:
        note("[cross] %s 未获批，跳过" % car)
        return False

    tried = [None]

    def relpoll(pos, speed, _a=None):
        y = pos[1]
        if not (-18.0 <= y <= -2.0):
            return None
        mark = int(math.floor(y / 0.05))   # 出口窗只有 ~1 m 宽，0.5 m 一格容易整格跳过去
        if tried[0] == mark:
            return None
        tried[0] = mark
        try:
            r = to_plain(api.interact(car, "Intersection_Center", "release"))
        except Exception as e:                                 # noqa: BLE001
            r = {"granted": False, "message": "ERR %s" % e}
        note("[rel] %s @(%.1f,%.1f) mark=%.1f -> %s | %s"
             % (car, pos[0], pos[1], y, r.get("granted"), str(r.get("message"))[:40]))
        return "released" if r.get("granted") else None

    # 闸口两墙在 x=726.26/731.32，中心线 x=728.79；车宽 2.45 只能走中心线。
    # 出口判定是一个**极窄的点**（实测 −11.3 能过、−11.4 就报 not reached），
    # 所以分成两段：主段 1.6 m/s，接近出口窗（y −14~−6）降到 0.5 m/s，
    # 采样标记也从 0.2 m 收到 0.1 m —— 1.6 m/s 时一帧就跨 0.15 m，整个窗会被跳过去。
    gate_fast = [(728.79, -31.0), (728.79, -24.0), (728.79, -14.0)]
    got = drive_polyline(ctx, car, gate_fast, seconds, v_cap=1.6, poll=relpoll)
    ok = (got == "released")
    if not ok:
        gate_slow = [(728.79, -14.0), (728.79, -12.0), (728.79, -11.0),
                     (728.79, -9.0), (728.79, -7.0)]
        got = drive_polyline(ctx, car, gate_slow, seconds, v_cap=0.8, poll=relpoll)
        ok = (got == "released")
    if not ok:
        # 还不行就在出口点周围做小网格补扫（通行权还在自己手里，别开出互斥区）
        ok = _release_sweep(ctx, car, seconds)
    note("[cross] %s 通行 %s" % (car, "成功" if ok else "失败"))
    return ok


def _release_sweep(ctx, car, seconds, max_s=30.0):
    """出口 release 补扫：在 (728.79,−11) 附近按小网格逐点试。

    实测出口是**很窄的一段**：y 在 −12~−10、x 必须贴着中心线 728.79。
    17:32 / 18:54 两局 B 都是从窗口边上一擦而过（x=728.5、y=−11.4 报
    `has not reached a connected intersection exit`），然后一路开到 y=+20 就回不来了。
    所以这里不再"往北扫"，而是在窗口附近横竖各试几个点。
    """
    api = ctx.api
    t_end = min(time.time() + max_s, ctx.t0 + seconds - 6.0)
    grid = [(728.79, -13.0), (728.79, -11.0), (728.79, -10.0),
            (729.10, -11.0), (728.50, -11.0),
            (728.79, -12.0), (728.79, -9.5), (728.79, -8.0)]
    asked = False
    for (px, py) in grid:
        if time.time() > t_end:
            break
        pos = pose_of(api, car)[0]
        try:
            drive_polyline(ctx, car, [(pos[0], pos[1]), (px, py)],
                           seconds, v_cap=0.8)
        except Exception as exc:                               # noqa: BLE001
            ctx.note("[cross] %s 挪到出口点异常 %s" % (car, exc))
        for _k in range(2):
            try:
                r = to_plain(api.interact(car, "Intersection_Center", "release"))
            except Exception as e:                             # noqa: BLE001
                r = {"granted": False, "message": "ERR %s" % e}
            pos = pose_of(api, car)[0]
            ctx.note("[rel] %s @(%.1f,%.1f) sweep(%.1f,%.1f) -> %s | %s"
                     % (car, pos[0], pos[1], px, py, r.get("granted"),
                        str(r.get("message"))[:36]))
            if r.get("granted"):
                return True
            time.sleep(0.3)
        if not asked and time.time() - ctx.t0 > 0:
            asked = True
            try:
                r2 = to_plain(api.interact(car, "Intersection_Center", "request_entry"))
                ctx.note("[cross] %s 重新申请通行权 -> %s"
                         % (car, str(r2.get("message"))[:40]))
            except Exception:                                  # noqa: BLE001
                pass
    return False


GATE_READY = (728.8, -55.0)      # 闸口南侧待命点（request_entry 从这里起步最顺）
GATE_HOLD = (728.8, -74.0)       # B 的备用待命点：在 A 后面 19 m，避免两车抢同一个点


def _job(ctx, vid, goal, v_cap=3.7, stop_m=2.0, limit=140.0, avoid=None, delay=0.0):
    """给 drive_parallel 造一条活：栅格 A* 从当前位姿规划到 goal。
    已经在 goal 3 m 内就返回 None（省一次规划，也避免零长路径把 LaneDriver 搞乱）。"""
    pos = pose_of(ctx.api, vid)[0]
    if math.hypot(goal[0] - pos[0], goal[1] - pos[1]) < 3.0:
        ctx.note("    [pre] %s 已在 (%.1f,%.1f)，跳过" % (vid, pos[0], pos[1]))
        return None
    pts = nav_path(ctx, pos, goal, avoid=avoid)
    if not pts or len(pts) < 2:
        pts = [pos, goal]
    return {"vid": vid, "pts": pts, "v_cap": v_cap, "stop_xy": goal,
            "stop_m": stop_m, "limit": limit, "delay": delay}


def _pre(ctx, jobs, seconds, tag):
    """把闲置的车提前送出去（和主车并跑一段），全部到位或超时后返回。"""
    jobs = [j for j in jobs if j]
    if not jobs:
        return
    ctx.note("[pre] %s → %s" % (tag, " / ".join(
        "%s(%.0f,%.0f)%d点" % (j["vid"], j["stop_xy"][0], j["stop_xy"][1],
                               len(j["pts"])) for j in jobs)))
    try:
        drive_parallel(ctx, jobs, seconds)
    except Exception as exc:                                   # noqa: BLE001
        ctx.note("[pre] %s 异常 %s: %s" % (tag, type(exc).__name__, exc))


def stage_cross(ctx, seconds):
    """A 过闸 → **立刻交付泊位 A**（D 已提前就位）→ B 过闸 → B/C 去泊位 B。

    按 HANDOFF 第 8 节方案①重排。原来 A 的交付排在 B 过闸之后（B 泊位只剩 40~60 s，
    近 8 局里 4 局卡在这里超时）；顺带修两个真 bug：
    ① 14:39 那版把 Car_D 的 parallel job 整段删掉只剩一行 note，它从开局一直待在
       (622,121)，直到 409.7 s 才被派活，跑 278 m 用掉 84 s，A 的观测被拖到 497 s；
    ② "C 去 B 的同时送 D" 那段写进了第 259 行的旧 stage_signals（已被 1653 行覆盖），
       从来没执行过 —— 现在改成在闸口这一阶段正面派出。
    """
    note = ctx.note
    note("=== 阶段 3：两辆运输车通行中央路口 ===")
    ca = _berth_center(ctx, "Berth_A", (630.97, -65.72))
    cb = _berth_center(ctx, "Berth_B", (834.47, -65.72))
    # 3.0 先把扫到的动态路障记成禁区：后面每辆车、每条路线都会自动绕开它
    sync_dyn_avoid(ctx)
    # 3.0b Car_C 扫完路障就彻底驻车：
    #     ① 阶段 3 之后没人给它发指令，最后一条还是扫描时的油门，它会自己乱窜；
    #     ② 0.35 的刹车在缓坡上按不住，19:44 那局它 514 s 还在路障边上蹭（6 条 collision）。
    park_car(ctx, "Car_C", 1.0)
    # 3.1 A/B 提前出发：A 去闸口待命、B 去后方 19 m 的备用待命点（错峰 8 s，别挤同一条车道），
    #     同时把闲置的 Car_D 送到泊位 A 北侧见证位（278 m / 约 85 s）
    _pre(ctx, [_job(ctx, "Car_A", GATE_READY, 4.0, 2.0, 130.0),
               _job(ctx, "Car_B", GATE_HOLD, 4.0, 2.0, 150.0, delay=8.0),
               _job(ctx, "Car_D", (ca[0], ca[1] + 15.0), 4.2, 2.0, 175.0)],
         seconds, "A→闸口待命，B→备用待命点，D→泊位 A 见证位")
    # 3.2 A 过闸
    ok_a = cross_one(ctx, "Car_A", seconds)
    crossed = getattr(ctx, "crossed", None)
    if crossed is None:
        crossed = ctx.crossed = {}
    crossed["Car_A"] = bool(ok_a)
    if not ok_a:
        # 通行权还在 A 手里，B 这时进互斥区就是 `unauthorized_intersection_entry`
        # （19:15 那局就是这样把整局判掉的）。所以 A 没出闸就到此为止。
        crossed["Car_B"] = False
        note("[cross] Car_A 没能出闸，通行权还在它手里 → B 不再申请，直接收尾")
        return
    # 3.3 A 去泊位南侧 → 立刻观测 + 交付（不等 B）
    if ok_a:
        _pre(ctx, [_job(ctx, "Car_A", (ca[0], ca[1] - 28.0), 4.0, 2.0, 130.0)],
             seconds, "A→泊位 A 南侧（22 m 助跑位）")
        _deliver_one(ctx, "Berth_A", "Car_A", "Car_D", ca, seconds)
    # 3.4 B 过闸（B 早就在闸口待命）
    ok_b = cross_one(ctx, "Car_B", seconds)
    crossed = getattr(ctx, "crossed", None)
    if crossed is None:
        crossed = ctx.crossed = {}
    crossed["Car_A"] = bool(ok_a)
    crossed["Car_B"] = bool(ok_b)
    if not ok_b:
        note("[cross] Car_B 没过闸，跳过 B 泊位"
             "（没进互斥区还往东开，18:54 那局撞了 24 次场院隔离柱）")
        return
    # 3.5 B 去泊位南侧；见证用交付完闲着的 Car_A（从北侧，正好顺着它去东边）
    #     Car_C 扫完路障就贴在路障边上，无论绕还是倒，近两局都刷了 20~60 条
    #     collision（Roadblock_East / QuayCrane_3 / Port_Lighthouse_Terminal），
    #     所以让它就地停住，不再参与后续阶段。
    _ctrl(ctx, "Car_C", 0.0, 0.6, 0.0)
    _send(ctx, {})
    # 见证车用 Car_D：A 交付完就停在泊位旁边不动，D 从它北侧一路向东去 B 泊位。
    # （17:32 那局让 A 自己跑去 B 见证位，A 从 (629,-67) 向东正好撞上停在
    #   (631,-54) 的 D，`[collision] Car_A 与 Car_D`。）
    _pre(ctx, [_job(ctx, "Car_B", (cb[0], cb[1] - 28.0), 4.0, 2.0, 130.0),
               _job(ctx, "Car_D", (cb[0], cb[1] + 22.0), 4.2, 2.0, 175.0)],
         seconds, "B→泊位 B 南侧（22 m 助跑位），D→泊位 B 北侧见证位")
    # 3.6 B 交付
    _deliver_one(ctx, "Berth_B", "Car_B", "Car_D", cb, seconds)


def _berth_center(ctx, name, fallback):
    ts = to_plain(ctx.api.get_task_state())
    for o in ts.get("shared_task_objects") or []:
        if str(o.get("object_id") or "") != name:
            continue
        unc = o.get("uncertainty_region") or {}
        mn, mx = unc.get("minimum") or {}, unc.get("maximum") or {}
        if mn and mx:
            return (0.5 * (mn["x"] + mx["x"]), 0.5 * (mn["y"] + mx["y"]))
    return fallback


def _stage_at(ctx, vid, outer, inner, seconds, tol=2.5, rounds=3, budget=80.0,
              face=None):
    """把车摆到 inner（从 outer 那一侧、沿着轴线正着开进去），并且车头要朝着 face。

    只许用直连折线，**不许在最后这段用 A***：车一旦贴到建筑（落在膨胀后的禁区里），
    栅格 A* 的起点会被 `_nearest_free` 挪到几十米外的空格子，规划出来的是一条
    40 m 的绕圈路线 —— 15:27 那局 Car_A 就是这样绕着 EastBulkCargoWarehouse
    一圈一圈地撞。而 outer→inner 这十几米是直的，纯跟踪自己会拉直。

    姿势不对（车头差 >40°）时再来一轮助跑：实测第二轮一定能把车头摆正
    （15:45 那局第一轮 yaw=27 看不见泊位，第二轮 yaw=83 就看见了）。
    """
    t_end = min(time.time() + budget, ctx.t0 + seconds - 20.0)

    def _eval():
        pos, dyaw, _sp = pose_of(ctx.api, vid)
        d_in = math.hypot(pos[0] - inner[0], pos[1] - inner[1])
        err = 0.0
        if face is not None:
            want = math.degrees(math.atan2(face[1] - pos[1], face[0] - pos[0]))
            # 用**位移航向**判断：ori_z 在这台车上不可靠（16:46 那局 A 的 ori_z=-18、
            # 位移航向却是北；而 D 在泊位北侧朝南、ori_z 却报 87）。
            # 这一路上都在正着开，位移航向是干净的。
            err = (dyaw - want + 540.0) % 360.0 - 180.0
        return pos, d_in, err

    for k in range(rounds):
        pos, d_in, err = _eval()
        if d_in <= tol and abs(err) <= 40.0:
            return True
        if time.time() > t_end:
            ctx.note("[deliver] %s 就位超时 d=%.1f m 车头差 %.0f° @(%.1f,%.1f)"
                     % (vid, d_in, err, pos[0], pos[1]))
            return False
        try:
            if d_in > 6.0 or abs(err) > 40.0:
                route = [(pos[0], pos[1]), outer, inner]   # 先到远端拉直再进
            else:
                route = [(pos[0], pos[1]), inner]          # 已在门口：别再跑一趟远端
            drive_polyline(ctx, vid, route, seconds, v_cap=1.4)
        except Exception as exc:                               # noqa: BLE001
            ctx.note("[deliver] %s 就位异常 %s" % (vid, exc))
        pos, d_in, err = _eval()
        ctx.note("[deliver] %s 就位第 %d 轮 d=%.1f m 车头差 %.0f° @(%.1f,%.1f)"
                 % (vid, k + 1, d_in, err, pos[0], pos[1]))
        if d_in <= tol and abs(err) <= 40.0:
            return True
    return False


def _sees(ctx, vid, name):
    """这辆车眼下能不能看见目标（观测只在 30 m / 前方 120° 内给）。"""
    try:
        ob = to_plain(ctx.api.get_observation(vid))
    except Exception:                                          # noqa: BLE001
        return False
    for it in (ob.get("visible_objects") or []):
        if str(it.get("object_id") or "") == name:
            return True
    return False


def _true_yaw(ctx, vid):
    """读 ori_z 真航向（先 reset_pose，否则拿到的是被倒车污染 180° 的位移航向）。"""
    reset_pose(vid)
    _p, yaw, _s = pose_of(ctx.api, vid)
    return yaw


def _reverse_sign(ctx, vid, seconds):
    """探一次"倒车 +0.5 舵时 yaw 往哪走"，记在 ctx 上（同一车型只需探一次）。
    读角度前必须 reset_pose：pose_of 的航向是位移法算的，倒车会让它整体翻 180°。"""
    cache = getattr(ctx, "rev_sign", None)
    if cache is None:
        cache = ctx.rev_sign = {}
    if vid in cache:
        return cache[vid]
    y0 = _true_yaw(ctx, vid)
    veh = VehicleControl(throttle=0.14, brake=0.0, steering=0.5)
    try:
        veh.gear = GearMode.REVERSE
    except Exception:                                          # noqa: BLE001
        pass
    ctx.ctl[vid] = veh
    for _ in range(4):
        _send(ctx, {})
        time.sleep(0.1)
    _ctrl(ctx, vid, 0.0, 0.45, 0.0)
    _send(ctx, {})
    time.sleep(0.4)
    y1 = _true_yaw(ctx, vid)
    dy = (y1 - y0 + 540.0) % 360.0 - 180.0
    s = 1.0 if dy > 0 else -1.0
    cache[vid] = s
    ctx.note("[turn] %s 倒车 +0.5 舵 → yaw %+.1f°（转向符号 %+0.0f）" % (vid, dy, s))
    return s


def _turn_to_face(ctx, vid, tgt_xy, seconds, tol=12.0, max_s=10.0):
    """把车头转到对着 tgt_xy —— **用倒车满舵**（正走约 2~3°/s，倒走约 50°/s）。
    只在 `_stage_at` 三轮回不来时才用；能用"重新走一趟助跑"解决就别倒车（倒车冲得远）。"""
    s = _reverse_sign(ctx, vid, seconds)
    t_end = min(time.time() + max_s, ctx.t0 + seconds - 6.0)
    p0 = pose_of(ctx.api, vid)[0]
    while time.time() < t_end:
        pos, _yaw, _sp = pose_of(ctx.api, vid)
        yaw = _true_yaw(ctx, vid)
        want = math.degrees(math.atan2(tgt_xy[1] - pos[1], tgt_xy[0] - pos[0]))
        err = (want - yaw + 540.0) % 360.0 - 180.0
        if abs(err) < tol:
            break
        steer = clip(s * 0.65 * (1.0 if err > 0 else -1.0), -0.7, 0.7)
        safe = getattr(ctx, "safety", None)
        if safe is not None:
            bx = pos[0] - 2.2 * math.cos(math.radians(yaw))
            by = pos[1] - 2.2 * math.sin(math.radians(yaw))
            gap, oid, _ = safe.gap(bx, by)
            if gap < 0.40:
                ctx.note("[turn] %s 车后有东西（%.2f m %s），不敢再倒" % (vid, gap, oid))
                break
        veh = VehicleControl(throttle=0.12, brake=0.0, steering=steer)
        try:
            veh.gear = GearMode.REVERSE
        except Exception:                                      # noqa: BLE001
            pass
        ctx.ctl[vid] = veh
        _send(ctx, {})
        if math.hypot(pos[0] - p0[0], pos[1] - p0[1]) > 8.0:
            break
        time.sleep(0.1)
    _ctrl(ctx, vid, 0.0, 0.5, 0.0)
    _send(ctx, {})
    time.sleep(0.3)
    pos = pose_of(ctx.api, vid)[0]
    yaw = _true_yaw(ctx, vid)
    want = math.degrees(math.atan2(tgt_xy[1] - pos[1], tgt_xy[0] - pos[0]))
    err = (want - yaw + 540.0) % 360.0 - 180.0
    ctx.note("[turn] %s 摆头后 yaw=%.0f 目标方位=%.0f 差=%.0f @(%.1f,%.1f)"
             % (vid, yaw, want, err, pos[0], pos[1]))
    return abs(err) < max(tol, 35.0)


def _dump_vis(ctx, tag):
    """把每辆车眼下的可见列表、位姿打出来（排查"到了却看不见"用）。"""
    for v in ctx.vids:
        try:
            ob = to_plain(ctx.api.get_observation(v))
            pos = pose_of(ctx.api, v)[0]
            yaw = _true_yaw(ctx, v)
            vis = ["%s@%.1f" % (str(it.get("object_id")),
                                it.get("distance_m") or -1)
                   for it in (ob.get("visible_objects") or [])]
            ctx.note("[vis] %s %s (%.1f,%.1f) yaw=%.0f 可见=%s"
                     % (tag, v, pos[0], pos[1], yaw, vis[:6] or "[]"))
        except Exception as exc:                               # noqa: BLE001
            ctx.note("[vis] %s %s 读取失败 %s" % (tag, v, exc))


def _wait_visible(ctx, vid, name, seconds, tries=25, pause=0.4):
    """反复读观测，等目标出现在 visible_objects 里。

    这个接口的头几次回包经常是空快照（同一位置、同一朝向，隔一两秒就出现了），
    所以"看不见"先等一等，别急着开车调整 —— 16:07 那局为了摆车头白烧了 70 s。
    """
    for _k in range(tries):
        if _sees(ctx, vid, name):
            return True
        if time.time() - ctx.t0 > seconds - 10.0:
            return False
        time.sleep(pause)
    return _sees(ctx, vid, name)


def _make_visible(ctx, vid, spots, name, seconds):
    """先在标准观测点死等（这个接口的空快照是随机出现的），再换两个斜向点。

    同一个点同一次停靠，有时 `visible=['Berth_A']`、有时 `visible=[]`；
    同一时刻北侧的 Car_D 却稳定看得见（16:46 那局的 dump：A 在 (631.1,-78.0)
    可见列表整个是空的，D 在 (631.0,-53.6) 稳定拿到 Berth_A 和 token）。
    所以先多等几秒（便宜），再换点（贵）。
    """
    if _wait_visible(ctx, vid, name, seconds, tries=30):
        return True
    for k, spot in enumerate(spots):
        pos = pose_of(ctx.api, vid)[0]
        drive_polyline(ctx, vid, [(pos[0], pos[1]), spot], seconds, v_cap=1.3)
        pos = pose_of(ctx.api, vid)[0]
        ctx.note("[look] %s 换观测点 %d/%d (%.1f,%.1f) 现在 (%.1f,%.1f)"
                 % (vid, k + 1, len(spots), spot[0], spot[1], pos[0], pos[1]))
        if _wait_visible(ctx, vid, name, seconds, tries=12):
            return True
    _dump_vis(ctx, "看不见" + name)
    return False


def _obs_spots(cx, cy, side):
    """两个斜向候选观测点：正面那个已经试过了，这里只放左右斜前方。"""
    sy = -1.0 if side < 0 else 1.0
    return [(cx - 7.0, cy + sy * 9.0), (cx + 7.0, cy + sy * 9.0)]


def _aim_at(ctx, vid, center, name, seconds, spots):
    """观测前的准备：先把两个航向都打出来，然后换观测点试。

    不在这里硬摆车头 —— 17:03 那局 `_turn_to_face` 确实把 ori_z 从 -21 转到 83，
    可车头"对着泊位"之后 A 依旧 `visible=[]`，说明 ori_z 并不是可靠的航向
    （这台车的位移航向和 ori_z 会差到 100° 以上）。观测本身交给多点轮试。
    """
    pos = pose_of(ctx.api, vid)[0]
    dyaw = pose_of(ctx.api, vid)[1]
    oyaw = _true_yaw(ctx, vid)
    want = math.degrees(math.atan2(center[1] - pos[1], center[0] - pos[0]))
    err = (want - dyaw + 540.0) % 360.0 - 180.0
    ctx.note("[aim] %s (%.1f,%.1f) 位移航向=%.0f ori_z=%.0f 泊位方位=%.0f 差=%.0f"
             % (vid, pos[0], pos[1], dyaw, oyaw, want, err))
    if _wait_visible(ctx, vid, name, seconds, tries=5):
        return True
    return _make_visible(ctx, vid, spots, name, seconds)


def deliver_berth(ctx, name, carrier, witness, center, seconds):
    """两车一南一北各 12 m 观察 → lv3 → 入位 deliver。"""
    cx, cy = center
    for attempt in range(3):
        _stage_at(ctx, carrier, (cx, cy - 24.0), (cx, cy - 12.0), seconds, face=center)
        _stage_at(ctx, witness, (cx, cy + 24.0), (cx, cy + 12.0), seconds, face=center)
        for who, side in ((carrier, -1), (witness, +1)):
            if not _aim_at(ctx, who, center, name, seconds,
                           _obs_spots(cx, cy, side)):
                ctx.note("[deliver] %s 仍然看不见 %s" % (who, name))
        hold_still(ctx, carrier, seconds=0.6)
        a = _report(ctx, carrier, name, verbose=(attempt == 0))
        d = _report(ctx, witness, name, verbose=(attempt == 0))
        lv = level_of(ctx, name)
        ctx.note("[deliver] %s 第%d次观测：%s=%s %s=%s lv=%s"
                 % (name, attempt + 1, carrier, a, witness, d, lv))
        if lv >= 3:
            break
    if level_of(ctx, name) < 3:
        ctx.note("[deliver] %s 观测没到 lv3，放弃" % name)
        return False
    tgt = precise_point(ctx, name, (cx, cy))
    ctx.note("[deliver] %s 精确点 (%.2f,%.2f)" % (name, tgt[0], tgt[1]))
    try:
        q = nav_path(ctx, pose_of(ctx.api, carrier)[0], tgt)
        drive_polyline(ctx, carrier, q, seconds, v_cap=1.6)
        creep_into(ctx, carrier, tgt, seconds, tol=0.9)
    except Exception as exc:                                   # noqa: BLE001
        ctx.note("[deliver] %s 入位异常 %s" % (name, exc))
    park_and_hold(ctx, carrier, seconds)
    for k in range(6):
        try:
            r = to_plain(ctx.api.interact(carrier, name, "deliver"))
        except Exception as exc:                               # noqa: BLE001
            ctx.note("[deliver] %s deliver 异常 %s" % (name, exc))
            break
        ctx.note("[deliver] %s deliver(%d) granted=%s | %s"
                 % (name, k + 1, r.get("granted"), r.get("message")))
        if r.get("granted"):
            return True
        park_and_hold(ctx, carrier, seconds, need_s=1.2)
    return False


def _deliver_one(ctx, name, carrier, witness, center, seconds):
    """一个泊位的完整交付：就位 → 双车观测到 lv3 → 入位 → park → deliver。
    成功后在 ctx.done_berths 里记一笔，阶段 4 兜底时跳过。"""
    note = ctx.note
    done = getattr(ctx, "done_berths", None)
    if done is None:
        done = ctx.done_berths = set()
    if name in done:
        note("[deliver] %s 之前已交付，跳过" % name)
        return True
    left = seconds - (time.time() - ctx.t0)
    if left < 70:
        note("[deliver] %s 时间不够（剩 %.0f s），跳过" % (name, left))
        return False
    center = _berth_center(ctx, name, center)     # 模糊区会缩，交付前再读一次
    note("[deliver] %s 中心 (%.1f,%.1f)，%s 交付 / %s 见证，剩 %.0f s"
         % (name, center[0], center[1], carrier, witness, left))
    try:
        ok = deliver_berth(ctx, name, carrier, witness, center, seconds)
    except Exception as exc:                                   # noqa: BLE001
        note("[deliver] %s 异常 %s: %s" % (name, type(exc).__name__, exc))
        ok = False
    note("[deliver] %s %s（%.1f s）" % (name, "交付成功" if ok else "交付失败",
                                       time.time() - ctx.t0))
    if ok:
        done.add(name)
        try:
            back_out(ctx, carrier, seconds, dist_m=8.0)
        except Exception:                                      # noqa: BLE001
            pass
    return ok


def stage_deliver_v2(ctx, seconds):
    """阶段 4：兜底。阶段 3 已按"A 先交付 → B 过闸 → B 交付"重排，
    这里只补还没交付的泊位（正常情况一条都不会跑）。"""
    note = ctx.note
    note("=== 阶段 4：泊位交付兜底 ===")
    done = getattr(ctx, "done_berths", None)
    if done is None:
        done = ctx.done_berths = set()
    plan = [("Berth_A", "Car_A", "Car_D", (630.97, -65.72)),
            ("Berth_B", "Car_B", "Car_D", (834.47, -65.72))]
    crossed = getattr(ctx, "crossed", None) or {}
    todo = [p for p in plan
            if p[0] not in done and crossed.get(p[1], True)]
    if not todo:
        note("[deliver] 两个泊位都已在阶段 3 交付完，跳过")
        return
    for name, carrier, witness, fb in todo:
        center = _berth_center(ctx, name, fb)
        _deliver_one(ctx, name, carrier, witness, center, seconds)


def drive_parallel(ctx, jobs, seconds):
    """单线程多车并行（每帧一次批量下发，避免多线程打同一条 socket）。"""
    safe = getattr(ctx, "safety", None)
    for j in jobs:
        dense = _dense(j["pts"])
        j["drv"] = LaneDriver(fog_route.Route(dense, [j.get("v_cap", 4.0)] * len(dense)),
                              ctx.wb, v_cap=j.get("v_cap", 4.0),
                              stop_xy=j.get("stop_xy"), stop_m=j.get("stop_m", 0.0))
        j["t_start"] = time.time()
        j["limit"] = float(j.get("limit", 110.0))
        j["t_log"] = 0.0
        j["stall"] = None
        j["n_unstick"] = 0
        j["done"] = False
        j["got"] = None
    while time.time() - ctx.t0 < seconds:
        ctrls = {}
        alive = 0
        for j in jobs:
            vid = j["vid"]
            if j["done"]:
                _ctrl(ctx, vid, 0.0, 0.4, 0.0)
                continue
            pos, yaw, speed = pose_of(ctx.api, vid)
            ctx.poll_events_throttled()
            if time.time() - j["t_start"] < float(j.get("delay", 0.0)):
                # 错峰出发：先让前面的车站住，别一起挤进同一条车道
                _ctrl(ctx, vid, 0.0, 0.4, 0.0)
                alive += 1
                continue
            if time.time() - j["t_start"] > j["limit"]:
                ctx.note("    [pre] %s 超时 %.0fs，先停下" % (vid, j["limit"]))
                j["done"] = True
                _ctrl(ctx, vid, 0.0, 0.5, 0.0)
                continue
            if abs(speed) < 0.15:
                j["stall"] = j["stall"] or time.time()
                if time.time() - j["stall"] > 1.2:
                    j["n_unstick"] += 1
                    if j["n_unstick"] > 4:
                        ctx.note("    [pre] %s 倒车 4 次无效，放弃" % vid)
                        j["done"] = True
                        continue
                    ctx.note("    [pre] %s 卡住 @(%.1f,%.1f) 倒车脱困" % (vid, pos[0], pos[1]))
                    unstick(ctx, vid, seconds, back_s=1.4)
                    j["drv"].reset_heading()
                    j["stall"] = None
                    continue
            else:
                j["stall"] = None
            thr, brk, steer = j["drv"].step(pos, yaw, speed, 0.1)
            if safe is not None:
                thr, brk, steer, _g = safe.guard(
                    pos[0], pos[1], yaw, speed, thr, brk, steer,
                    horizon=0.6 if getattr(ctx, "no_bias", False) else 1.0,
                    allow_bias=not getattr(ctx, "no_bias", False))
            _ctrl(ctx, vid, thr, brk, steer)
            if j.get("poll"):
                ob = to_plain(ctx.api.get_observation(vid))
                g = j["poll"](pos, speed, ob)
                if g is not None:
                    j["got"] = g
                    j["done"] = True
            if j["drv"].done:
                j["done"] = True
            if time.time() - j["t_log"] > 8.0:
                j["t_log"] = time.time()
                ctx.note("    [pre] %s (%.1f,%.1f) v=%.1f seg=%d/%d"
                         % (vid, pos[0], pos[1], speed, j["drv"].i, len(j["drv"].pts)))
            alive += 1
        _send(ctx, ctrls)
        if alive == 0:
            break
        time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=620.0)
    ap.add_argument("--probe", type=int, default=0)
    ap.add_argument("--cross", type=int, default=1)
    ap.add_argument("--deliver", type=int, default=0)
    ap.add_argument("--probe2", type=int, default=0)
    ap.add_argument("--probe3", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    api = connect_retry()
    log = open(os.path.join(args.out, "run.log"), "w", encoding="utf-8")
    t0 = time.time()

    def note(msg):
        line = "[%6.1f] %s" % (time.time() - t0, msg)
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

    vids = []
    try:
        static = to_plain(api.get_scene_static_data())
        graph = RoadGraph(static)
        specs = {v["vehicle_id"]: v for v in static.get("vehicles", [])}
        vids = list(api.get_vehicle_ids())
        wb = WHEELBASE_RATIO * float(specs["Car_C"].get("length_m") or 4.81)
        ctx = Ctx(api, vids, graph, wb, args.out, t0, note)
        ctx.safety = Safety(static)
        ctx.ctl = {v: VehicleControl(throttle=0.0, brake=0.0, steering=0.0)
                   for v in vids}
        # ↑ 开局别一上来就踩 0.35 的刹车：实测每局 2.2 s 都会因为这一下判一次
        #   `[excessive_acceleration] Car_D`（那辆车根本没动过）。静止的车不需要刹车。
        note("vehicles=%s  防撞层：实体障碍 %d 个" % (vids, len(ctx.safety.objs)))

        note("=== 阶段 0：三个现场信号 ===")
        got = stage_signals(ctx, args.seconds)
        note("signals ok=%s" % json.dumps(got, ensure_ascii=False))
        dump_state(ctx, "after_signals")

        note("=== 阶段 1：路障扫描 ===")
        scanned = stage_scan(ctx, args.seconds)
        dump_state(ctx, "after_scan")

        if args.probe:
            stage_probe(ctx, args.seconds)
        elif scanned:
            if args.cross:
                stage_cross(ctx, args.seconds)
            if args.deliver:
                stage_deliver_v2(ctx, args.seconds)
        try:
            for cmd in ("get_score", "get_run_report", "get_task_state"):
                try:
                    raw = api._request({"command": cmd})
                except Exception as exc:                           # noqa: BLE001
                    note("[score] %s 失败 %s" % (cmd, exc))
                    continue
                note("[score] %s -> %s" % (cmd, json.dumps(raw, ensure_ascii=False)[:700]))
        except Exception as exc:                               # noqa: BLE001
            pass
        try:
            for cmd in ("get_score", "get_run_report", "get_task_state"):
                try:
                    raw = api._request({"command": cmd})
                except Exception as exc:                           # noqa: BLE001
                    note("[score] %s 失败 %s" % (cmd, exc))
                    continue
                note("[score] %s -> %s" % (cmd, json.dumps(raw, ensure_ascii=False)[:700]))
        except Exception as exc:                               # noqa: BLE001
            pass
    finally:
        try:
            api.set_vehicle_controls({v: VehicleControl(throttle=0.0, brake=0.6,
                                                        steering=0.0) for v in vids})
        except Exception:                                      # noqa: BLE001
            pass
        log.close()
        try:
            api.close()
        except Exception:                                      # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
