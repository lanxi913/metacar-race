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
    """踩住刹车直到车速接近 0。返回最终速度。"""
    t_end = time.time() + seconds
    sp = 9.9
    while time.time() < t_end:
        pos, yaw, sp = pose_of(ctx.api, vid)
        ctrls = {v: VehicleControl(throttle=0.0, brake=0.4, steering=0.0)
                 for v in ctx.vids}
        ctrls[vid] = VehicleControl(throttle=0.0, brake=0.6, steering=0.0)
        ctx.api.set_vehicle_controls(ctrls)
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
        ctrls = {v: VehicleControl(throttle=0.0, brake=0.35, steering=0.0)
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


def nav_path(ctx, frm, to, inflate=2.4):
    """用栅格 A* 规划（避开静态物），返回折线船点。"""
    if getattr(ctx, "grid", None) is None:
        st = to_plain(ctx.api.get_scene_static_data())
        ctx.grid = fog_nav.Grid(st)
    pts = ctx.grid.astar(frm, to)
    if pts:
        pts = fog_nav.smooth(pts, ctx.grid)
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


def drive_polyline(ctx, vid, pts, seconds, poll=None, v_cap=4.2):
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
        drive_polyline(ctx, "Car_C", seq, seconds, poll=poll)
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
        ctx.note("[obs] %s 看到 %s d=%.1f tok=%s"
                 % (carrier, name, it.get("distance_m") or -1, tok[:12]))
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
    """分段瞑头蜷行到目标点附近（tol 内），每段 4 s 重新对准。"""
    for k in range(rounds):
        pos = pose_of(ctx.api, vid)[0]
        d = math.hypot(pos[0] - target[0], pos[1] - target[1])
        ctx.note("[creep] %s d=%.2f (%.1f,%.1f)" % (vid, d, pos[0], pos[1]))
        if d <= tol:
            return True
        if time.time() - ctx.t0 > seconds - 5.0:
            return False
        try:
            drive_polyline(ctx, vid, [(pos[0], pos[1]), (target[0], target[1])], seconds, v_cap=0.9)
        except Exception as exc:
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
        ctrls = {v: VehicleControl(throttle=0.0, brake=0.5, steering=0.0) for v in ctx.vids}
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
              (800.0, -10.0), (812.0, -30.0), (838.5, -49.5), (838.5, -59.5)]
    try:
        from_a2 = [(632.0, -62.0), (636.0, -58.5)] + from_a[1:]
        drive_multi(ctx, [("Car_B", north_b, 4.2, 3.0), ("Car_A", from_a2, 4.6, 3.0)], seconds)
        face_target(ctx, "Car_B", (834.47, -65.72), seconds)
        face_target(ctx, "Car_A", (834.47, -65.72), seconds)
    except Exception as exc:
        note("[deliver] 并行赶赴 B 异常 %s: %s" % (type(exc).__name__, exc))
    for k in range(8):
        b = _report(ctx, "Car_B", "Berth_B")
        c = _report(ctx, "Car_A", "Berth_B", verbose=True)
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
        for vid, standoff in (("Car_B", 6.0), ("Car_A", 8.0)):
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


if __name__ == "__main__":
    main()
