# -*- coding: utf-8 -*-
"""港口迷雾 v5：车道航点 + "到信号 6 m 就停"（不撞闸口立柱）。

v3b/v4a 的实测：灯塔路的车道尽头就是 `Port_Lighthouse_Terminal`
（0.7×0.6 m 的闸口立柱，真的是碰撞体），车按航点直扑过去会在
(718.1,-94.4) 顶住立柱、2.8 m 处卡死 180 s，而且因为贴太近反而读不到码。

v5 的规则：不看航点，看距离 —— 车沿车道开到离信号 ≤6.0 m 就刹停，
在那儿读码（Signal_A 实测 6.9 m 就能读）；读不到再往前蹭 1 m，最多两次。
"""
import argparse
import json
import math
import os
import re
import time

from metacar.models import VehicleControl
from fog_map import RoadGraph, connect_retry, to_plain
import fog_route
import fog_nav
from fog_run3 import pose_of, clip, KEY_RE, WHEELBASE_RATIO, MAX_STEER

# 实测（fog_v5b）：Relay 终端看到的是 Signal_B，Customs 是 Signal_A ⇒
# Lighthouse 就是 Signal_C。必须按 A→B→C 的顺序提交。
SIGNALS = [("Signal_A", (587.47, 124.78)),    # Port_Customs_Terminal
           ("Signal_B", (766.5, -34.5)),        # Port_Relay_Terminal
           ("Signal_C", (720.5, -96.0))]        # Port_Lighthouse_Terminal

STOP_FAR = 6.0
CREEP_STEP = 1.0


class LaneDriver(object):
    """车道航点 + 距离触发停车 + 卡死自救。"""

    def __init__(self, route, wheelbase, v_cap=5.1, stop_xy=None, stop_m=STOP_FAR):
        self.pts = list(route.pts)
        self.speeds = list(route.speeds) or [v_cap]
        self.wb = wheelbase
        self.v_cap = v_cap
        self.stop_xy = stop_xy
        self.stop_m = stop_m
        self.i = 0
        self.done = False
        self.stuck_t = 0.0
        self.recover_t = 0.0
        self.creep_t = 0.0

    def _closest(self, pos):
        best = None
        for k in range(max(0, self.i - 1), len(self.pts) - 1):
            ax, ay = self.pts[k]
            bx, by = self.pts[k + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            if L2 < 1e-9:
                continue
            t = clip(((pos[0] - ax) * dx + (pos[1] - ay) * dy) / L2, 0.0, 1.0)
            px, py = ax + t * dx, ay + t * dy
            d = math.hypot(pos[0] - px, pos[1] - py)
            if best is None or d < best[0]:
                best = (d, k, t)
        return best or (1e9, 0, 0.0)

    def _lookahead(self, k, t, dist):
        ax, ay = self.pts[k]
        bx, by = self.pts[k + 1]
        seg = math.hypot(bx - ax, by - ay)
        acc = (1.0 - t) * seg
        if acc >= dist:
            r = (t * seg + dist) / max(seg, 1e-6)
            return (ax + (bx - ax) * r, ay + (by - ay) * r)
        for j in range(k + 1, len(self.pts) - 1):
            ax, ay = self.pts[j]
            bx, by = self.pts[j + 1]
            seg = math.hypot(bx - ax, by - ay)
            if acc + seg >= dist:
                r = (dist - acc) / max(seg, 1e-6)
                return (ax + (bx - ax) * r, ay + (by - ay) * r)
            acc += seg
        return self.pts[-1]

    def step(self, pos, yaw_deg, speed, dt):
        if self.stop_xy is not None:
            d_sig = math.hypot(pos[0] - self.stop_xy[0], pos[1] - self.stop_xy[1])
            if d_sig <= self.stop_m:
                self.done = True
                return 0.0, (0.28 if speed > 0.06 else 0.05), 0.0
        if self.done:
            return 0.0, (0.28 if speed > 0.06 else 0.05), 0.0

        _d, k, t = self._closest(pos)
        self.i = max(self.i, k)
        remain = math.hypot(self.pts[-1][0] - pos[0], self.pts[-1][1] - pos[1])
        if remain <= 1.5:
            self.done = True
            return 0.0, (0.28 if speed > 0.06 else 0.05), 0.0

        ld = clip(1.15 * max(speed, 1.0) + 3.5, 4.0, 11.0)
        tgt = self._lookahead(k, t, ld)
        yaw = math.radians(yaw_deg)
        hx, hy = math.cos(yaw), -math.sin(yaw)
        alpha = math.atan2(tgt[1] - pos[1], tgt[0] - pos[0]) - math.atan2(hy, hx)
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi
        steer = clip(math.atan2(2.0 * self.wb * math.sin(-alpha),
                                max(math.hypot(tgt[0] - pos[0], tgt[1] - pos[1]), 1.0))
                     / MAX_STEER, -1.0, 1.0)

        v_t = min(self.speeds[min(k, len(self.speeds) - 1)], self.v_cap)
        if self.stop_xy is not None:
            d_sig = math.hypot(pos[0] - self.stop_xy[0], pos[1] - self.stop_xy[1])
            # 靠近信号时按"到停点的距离"减速
            v_t = min(v_t, max(1.0, math.sqrt(2.0 * 0.9 * max(d_sig - self.stop_m, 0.2))))
        if remain < 10.0:
            v_t = min(v_t, max(1.0, math.sqrt(2.0 * 1.0 * max(remain - 1.5, 0.2))))
        v_t = min(v_t, max(1.2, self.v_cap - 4.0 * abs(-alpha) / math.pi))

        if speed < 0.15:
            self.stuck_t += dt
        else:
            self.stuck_t = 0.0
        if self.recover_t > 0.0:
            self.recover_t -= dt
            return 0.5, 0.0, clip(steer, -0.2, 0.2)
        if self.stuck_t > 5.0:
            self.stuck_t = 0.0
            self.recover_t = 1.2
            return 0.5, 0.0, clip(steer, -0.2, 0.2)

        err_v = v_t - speed
        thr = clip(0.16 * err_v * 10.0, 0.0, 0.42) if err_v > 0 else 0.0
        brk = clip(0.22 * (-err_v), 0.0, 0.32) if err_v < -0.2 else 0.0
        if speed < 0.25 and v_t > 0.5:
            thr = max(thr, 0.20)
        return thr, brk, steer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=470.0)
    ap.add_argument("--patrol", default="Car_C")
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
        wb = WHEELBASE_RATIO * float(specs[args.patrol].get("length_m") or 4.81)
        note("vehicles=%s patrol=%s wb=%.2f" % (vids, args.patrol, wb))

        submitted = {}
        for sig_id, sig_xy in SIGNALS:
            if time.time() - t0 > args.seconds:
                note("time up before %s" % sig_id)
                break
            pos, _y, _v = pose_of(api, args.patrol)
            # 目标改成"信号本身"，v5 靠距离触发停车
            r = fog_route.route(graph, pos, sig_xy)
            if r is None:
                # 信号不在车道附近 35 m 内时退回最近车道点
                best = None
                for rid in graph.roads:
                    _t, p, d = graph.project(sig_xy, rid)
                    if best is None or d < best[0]:
                        best = (d, p)
                r = fog_route.route(graph, pos, best[1])
            if r is None:
                note("no route to %s" % sig_id)
                break
            note("leg %s: %d wp %.0f m, 到 %.1f m 停车读码"
                 % (sig_id, len(r), r.length(), STOP_FAR))
            drv = LaneDriver(r, wb, v_cap=5.1, stop_xy=sig_xy, stop_m=STOP_FAR)

            key = None
            last = 0.0
            creep_until = 0.0
            creep_n = 0
            while time.time() - t0 < args.seconds:
                pos, yaw, speed = pose_of(api, args.patrol)
                d_sig = math.hypot(pos[0] - sig_xy[0], pos[1] - sig_xy[1])
                ctrls = {v: VehicleControl(throttle=0.0, brake=0.35, steering=0.0)
                         for v in vids if v != args.patrol}
                if creep_until > time.time():
                    thr, brk, steer = 0.26, 0.0, 0.0
                else:
                    thr, brk, steer = drv.step(pos, yaw, speed, 0.1)
                ctrls[args.patrol] = VehicleControl(throttle=thr, brake=brk, steering=steer)
                api.set_vehicle_controls(ctrls)

                ob = to_plain(api.get_observation(args.patrol))
                for it in (ob.get("visible_objects") or []):
                    oid = str(it.get("object_id") or "")
                    if oid.startswith("Signal_"):
                        m = KEY_RE.search(it.get("hint") or "")
                        if m and it.get("state") == "key_visible":
                            key = m.group(0).replace(" ", "")
                            note("  %s key=%s d=%.1f" % (oid, key, it.get("distance_m") or -1))
                if time.time() - t0 - last > 6.0:
                    last = time.time() - t0
                    stt = api.get_vehicle_state(args.patrol)
                    note("  %s (%.1f,%.1f) v=%.2f d_sig=%.1f seg=%d/%d state=%s obs=%d"
                         % (args.patrol, pos[0], pos[1], speed, d_sig, drv.i,
                            len(drv.pts), getattr(stt, "state", "?"),
                            len(ob.get("obstacles") or [])))
                if key is not None:
                    break
                if drv.done and key is None:
                    if creep_n < 2 and d_sig > 3.6:
                        creep_n += 1
                        creep_until = time.time() + (CREEP_STEP / 0.9)
                        drv.stop_m = max(3.4, d_sig - CREEP_STEP)
                        note("  没读到码：往前蹭 %.1f m（第 %d 次）" % (CREEP_STEP, creep_n))
                    else:
                        note("  停住等 3 s 再看")
                        t_wait = time.time()
                        while time.time() - t_wait < 3.0 and key is None:
                            api.set_vehicle_controls({v: VehicleControl(throttle=0.0,
                                                                       brake=0.25,
                                                                       steering=0.0)
                                                      for v in vids})
                            ob = to_plain(api.get_observation(args.patrol))
                            for it in (ob.get("visible_objects") or []):
                                if str(it.get("object_id") or "").startswith("Signal_"):
                                    m = KEY_RE.search(it.get("hint") or "")
                                    if m:
                                        key = m.group(0).replace(" ", "")
                                        note("  (wait) key=%s" % key)
                            time.sleep(0.3)
                        break
                time.sleep(0.1)

            if key is None:
                note("!! %s 未读到码" % sig_id)
                continue
            res = to_plain(api.interact(args.patrol, sig_id, "submit_key", {"key": key}))
            note("submit %s %s -> granted=%s %s" % (sig_id, key, res.get("granted"),
                                                    res.get("message")))
            if res.get("granted"):
                submitted[sig_id] = key
            ts = to_plain(api.get_task_state())
            note("  signals=%s/%s roadblock=%s"
                 % (ts.get("verified_signal_count"), ts.get("total_signal_count"),
                    ts.get("roadblock_active")))

        note("submitted=%s" % json.dumps(submitted, ensure_ascii=False))
        ts = to_plain(api.get_task_state())
        ob = to_plain(api.get_observation(args.patrol))
        rbs = [(it.get("object_id"), it.get("distance_m"), it.get("position"))
               for it in (ob.get("visible_objects") or [])
               if it.get("object_type") == "roadblock"]
        note("roadblock visible=%s active=%s" % (rbs, ts.get("roadblock_active")))
        with open(os.path.join(args.out, "task.json"), "w", encoding="utf-8") as fh:
            json.dump(ts, fh, ensure_ascii=False, indent=1)
        with open(os.path.join(args.out, "obs_patrol.json"), "w", encoding="utf-8") as fh:
            json.dump(ob, fh, ensure_ascii=False, indent=1)
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
