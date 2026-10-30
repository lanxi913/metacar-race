# -*- coding: utf-8 -*-
"""港口迷雾 v3：车道航点 + 分段限速 + 卡死自救。

与 v2 的差别：
  * 航点来自 fog_route（只取每条车道的终点，去掉"往回开"的病态点）；
  * 每个航点带期望速度（转角越大越慢），纵向用 P 律跟这个速度；
  * 卡死自救：速度 < 0.15 m/s 持续 6 s 就放开刹车、给 0.5 油门 1.5 s
    （实测卡死点都在路基边上，小幅前冲能退回路面）；
  * 读码段：离信号 4.2 m 停下，轮询 `hint` 里的 KEY-xxx，读到就提交。

用法：
  python -u fog_run3.py --out ...\\logs\\fog_v3a --seconds 520
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

KEY_RE = re.compile(r"KEY[- ]?([A-Za-z]\d{1,3})")
SIGNALS = [("Signal_A", (587.47, 124.78)),
           ("Signal_B", (720.5, -96.0)),
           ("Signal_C", (766.5, -34.5))]
WHEELBASE_RATIO = 0.62
V_CAP = 5.1
MAX_STEER = 0.45


def pose_of(api, vid):
    st = api.get_vehicle_state(vid)
    p = st.pose
    return (float(p.pos_x), float(p.pos_y)), float(p.ori_z), float(st.speed_mps or 0.0)


def clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class PathDriver(object):
    """沿航点开：纯跟踪 + 每段限速 + 卡死自救。"""

    def __init__(self, route, wheelbase, v_cap=V_CAP, arrive_m=2.5):
        self.pts = list(route.pts)
        self.speeds = list(route.speeds)
        self.wb = wheelbase
        self.v_cap = v_cap
        self.arrive_m = arrive_m
        self.i = 0
        self.done = False
        self.stuck_t = 0.0
        self.recover_t = 0.0

    def _closest_seg(self, pos):
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

    def _lookahead_point(self, k, t, dist):
        """从"投影点"沿路径前视 dist 米。

        v3 第一版从"段起点"起算，车一过段起点目标点就落在车后，
        于是车冲过路口一直往西撞到地图边界（实测 (559,128) 卡死）。
        """
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
        if self.done:
            return 0.0, (0.30 if speed > 0.05 else 0.05), 0.0
        _d, k, t = self._closest_seg(pos)
        self.i = max(self.i, k)
        remain = math.hypot(self.pts[-1][0] - pos[0], self.pts[-1][1] - pos[1])
        if remain <= self.arrive_m:
            self.done = True
            return 0.0, (0.30 if speed > 0.05 else 0.05), 0.0

        ld = clip(1.15 * max(speed, 1.0) + 3.5, 4.0, 11.0)
        tgt = self._lookahead_point(k, t, ld)
        yaw = math.radians(yaw_deg)
        hx, hy = math.cos(yaw), -math.sin(yaw)
        alpha = math.atan2(tgt[1] - pos[1], tgt[0] - pos[0]) - math.atan2(hy, hx)
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi
        err = -alpha
        steer = clip(math.atan2(2.0 * self.wb * math.sin(err),
                                max(math.hypot(tgt[0] - pos[0], tgt[1] - pos[1]), 1.0))
                     / MAX_STEER, -1.0, 1.0)

        v_t = min(self.speeds[min(k, len(self.speeds) - 1)] if self.speeds else self.v_cap,
                  self.v_cap)
        if remain < 10.0:
            v_t = min(v_t, max(1.2, math.sqrt(2.0 * 1.0 * max(remain - self.arrive_m, 0.2))))
        # 大转角自己也要降速
        v_t = min(v_t, max(1.2, self.v_cap - 4.0 * abs(err) / math.pi))

        # 卡死自救
        if speed < 0.15:
            self.stuck_t += dt
        else:
            self.stuck_t = 0.0
        if self.recover_t > 0.0:
            self.recover_t -= dt
            return 0.52, 0.0, clip(steer, -0.25, 0.25)
        if self.stuck_t > 6.0:
            self.stuck_t = 0.0
            self.recover_t = 1.5
            return 0.52, 0.0, clip(steer, -0.25, 0.25)

        err_v = v_t - speed
        thr = clip(0.16 * err_v * 10.0, 0.0, 0.42) if err_v > 0 else 0.0
        brk = clip(0.22 * (-err_v), 0.0, 0.30) if err_v < -0.25 else 0.0
        if speed < 0.25 and v_t > 0.6:
            thr = max(thr, 0.20)
        return thr, brk, steer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=520.0)
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
            # 目标：离信号最近的车道点（4~6 m 外），到了再直扑 4.2 m
            best = None
            for rid in graph.roads:
                _t, p, d = graph.project(sig_xy, rid)
                if best is None or d < best[0]:
                    best = (d, p)
            road_pt = best[1]
            pos, _yaw, _v = pose_of(api, args.patrol)
            r = fog_route.route(graph, pos, road_pt)
            if r is None:
                note("no route to %s" % sig_id)
                break
            note("leg %s: %d wp %.0f m (road_pt %.1f,%.1f)"
                 % (sig_id, len(r), r.length(), road_pt[0], road_pt[1]))
            drv = PathDriver(r, wb, v_cap=V_CAP, arrive_m=2.5)

            key = None
            last = 0.0
            app = (sig_xy[0], sig_xy[1] - 4.2)
            while time.time() - t0 < args.seconds:
                pos, yaw, speed = pose_of(api, args.patrol)
                ctrls = {v: VehicleControl(throttle=0.0, brake=0.35, steering=0.0)
                         for v in vids if v != args.patrol}
                if not drv.done:
                    thr, brk, steer = drv.step(pos, yaw, speed, 0.1)
                else:
                    # 直扑接近点
                    d_sig = math.hypot(pos[0] - sig_xy[0], pos[1] - sig_xy[1])
                    dd = math.hypot(app[0] - pos[0], app[1] - pos[1])
                    if dd > 1.6:
                        yaw_r = math.radians(yaw)
                        hx, hy = math.cos(yaw_r), -math.sin(yaw_r)
                        alpha = math.atan2(app[1] - pos[1], app[0] - pos[0]) - math.atan2(hy, hx)
                        while alpha > math.pi:
                            alpha -= 2 * math.pi
                        while alpha < -math.pi:
                            alpha += 2 * math.pi
                        steer = clip(math.atan2(2.0 * wb * math.sin(-alpha),
                                                max(dd, 1.0)) / MAX_STEER, -1.0, 1.0)
                        thr = 0.22 if speed < 1.2 else 0.0
                        brk = 0.12 if speed > 1.6 else 0.0
                    else:
                        thr, brk, steer = 0.0, (0.28 if speed > 0.06 else 0.05), 0.0
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
                    note("  %s (%.1f,%.1f) v=%.2f seg=%d/%d"
                         % (args.patrol, pos[0], pos[1], speed, drv.i, len(drv.pts)))
                if key is not None:
                    break
                if drv.done and math.hypot(pos[0] - sig_xy[0], pos[1] - sig_xy[1]) <= 6.9:
                    t_wait = time.time()
                    while time.time() - t_wait < 4.0 and key is None:
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
                                    note("  (wait) %s key=%s" % (it.get("object_id"), key))
                        time.sleep(0.25)
                    if key is None:
                        note("!! 到点但没读到码（信号不在 7 m 内？）")
                    break
                time.sleep(0.1)

            if key is None:
                note("!! %s 未读到码，跳过" % sig_id)
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
        note("saved task.json obs_patrol.json")
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
