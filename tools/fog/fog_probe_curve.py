# -*- coding: utf-8 -*-
"""油门 / 刹车完整响应曲线（2026-10-31）。

对每一档控制量，从静止（油门）或以 ~5 m/s 巡航（刹车）开始，10 Hz 采样 2 s，
统计：稳态速度、峰值速度、到 90% 用时、逐帧加速度峰值、**1 秒窗平均加速度峰值**
（平台的提示语是"持续超过 5 m/s²"，所以窗口值才是真正对得上的口径），
以及这一段里平台记了哪些 violation。

用法（场景已起）：
    python fog_probe_curve.py --out <日志目录>
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from metacar.models import GearMode, VehicleControl                 # noqa: E402
from fog_map import connect_retry, to_plain                         # noqa: E402

THR_SWEEP = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10,
             0.15, 0.20, 0.30, 0.50, 0.75, 1.00]
BRK_SWEEP = [0.05, 0.10, 0.20, 0.30, 0.50, 0.75, 1.00]
THR_HOLD = 2.0
BRK_HOLD = 2.0
PRE_SPEED_THR = 0.05          # 刹车实验前先用这个油门加到 ~5 m/s


def plain(x):
    try:
        return to_plain(x)
    except Exception:                                          # noqa: BLE001
        pass
    if isinstance(x, dict):
        return x
    d = getattr(x, "__dict__", None)
    if isinstance(d, dict):
        return {k: v for k, v in d.items() if not k.startswith("_")}
    return {"repr": repr(x)}


class Probe(object):
    def __init__(self, api, car, t0, out):
        self.api = api
        self.car = car
        self.t0 = t0
        self.ev_seq = 0
        self.log = open(os.path.join(out, "curve.log"), "w", encoding="utf-8")
        self.rows = []

    def w(self, msg):
        line = "[%6.2f] %s" % (time.time() - self.t0, msg)
        print(line, flush=True)
        self.log.write(line + "\n")
        self.log.flush()

    def _events(self, tag):
        got = []
        try:
            evs = plain(self.api.get_events(self.ev_seq))
            if isinstance(evs, dict):
                evs = evs.get("events") or []
            for e in evs or []:
                d = e if isinstance(e, dict) else plain(e)
                if not isinstance(d, dict):
                    continue
                seq = int(d.get("sequence") or 0)
                if seq > self.ev_seq:
                    self.ev_seq = seq
                if (d.get("event_type") or "") in ("violation", "warning", "penalty"):
                    got.append("%s#%s" % (d.get("message", "")[:40], seq))
                    self.w("   !! EVENT %s" % str(d.get("message"))[:80])
        except Exception:                                      # noqa: BLE001
            pass
        return got

    def hold(self, thr, brk, hold_s, tag, gear=None):
        """保持一段控制量，10 Hz 采样，返回这一段的统计。"""
        api, car = self.api, self.car
        t_start = time.time()
        t_end = t_start + hold_s
        rows = []
        evs = []
        while time.time() < t_end:
            veh = VehicleControl(throttle=thr, brake=brk, steering=0.0)
            try:
                veh.gear = gear if gear is not None else GearMode.DRIVE
            except Exception:                                  # noqa: BLE001
                pass
            try:
                api.set_vehicle_controls({car: veh})
                st = plain(api.get_vehicle_state(car))
            except Exception as exc:                           # noqa: BLE001
                self.w("   控制失败 %s" % exc)
                break
            sp = float(st.get("speed_mps") or 0.0)
            pose = st.get("pose") or {}
            px = py = 0.0
            if isinstance(pose, dict):
                px = float(pose.get("pos_x") or 0.0)
                py = float(pose.get("pos_y") or 0.0)
            rows.append((time.time() - t_start, sp, px, py))
            evs.extend(self._events(tag))
            time.sleep(0.1)

        if not rows:
            return None
        sp = [r[1] for r in rows]
        ts = [r[0] for r in rows]
        v_ss = sum(sp[-5:]) / min(5, len(sp))
        v_max = max(sp)
        # 到 90% 稳态的时间
        t90 = None
        for i, v in enumerate(sp):
            if v_ss > 0.2 and abs(v) >= 0.9 * abs(v_ss):
                t90 = ts[i]
                break
        # 逐帧加速度峰值
        a1 = 0.0
        for i in range(1, len(sp)):
            dt = max(ts[i] - ts[i - 1], 1e-3)
            a1 = max(a1, abs(sp[i] - sp[i - 1]) / dt)
        # 1 秒窗加速度峰值（平台口径更像这个）
        aw = 0.0
        for i in range(len(sp)):
            j = i - 10
            if j < 0:
                continue
            dt = max(ts[i] - ts[j], 1e-3)
            aw = max(aw, abs(sp[i] - sp[j]) / dt)
        dist = 0.0
        for i in range(1, len(rows)):
            dist += ((rows[i][2] - rows[i - 1][2]) ** 2
                     + (rows[i][3] - rows[i - 1][3]) ** 2) ** 0.5
        blocked = bool(sp and max(sp) < 0.10 and thr >= 0.02)
        return dict(v0=sp[0], v_ss=v_ss, v_max=v_max, t90=t90, a1=a1, aw=aw,
                    dist=dist, blocked=blocked, evs=evs, n=len(rows))

    def stop(self, hold_s=1.3):
        self.hold(0.0, 1.0, hold_s, "stop")

    def escape(self):
        """卡住了：倒车 2.5 s 让开。"""
        self.w("   检测到卡住，倒车脱困")
        self.hold(0.05, 0.0, 2.5, "reverse", gear=GearMode.REVERSE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    api = connect_retry()
    t0 = time.time()
    vids = list(api.get_vehicle_ids())
    car = "Car_C"
    for other in vids:
        if other != car:
            try:
                veh = VehicleControl(throttle=0.0, brake=1.0, steering=0.0)
                veh.gear = GearMode.PARKING
                api.set_vehicle_controls({other: veh})
            except Exception:                                  # noqa: BLE001
                pass

    p = Probe(api, car, t0, args.out)
    p.w("=== 油门扫描（%s）===" % car)
    thr_res = []
    for thr in THR_SWEEP:
        p.stop()
        r = p.hold(thr, 0.0, THR_HOLD, "thr%.2f" % thr)
        if r is None:
            continue
        thr_res.append((thr, r))
        p.w("油门 %.2f | 稳态 %5.2f | 峰值 %5.2f | t90 %5s | 逐帧 %5.1f | 1s窗 %5.1f | 路程 %5.1f m%s"
            % (thr, r["v_ss"], r["v_max"],
               ("%.1f" % r["t90"]) if r["t90"] is not None else ">2",
               r["a1"], r["aw"], r["dist"], "  [卡住]" if r["blocked"] else ""))
        if r["blocked"]:
            p.escape()

    p.w("=== 刹车扫描（先油门 %.2f 加到 ~5 m/s）===" % PRE_SPEED_THR)
    brk_res = []
    for brk in BRK_SWEEP:
        p.stop()
        p.hold(PRE_SPEED_THR, 0.0, 2.5, "pre")
        r = p.hold(0.0, brk, BRK_HOLD, "brk%.2f" % brk)
        if r is None:
            continue
        brk_res.append((brk, r))
        p.w("刹车 %.2f | 起速 %5.2f | 末速 %5.2f | 用时 %5s | 逐帧 %5.1f | 1s窗 %5.1f | 刹距 %5.2f m%s"
            % (brk, r["v0"], r["v_ss"],
               ("%.1f" % r["t90"]) if r["t90"] is not None else ">2",
               r["a1"], r["aw"], r["dist"], "  [卡住]" if r["blocked"] else ""))
        if r["blocked"]:
            p.escape()

    p.w("=== 汇总：油门 ===")
    p.w("%6s %8s %8s %8s %9s %9s" % ("thr", "稳态m/s", "峰值m/s", "t90(s)", "逐帧a", "1s窗a"))
    for thr, r in thr_res:
        p.w("%6.2f %8.2f %8.2f %8s %9.1f %9.1f"
            % (thr, r["v_ss"], r["v_max"],
               ("%.1f" % r["t90"]) if r["t90"] is not None else ">2", r["a1"], r["aw"]))
    p.w("=== 汇总：刹车 ===")
    p.w("%6s %8s %8s %8s %9s %9s" % ("brk", "起速", "末速", "t90(s)", "逐帧a", "1s窗a"))
    for brk, r in brk_res:
        p.w("%6.2f %8.2f %8.2f %8s %9.1f %9.1f"
            % (brk, r["v0"], r["v_ss"],
               ("%.1f" % r["t90"]) if r["t90"] is not None else ">2", r["a1"], r["aw"]))
    try:
        veh = VehicleControl(throttle=0.0, brake=1.0, steering=0.0)
        veh.gear = GearMode.PARKING
        api.set_vehicle_controls({car: veh})
    except Exception:                                          # noqa: BLE001
        pass
    p.log.close()


if __name__ == "__main__":
    main()
