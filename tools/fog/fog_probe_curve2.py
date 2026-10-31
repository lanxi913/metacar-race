# -*- coding: utf-8 -*-
"""油门 / 刹车完整响应曲线（往返式，2026-10-31）。

上一版的问题是车一路往西开，30 m 后撞上 Signal_A 标志，后半段数据全废。
这一版每一档都"前进 → 刹停 → 倒车回原位 → 刹停"，净位移≈0，可以在起点附近把
所有档位都测完。

统计量：
  v_ss   稳态速度（末 0.5 s 平均）
  t90    到 90% 稳态用时
  a1     逐帧 dv/dt 峰值（10 Hz 观测）
  aw     1 秒窗平均加速度峰值（平台提示语"持续超过 5 m/s²"对得上的口径）
  dist   该段行驶距离
  evs    该段平台记的 violation
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from metacar.models import GearMode, VehicleControl                 # noqa: E402
from fog_map import connect_retry, to_plain                         # noqa: E402

THR_SWEEP = [0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.75, 1.00]
BRK_SWEEP = [0.03, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30, 0.50, 0.75, 1.00]


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
        self.api, self.car, self.t0 = api, car, t0
        self.ev_seq = 0
        self.log = open(os.path.join(out, "curve2.log"), "w", encoding="utf-8")

    def w(self, msg):
        line = "[%6.2f] %s" % (time.time() - self.t0, msg)
        print(line, flush=True)
        self.log.write(line + "\n")
        self.log.flush()

    def events(self):
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
                    got.append(str(d.get("message"))[:60])
                    self.w("   !! EVENT %s" % str(d.get("message"))[:70])
        except Exception:                                      # noqa: BLE001
            pass
        return got

    def hold(self, thr, brk, hold_s, tag, gear=None):
        api, car = self.api, self.car
        rows, evs = [], []
        t_end = time.time() + hold_s
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
            pose = st.get("pose") or {}
            px = py = 0.0
            if isinstance(pose, dict):
                px = float(pose.get("pos_x") or 0.0)
                py = float(pose.get("pos_y") or 0.0)
            rows.append((time.time(), float(st.get("speed_mps") or 0.0), px, py))
            evs.extend(self.events())
            time.sleep(0.08)
        if not rows:
            return None
        ts = [r[0] - rows[0][0] for r in rows]
        sp = [r[1] for r in rows]
        v_ss = sum(sp[-5:]) / min(5, len(sp))
        t90 = None
        for i, v in enumerate(sp):
            if abs(v_ss) > 0.3 and abs(v) >= 0.9 * abs(v_ss):
                t90 = ts[i]
                break
        a1 = 0.0
        for i in range(1, len(sp)):
            a1 = max(a1, abs(sp[i] - sp[i - 1]) / max(ts[i] - ts[i - 1], 1e-3))
        aw = 0.0
        for i in range(len(sp)):
            j = i - 10
            if j < 0:
                continue
            aw = max(aw, abs(sp[i] - sp[j]) / max(ts[i] - ts[j], 1e-3))
        dist = 0.0
        for i in range(1, len(rows)):
            dist += ((rows[i][2] - rows[i - 1][2]) ** 2
                     + (rows[i][3] - rows[i - 1][3]) ** 2) ** 0.5
        return dict(v0=sp[0], v_ss=v_ss, v_max=max(sp), t90=t90, a1=a1, aw=aw,
                    dist=dist, evs=evs, blocked=(max(sp) < 0.15 and thr >= 0.02))

    def back(self, sec=1.6):
        """倒车回原位（反方向），再刹停。"""
        self.hold(0.05, 0.0, sec, "back", gear=GearMode.REVERSE)
        self.hold(0.0, 1.0, 1.0, "stop")


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

    p.w("=== 高油门段（每档前进 1.5 s，然后倒车回原位）===")
    hi = []
    for thr in THR_SWEEP:
        r = p.hold(thr, 0.0, 1.5, "thr%.2f" % thr)
        if r:
            hi.append((thr, r))
            p.w("油门 %.2f | 稳态 %5.2f | 峰值 %5.2f | t90 %5s | 逐帧 %5.1f | 1s窗 %5.1f | 路程 %4.1f m%s"
                % (thr, r["v_ss"], r["v_max"],
                   ("%.1f" % r["t90"]) if r["t90"] is not None else ">1.5",
                   r["a1"], r["aw"], r["dist"], "  [卡住]" if r["blocked"] else ""))
        p.back()

    p.w("=== 刹车段（油门 0.05 加到 ~5 m/s，再施加刹车 2 s）===")
    br = []
    for brk in BRK_SWEEP:
        p.hold(0.05, 0.0, 2.2, "pre")
        r = p.hold(0.0, brk, 2.0, "brk%.2f" % brk)
        if r:
            br.append((brk, r))
            p.w("刹车 %.2f | 起速 %5.2f | 末速 %5.2f | t90 %5s | 逐帧 %5.1f | 1s窗 %5.1f | 刹距 %4.1f m"
                % (brk, r["v0"], r["v_ss"],
                   ("%.1f" % r["t90"]) if r["t90"] is not None else ">2",
                   r["a1"], r["aw"], r["dist"]))
        p.back(1.4)

    p.w("=== 汇总：油门 ===")
    p.w("%6s %8s %8s %8s %9s %9s %7s" % ("thr", "稳态", "峰值", "t90(s)", "逐帧a", "1s窗a", "路程"))
    for thr, r in hi:
        p.w("%6.2f %8.2f %8.2f %8s %9.1f %9.1f %7.1f"
            % (thr, r["v_ss"], r["v_max"],
               ("%.1f" % r["t90"]) if r["t90"] is not None else ">1.5", r["a1"], r["aw"], r["dist"]))
    p.w("=== 汇总：刹车 ===")
    p.w("%6s %8s %8s %8s %9s %9s %7s" % ("brk", "起速", "末速", "t90(s)", "逐帧a", "1s窗a", "刹距"))
    for brk, r in br:
        p.w("%6.2f %8.2f %8.2f %8s %9.1f %9.1f %7.1f"
            % (brk, r["v0"], r["v_ss"],
               ("%.1f" % r["t90"]) if r["t90"] is not None else ">2", r["a1"], r["aw"], r["dist"]))
    try:
        veh = VehicleControl(throttle=0.0, brake=1.0, steering=0.0)
        veh.gear = GearMode.PARKING
        api.set_vehicle_controls({car: veh})
    except Exception:                                          # noqa: BLE001
        pass
    p.log.close()


if __name__ == "__main__":
    main()
