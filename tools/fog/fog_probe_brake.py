# -*- coding: utf-8 -*-
"""刹车响应曲线（只测刹车，往返式）。

每档：油门 0.05 加速 2.0 s（到 ~5 m/s）→ 施加刹车 2.0 s → 倒车回位 → 停。
统计：起速、末速、刹停用时、逐帧减速度峰值、**按时间取的 1 秒窗减速度峰值**、刹距。
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

BRK = [0.05, 0.10, 0.15, 0.20, 0.30, 0.50, 0.75, 1.00]


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    log = open(os.path.join(args.out, "brake.log"), "w", encoding="utf-8")

    api = connect_retry()
    t0 = time.time()
    car = "Car_C"
    for other in api.get_vehicle_ids():
        if other != car:
            try:
                veh = VehicleControl(throttle=0.0, brake=1.0, steering=0.0)
                veh.gear = GearMode.PARKING
                api.set_vehicle_controls({other: veh})
            except Exception:                                  # noqa: BLE001
                pass

    ev_seq = [0]

    def w(msg):
        line = "[%6.2f] %s" % (time.time() - t0, msg)
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

    def events():
        got = []
        try:
            evs = plain(api.get_events(ev_seq[0]))
            if isinstance(evs, dict):
                evs = evs.get("events") or []
            for e in evs or []:
                d = e if isinstance(e, dict) else plain(e)
                if not isinstance(d, dict):
                    continue
                s = int(d.get("sequence") or 0)
                if s > ev_seq[0]:
                    ev_seq[0] = s
                if (d.get("event_type") or "") in ("violation", "warning"):
                    got.append(str(d.get("message"))[:60])
                    w("   !! EVENT %s" % str(d.get("message"))[:70])
        except Exception:                                      # noqa: BLE001
            pass
        return got

    def hold(thr, brk, sec, gear=None):
        rows, evs = [], []
        t_end = time.time() + sec
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
                w("   控制失败 %s" % exc)
                break
            pose = st.get("pose") or {}
            px = py = 0.0
            if isinstance(pose, dict):
                px = float(pose.get("pos_x") or 0.0)
                py = float(pose.get("pos_y") or 0.0)
            rows.append((time.time(), float(st.get("speed_mps") or 0.0), px, py))
            evs.extend(events())
            time.sleep(0.08)
        return rows, evs

    def stats(rows):
        if not rows:
            return None
        ts = [r[0] - rows[0][0] for r in rows]
        sp = [r[1] for r in rows]
        a1 = 0.0
        for i in range(1, len(sp)):
            a1 = max(a1, abs(sp[i] - sp[i - 1]) / max(ts[i] - ts[i - 1], 1e-3))
        aw = 0.0
        for i in range(len(sp)):
            j = i
            while j > 0 and (ts[i] - ts[j]) < 1.0:
                j -= 1
            if j == i or (ts[i] - ts[j]) < 0.8:
                continue
            aw = max(aw, abs(sp[i] - sp[j]) / max(ts[i] - ts[j], 1e-3))
        t_stop = None
        for i, v in enumerate(sp):
            if abs(v) < 0.15:
                t_stop = ts[i]
                break
        dist = 0.0
        for i in range(1, len(rows)):
            dist += ((rows[i][2] - rows[i - 1][2]) ** 2
                     + (rows[i][3] - rows[i - 1][3]) ** 2) ** 0.5
        return dict(v0=sp[0], v_end=sp[-1], v_max=max(sp), t_stop=t_stop,
                    a1=a1, aw=aw, dist=dist, n=len(sp))

    res = []
    w("=== 刹车扫描（%s）===" % car)
    for brk in BRK:
        hold(0.05, 0.0, 2.0)                       # 加速
        pre, _ = hold(0.05, 0.0, 0.3)              # 记录起速
        rows, evs = hold(0.0, brk, 2.0)            # 施加刹车
        s = stats(rows)
        v0 = pre[-1][1] if pre else (s["v0"] if s else 0.0)
        if s:
            res.append((brk, v0, s, evs))
            w("刹车 %.2f | 起速 %5.2f | 末速 %5.2f | 刹停 %5s | 逐帧 %5.1f | 1s窗 %5.1f | 刹距 %4.2f m"
              % (brk, v0, s["v_end"],
                 ("%.1f" % s["t_stop"]) if s["t_stop"] is not None else ">2",
                 s["a1"], s["aw"], s["dist"]))
        hold(0.05, 0.0, 1.8, gear=GearMode.REVERSE)   # 倒车回位
        hold(0.0, 1.0, 1.0)                            # 停

    w("=== 汇总：刹车 ===")
    w("%6s %8s %8s %8s %9s %9s %7s" % ("brk", "起速", "末速", "刹停(s)", "逐帧a", "1s窗a", "刹距"))
    for brk, v0, s, _ in res:
        w("%6.2f %8.2f %8.2f %8s %9.1f %9.1f %7.2f"
          % (brk, v0, s["v_end"],
             ("%.1f" % s["t_stop"]) if s["t_stop"] is not None else ">2",
             s["a1"], s["aw"], s["dist"]))
    try:
        veh = VehicleControl(throttle=0.0, brake=1.0, steering=0.0)
        veh.gear = GearMode.PARKING
        api.set_vehicle_controls({car: veh})
    except Exception:                                          # noqa: BLE001
        pass
    log.close()


if __name__ == "__main__":
    main()
