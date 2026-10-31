# -*- coding: utf-8 -*-
"""油门扫描：0~1 的油门到底怎么映射到速度/加速度。

从静止开始，每个油门值保持 4 s，记录：
  - 稳态速度（最后 1 s 的平均）
  - 到 90% 稳态所用的时间
  - 峰值逐帧加速度
  - 平台回显的 control.throttle（看它有没有被量化）
每个油门段之间用刹车 1.0 停稳。
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

SWEEP = [0.01, 0.02, 0.03, 0.05]


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
    log = open(os.path.join(args.out, "throttle.log"), "w", encoding="utf-8")

    def w(msg):
        line = "[%6.2f] %s" % (time.time() - t0, msg)
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

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
    w("油门扫描：%s，其余车驻车" % car)

    def sample(thr, brk, hold_s):
        """保持一段控制量，返回 (采样列表, 回显列表)。"""
        rows, echo = [], []
        t_end = time.time() + hold_s
        while time.time() < t_end:
            veh = VehicleControl(throttle=thr, brake=brk, steering=0.0)
            try:
                veh.gear = GearMode.DRIVE
            except Exception:                                  # noqa: BLE001
                pass
            try:
                api.set_vehicle_controls({car: veh})
                st = plain(api.get_vehicle_state(car))
            except Exception as exc:                           # noqa: BLE001
                w("  控制失败 %s" % exc)
                break
            sp = float(st.get("speed_mps") or 0.0)
            ctl = st.get("control") or {}
            if isinstance(ctl, str):
                ctl = json.loads(ctl) if ctl.strip().startswith("{") else {}
            rows.append((time.time() - t0, sp))
            echo.append(float(ctl.get("throttle") or 0.0))
            time.sleep(0.1)
        return rows, echo

    results = []
    for thr in SWEEP:
        # 先停稳
        sample(0.0, 1.0, 1.5)
        rows, echo = sample(thr, 0.0, 2.5)
        if not rows:
            continue
        sp = [r[1] for r in rows]
        tail = sp[-10:] if len(sp) >= 10 else sp
        v_ss = sum(tail) / len(tail)
        v_max = max(sp)
        t90 = None
        for i, v in enumerate(sp):
            if v_ss > 0.2 and v >= 0.9 * v_ss:
                t90 = rows[i][0] - rows[0][0]
                break
        peak = 0.0
        for i in range(1, len(sp)):
            dt = max(rows[i][0] - rows[i - 1][0], 1e-3)
            peak = max(peak, abs(sp[i] - sp[i - 1]) / dt)
        echo_mean = sum(echo) / len(echo) if echo else 0.0
        results.append((thr, echo_mean, v_ss, v_max, t90, peak))
        w("油门 %.2f | 回显 %.3f | 稳态 %.2f m/s | 峰值 %.2f | 到90%%用 %s s | 峰值加速度 %.1f"
          % (thr, echo_mean, v_ss, v_max, ("%.1f" % t90) if t90 is not None else ">4",
             peak))

    w("=== 汇总表 ===")
    w("%6s %8s %8s %8s %8s %8s" % ("thr", "回显", "稳态m/s", "峰值m/s", "t90(s)", "峰加(m/s2)"))
    for r in results:
        w("%6.2f %8.3f %8.2f %8.2f %8s %8.1f"
          % (r[0], r[1], r[2], r[3], ("%.1f" % r[4]) if r[4] is not None else ">4", r[5]))
    try:
        veh = VehicleControl(throttle=0.0, brake=1.0, steering=0.0)
        veh.gear = GearMode.PARKING
        api.set_vehicle_controls({car: veh})
    except Exception:                                          # noqa: BLE001
        pass
    log.close()


if __name__ == "__main__":
    main()
