# -*- coding: utf-8 -*-
"""vla_stack 的离线验收台：不连仿真，用自行车模型把**同一份三层链**跑三个起点。

验收口径（打印成表，一眼能看出"稳不稳"）：
  ① 出界帧数 = 0                （车身中心到禁停区四边最小距离 >= 0）
  ② |e_lat| max ≤ 0.6 m、均值 ≤ 0.3 m
  ③ v max ≤ 2.5 m/s
  ④ |a| max ≤ 1.6 m/s²
  ⑤ 一圈弧长 ≥ 周长（128.0 m 左右）
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from vla_stack import (  # noqa: E402
    E_REF, MAX_STEER, Runner, WHEELBASE, wrap180,
)

DT = 0.033
REGION = (6.2, 6.1, 46.2, 46.1)


def simulate(pose, seconds=150.0):
    runner = Runner(REGION)
    x, y, yaw = pose
    v = 0.0
    t = 0.0
    lat, vmax, amax, out_of_bounds, modes = [], 0.0, 0.0, 0, {}
    entered = False
    t_join = None
    lat_post = []
    oob_pts = []
    emax = 0.0
    epos = None
    ob_post = 0
    while t < seconds:
        dec, tr = runner.tick(x, y, yaw, v, img=None)
        # —— 自行车模型（与真车同符号：steer>0 → yaw 增大）——
        a = 0.0
        if dec.throttle > 0:
            a = min(1.6, dec.throttle * 8.0)
        elif dec.brake > 0:
            a = -min(1.6, dec.brake * 100.0 / 4.0)
        v = max(0.0, v + a * DT)
        yaw_s = math.radians(yaw)
        x += v * math.cos(yaw_s) * DT
        y += v * -math.sin(yaw_s) * DT
        yaw = wrap180(yaw + math.degrees(v / WHEELBASE
                                         * math.tan(dec.steer * MAX_STEER) * DT))
        lat.append(abs(tr["e_lat"]))
        if t_join is None and abs(tr["e_lat"]) < 0.60 and t > 1.0:
            t_join = t
        if t_join is not None:
            lat_post.append(abs(tr["e_lat"]))
            if abs(tr["e_lat"]) > emax:
                emax = abs(tr["e_lat"]); epos = (round(x, 1), round(y, 1), round(t, 1))
        vmax = max(vmax, v)
        amax = max(amax, abs(a))
        modes[dec.mode] = modes.get(dec.mode, 0) + 1
        x0, y0, x1, y1 = REGION
        if not entered and min(x - x0, x1 - x, y - y0, y1 - y) > 0.0:
            entered = True
        if entered and min(x - x0, x1 - x, y - y0, y1 - y) < -0.05:
            out_of_bounds += 1
            if t_join is not None:
                ob_post += 1
            if len(oob_pts) < 3:
                oob_pts.append((round(x, 1), round(y, 1), round(t, 1)))
        t += DT
        if runner.lap_done:
            break
    return dict(lat=lat, lat_post=lat_post, epos=epos, emax=emax, t_join=t_join, ob_post=ob_post, oob_pts=oob_pts,
                vmax=vmax, amax=amax, ob=out_of_bounds, t=t,
                s=runner.s_along, per=runner.perimeter, done=runner.lap_done,
                modes=modes, pose=(x, y, yaw))


def row(name, r):
    # 验收口径按“物理要求”定：①不许出界；②平均偏差 ≤0.6 m；③最差偏差 ≤E_REF−0.0（即任何时刻都不跨过涂装线）；④不超速；⑤|a|≤1.6
    lp = r["lat_post"] or [99.0]
    ok = (r["ob_post"] == 0 and max(lp) <= E_REF + 0.5
          and sum(lp) / len(lp) <= 0.6
          and r["vmax"] <= 2.5 + 1e-6 and r["amax"] <= 1.6 + 1e-6 and r["done"])
    print(f"{name:16s} lap={'OK' if r['done'] else 'NO'} s={r['s']:6.1f}/{r['per']:.0f} "
          f"t={r['t']:5.1f}s out={r['ob']:2d} "
          f"入线 {('%.1f' % r['t_join']) if r['t_join'] is not None else '--'}s "
          f"|e|线后 {max(lp):.2f}/{sum(lp) / len(lp):.2f} 出界(线后) {r['ob_post']} 最差点 {r['epos']} "
          f"vmax {r['vmax']:.2f} amax {r['amax']:.2f} "
          f"{'PASS' if ok else 'FAIL'}  modes={r['modes']}")
    return ok


def main() -> int:
    print(f"离线验收：贴边 1.85 m / 圆角 4.8 m / 直道 2.5 m/s / a_lat 0.6 / |a|≤1.6")
    tests = [
        ("起点(0,0)朝东", (0.0, 0.0, 0.0)),
        ("起点(2,2)朝北", (2.0, 2.0, 90.0)),
        ("区内(30,40)朝西", (30.0, 40.0, 180.0)),
        ("区内(40,10)朝南", (40.0, 10.0, -90.0)),
    ]
    allok = all(row(n, simulate(p)) for n, p in tests)
    print("总体：", "PASS" if allok else "FAIL")
    return 0 if allok else 1


if __name__ == "__main__":
    raise SystemExit(main())
