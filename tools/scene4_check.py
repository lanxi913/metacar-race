# -*- coding: utf-8 -*-
"""连续场景 4 跑次体检：一次跑完就知道"哪条预测不成立"。

用法（仓库根目录）：

    .\\.venv\\Scripts\\python.exe tools\\scene4_check.py logs\\scene4\\p7

输出：
1. 总帧数 / 场景时长 / ending
2. 逐元素：时长、帧数、`|a|>5` 连续帧事件数（与 `tools/replay_score.py` 同口径）、
   最大横向偏差、刹车帧占比
3. 锁定（hold lock）次数与出现的元素、升挡闸门次数
4. 最后一帧的泊车位姿与车位中心的差

判据口径（与 replay_score 保持一致，见 `docs/ACCEL-BONUS-STUDY.md`）：
平台判的是"加速度事件"，`|Δv/Δt| > 5` 且**连续 ≥2 帧**才算，孤立的单帧不算；
元素 1（场景刚开始）前 0.5 s 豁免。
"""

from __future__ import annotations

import csv
import json
import math
import os
import sys

ACC_LIMIT = 5.0
ACC_MIN_RUN = 2
START_EXEMPT_S = 0.5
DT_GUESS = 0.033


def load_rows(path):
    with open(path, encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def accel_events(rows):
    """返回 [(elem_i, t0, t1, frames, peak)]，只统计连续 >=ACC_MIN_RUN 帧的事件。"""
    out = []
    cur = None
    prev = None
    for r in rows:
        i = int(float(r["elem_i"]))
        t = float(r["sim_t"] or 0.0)
        v = float(r["speed"])
        if prev is not None and prev[0] == i and t > prev[1]:
            dt = t - prev[1]
            if dt > 1e-6:
                a = (v - prev[2]) / dt
                hit = abs(a) > ACC_LIMIT and not (i == 1 and t < START_EXEMPT_S)
                if hit:
                    if cur is None:
                        cur = [i, prev[1], t, 2, a]
                    else:
                        cur[2] = t
                        cur[3] += 1
                        if abs(a) > abs(cur[4]):
                            cur[4] = a
                elif cur is not None:
                    if cur[3] >= ACC_MIN_RUN:
                        out.append(tuple(cur))
                    cur = None
        elif cur is not None:
            if cur[3] >= ACC_MIN_RUN:
                out.append(tuple(cur))
            cur = None
        prev = (i, t, v)
    if cur is not None and cur[3] >= ACC_MIN_RUN:
        out.append(tuple(cur))
    return out


def hold_events(events_path):
    locks = []
    shifts = []
    try:
        with open(events_path, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    e = json.loads(line)
                except ValueError:
                    continue
                kind = e.get("event") or e.get("kind")
                if kind == "hold" and e.get("mode") == "hold":
                    locks.append(e)
                elif kind == "shift_gate":
                    shifts.append(e)
    except FileNotFoundError:
        pass
    return locks, shifts


def main(argv):
    run_dir = argv[1] if len(argv) > 1 else r"logs\scene4\p7"
    rows = load_rows(os.path.join(run_dir, "run.csv"))
    if not rows:
        print("run.csv 是空的")
        return 1
    total_s = len(rows) * DT_GUESS
    summary = {}
    sp = os.path.join(run_dir, "summary.json")
    if os.path.exists(sp):
        try:
            summary = json.load(open(sp, encoding="utf-8"))
        except ValueError:
            summary = {}
    print("=" * 74)
    print("跑次 %s：帧数 %d（≈%.0f s）  ending=%s  route_s=%s"
          % (os.path.basename(run_dir.rstrip("\\/")), len(rows), total_s,
             summary.get("ending", "?"), summary.get("route_s", "?")))
    print("=" * 74)

    ev = accel_events(rows)
    ev_by_elem = {}
    for i, t0, t1, n, peak in ev:
        ev_by_elem.setdefault(i, []).append((t0, t1, n, peak))

    by_elem = {}
    for r in rows:
        i = int(float(r["elem_i"]))
        d = by_elem.setdefault(i, {"n": 0, "name": r["sub_scene"], "lat": 0.0,
                                   "brk": 0, "vmax": 0.0})
        d["n"] += 1
        d["lat"] = max(d["lat"], abs(float(r["lat_err"] or 0)))
        d["vmax"] = max(d["vmax"], float(r["speed"]))
        if float(r["brake"] or 0) > 0.01:
            d["brk"] += 1

    print("%-3s %-14s %7s %7s %6s %8s %7s %7s" %
          ("#", "元素", "时长", "帧数", "事件", "最大横偏", "刹车%", "vmax"))
    for i in sorted(by_elem):
        d = by_elem[i]
        e = ev_by_elem.get(i, [])
        print("%-3d %-14s %6.1fs %7d %6d %8.2f %6.0f%% %7.2f"
              % (i, d["name"], d["n"] * DT_GUESS, d["n"], len(e), d["lat"],
                 100.0 * d["brk"] / d["n"], d["vmax"]))

    print("-" * 74)
    print("加速度事件（|Δv/Δt|>5 连续≥2 帧）合计 %d 段：" % len(ev))
    for i, t0, t1, n, peak in ev[:20]:
        print("   元素 %-2d t=%7.2f~%-7.2f %2d 帧  峰值 %+6.2f" % (i, t0, t1, n, peak))
    if len(ev) > 20:
        print("   …… 另有 %d 段" % (len(ev) - 20))

    locks, shifts = hold_events(os.path.join(run_dir, "events.jsonl"))
    print("-" * 74)
    print("锁定（hold lock）%d 次；升挡闸门 %d 次" % (len(locks), len(shifts)))

    last = rows[-1]
    print("末帧 x=%.2f y=%.2f yaw=%.2f v=%.2f  route_s=%.1f"
          % (float(last["x"]), float(last["y"]), float(last["yaw"]),
             float(last["speed"]), float(last["route_s"])))

    # 预测打分
    print("-" * 74)
    ok = []
    ok.append(("P1 总帧数 ≤30000", len(rows) <= 30000, len(rows)))
    e9 = by_elem.get(9, {}).get("n", 0) * DT_GUESS
    ok.append(("P2 元素9 ≤220 s", e9 <= 220.0, round(e9, 1)))
    ok.append(("P3 锁定 ≤2 次", len(locks) <= 2, len(locks)))
    ok.append(("P5 升挡 ≤2 次", len(shifts) <= 2, len(shifts)))
    ending = str(summary.get("ending", "?"))
    ok.append(("P7 ending 自然结束", ending in ("ended", "hold_done"), ending))
    for name, good, val in ok:
        print("   [%s] %-22s 实测=%s" % ("PASS" if good else "FAIL", name, val))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
