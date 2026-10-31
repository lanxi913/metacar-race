# -*- coding: utf-8 -*-
"""分析 trace.csv：量化"一扭一扭"（转向抖动 / 横向摆动）。"""
import csv
import math
import os
import sys
from collections import defaultdict

path = sys.argv[1]
rows = defaultdict(list)
with open(path, encoding="utf-8") as fh:
    for r in csv.DictReader(fh):
        try:
            rows[r["vid"]].append((float(r["t"]), float(r["x"]), float(r["y"]),
                                   float(r["steer"]), float(r["thr"]),
                                   float(r["brk"]), float(r["speed"])))
        except (TypeError, ValueError):
            continue

print("vid     n     路程m   均速   转向过零/100m  转向|均值|  |Δsteer|/帧  横向摆动std(m)")
for vid in sorted(rows):
    seq = rows[vid]
    if len(seq) < 50:
        continue
    dist = 0.0
    for i in range(1, len(seq)):
        dist += math.hypot(seq[i][1] - seq[i - 1][1], seq[i][2] - seq[i - 1][2])
    v_mean = sum(r[6] for r in seq) / len(seq)
    # 转向过零：只在 |steer|>0.03 时计，避免零附近的抖噪
    signs = [1 if r[3] > 0.03 else (-1 if r[3] < -0.03 else 0) for r in seq]
    zc = 0
    last = 0
    for sgn in signs:
        if sgn == 0:
            continue
        if last and sgn != last:
            zc += 1
        last = sgn
    # 横向摆动：20 m 窗口内，把轨迹投到"首尾连线"上算垂距的 std
    lat = []
    win = []
    acc = 0.0
    for i in range(1, len(seq)):
        win.append(seq[i])
        acc += math.hypot(seq[i][1] - seq[i - 1][1], seq[i][2] - seq[i - 1][2])
        if acc >= 20.0 and len(win) > 10:
            x0, y0 = win[0][1], win[0][2]
            x1, y1 = win[-1][1], win[-1][2]
            L = math.hypot(x1 - x0, y1 - y0)
            if L > 15.0:
                ds = [abs((x1 - x0) * (y0 - p[2]) - (x0 - p[1]) * (y1 - y0)) / L for p in win]
                m = sum(ds) / len(ds)
                lat.append(math.sqrt(sum((d - m) ** 2 for d in ds) / len(ds)))
            win = []
            acc = 0.0
    dsteer = [abs(seq[i][3] - seq[i - 1][3]) for i in range(1, len(seq))]
    lat_std = sorted(lat)[len(lat) // 2] if lat else 0.0
    print("%-7s %5d %7.1f %6.2f %14.1f %12.3f %12.4f %16.3f"
          % (vid, len(seq), dist, v_mean,
             zc * 100.0 / max(dist, 1e-6),
             sum(abs(r[3]) for r in seq) / len(seq),
             sum(dsteer) / max(len(dsteer), 1),
             lat_std))

print()
print("=== 抽样：Car_C 的一段直行（前 3 s，每 0.1 s）===")
seq = rows.get("Car_C") or []
start = None
for i in range(len(seq) - 40):
    seg = seq[i:i + 30]
    if all(abs(s[3]) < 0.08 for s in seg) and all(s[6] > 3.5 for s in seg):
        start = i
        break
if start is not None:
    for s in seq[start:start + 30]:
        print("t=%6.1f  steer=%+.3f  thr=%.2f  brk=%.2f  v=%.2f" % (s[0], s[3], s[4], s[5], s[6]))
