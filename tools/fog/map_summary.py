# -*- coding: utf-8 -*-
"""把地图整理成人能读的结构：主干道 / 竖向连接 / 贴路障碍 / 关键点关系。"""

import json
import math
import sys
from collections import defaultdict

d = json.load(open(sys.argv[1], encoding="utf-8"))
roads = d["roads"]
objs = d["static_objects"]

KEY = {"Berth_A": (631.0, -66.0), "Berth_B": (834.5, -66.0),
       "Gate": (728.8, -20.0), "Signal_A": (587.5, 124.8),
       "Roadblock(训练)": (766.5, -88.2)}

horiz, vert = [], []
for r in roads:
    (x0, y0), (x1, y1) = r["begin"], r["end"]
    L = math.hypot(x1 - x0, y1 - y0)
    if L < 8:
        continue
    if abs(y1 - y0) < 1.5 and L > 25:
        horiz.append((round((y0 + y1) / 2, 1), round(min(x0, x1), 1), round(max(x0, x1), 1), L, r["id"]))
    elif abs(x1 - x0) < 1.5 and L > 15:
        vert.append((round((x0 + x1) / 2, 1), round(min(y0, y1), 1), round(max(y0, y1), 1), L, r["id"]))

print("=== 东西向主干道（按 y 归类）===")
by_y = defaultdict(list)
for y, xa, xb, L, rid in horiz:
    by_y[y].append((xa, xb, L, rid))
for y in sorted(by_y, reverse=True):
    segs = by_y[y]
    total = sum(s[2] for s in segs)
    span = (round(min(s[0] for s in segs), 1), round(max(s[1] for s in segs), 1))
    print("  y=%7.1f  %2d 段 共 %5.0f m  覆盖 x %s" % (y, len(segs), total, span))

print()
print("=== 南北向连接段（按 x 归类，共 %d 条）===" % len(vert))
by_x = defaultdict(list)
for x, ya, yb, L, rid in vert:
    by_x[x].append((ya, yb, L))
for x in sorted(by_x):
    segs = by_x[x]
    span = (round(min(s[0] for s in segs), 1), round(max(s[1] for s in segs), 1))
    print("  x=%7.1f  %2d 段  覆盖 y %s" % (x, len(segs), span))

print()
print("=== 贴着路的静态构件（离最近车道 <= 4 m）===")
segs_all = []
for r in roads:
    (x0, y0), (x1, y1) = r["begin"], r["end"]
    segs_all.append((x0, y0, x1, y1))


def dist_to_seg(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    if L2 < 1e-9:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


near = []
for o in objs:
    x, y = o["pos"]
    dmin = min(dist_to_seg(x, y, *s) for s in segs_all)
    if dmin <= 4.5:
        near.append((round(dmin, 1), o["id"], round(x, 1), round(y, 1), o["size"][0]))
near.sort()
print("  共 %d 个；最近 25 个：" % len(near))
for dd, oid, x, y, L in near[:25]:
    print("    %-32s (%6.1f,%7.1f) 离路 %4.1f m  L=%4.1f" % (oid, x, y, dd, L or 0))

print()
print("=== 关键点相对关系 ===")
pts = list(KEY.items())
for i in range(len(pts)):
    for k in range(i + 1, len(pts)):
        (n1, p1), (n2, p2) = pts[i], pts[k]
        print("  %-16s → %-16s %6.1f m  (Δx=%+6.1f, Δy=%+6.1f)"
              % (n1, n2, math.hypot(p2[0] - p1[0], p2[1] - p1[1]),
                 p2[0] - p1[0], p2[1] - p1[1]))
