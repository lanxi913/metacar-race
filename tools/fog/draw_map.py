# -*- coding: utf-8 -*-
"""把 map_dump.json 画成 ASCII 地图 + 关键地物清单。"""

import json
import math
import sys

p = sys.argv[1]
d = json.load(open(p, encoding="utf-8"))

roads = d["roads"]
objs = d["static_objects"]
task = d.get("task") or {}
spawn = d.get("spawn") or {}

X0, X1, Y0, Y1 = 555.0, 910.0, -105.0, 145.0
SX, SY = 3.6, 4.6          # 每字符多少米
W = int((X1 - X0) / SX) + 1
H = int((Y1 - Y0) / SY) + 1
grid = [[" "] * W for _ in range(H)]


def put(x, y, ch):
    ix = int((x - X0) / SX)
    iy = int((Y1 - y) / SY)
    if 0 <= ix < W and 0 <= iy < H:
        grid[iy][ix] = ch


def line(a, b, ch):
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 1.5))
    for i in range(n + 1):
        t = i / n
        put(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, ch)


# 路网
for r in roads:
    line(r["begin"], r["end"], "·")

# 静态构件（只看贴近路两侧的）
big = []
for o in objs:
    x, y = o["pos"]
    if not (X0 <= x <= X1 and Y0 <= y <= Y1):
        continue
    put(x, y, "#")
    L = o["size"][0] or 0
    if L >= 6:
        big.append((L, o["id"], x, y))

# 关键点
marks = {
    "1": (587.5, 124.8),      # Signal_A（任务对象里读到的）
    "2": (728.8, -20.0),      # 闸口
    "A": (631.0, -66.0),      # Berth_A
    "B": (834.5, -66.0),      # Berth_B
    "R": (766.5, -88.2),      # Roadblock_East（训练场景实测位置）
}
for ch, (x, y) in marks.items():
    put(x, y, ch)
for vid, xy in spawn.items():
    put(xy[0], xy[1], "c")

print("=" * W)
for row in grid:
    print("".join(row).rstrip())
print("=" * W)
print("图例: · 路网   # 静态构件   1 Signal_A   2 闸口   A/B 泊位   R 路障   c 车辆起点")
print("比例: 每字符 %.1f m(横) × %.1f m(纵)" % (SX, SY))
print()
print("=== 尺寸 >= 6 m 的静态构件（按距离西侧泊位区排序）===")
big.sort()
for L, oid, x, y in big[:24]:
    print("  %-34s L=%5.1f  (%6.1f, %6.1f)" % (oid, L, x, y))
print()
print("=== 任务对象 ===")
for o in task.get("objects") or []:
    print("  %-12s %-20s (%6.1f, %6.1f)" % (o["id"], o["type"], o["pos"][0], o["pos"][1]))
print()
print("=== 车辆起点 ===")
for vid, xy in spawn.items():
    print("  %-8s (%6.1f, %6.1f)" % (vid, xy[0], xy[1]))
