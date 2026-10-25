# -*- coding: utf-8 -*-
"""压线离线复核：把某场景 mapd 里的车道边界线读出来，再拿逐帧 CSV 算车身离边界的最小距离。

用法:
  python line_check.py --scene chelianghuiru --csv metacar-race/logs/mer_r3.csv

输出：全程 min(车身四角到边界线距离)、min(车身中心到边界线距离)、以及最危险的那几帧。
判据：平台扣「压道路边线」时，四角最小距离一定接近 0；没扣时留有一定余量。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os

HALF_LEN = 2.0      # 车身长 4.0 m
HALF_WID = 0.905    # 车身宽 1.81 m


def load_borders(mapd_path):
    d = json.load(open(mapd_path, encoding="utf-8"))
    lines = []
    for road in d:
        for lane in road.get("laneData") or []:
            for key in ("LeftBorder", "RightBorder", "CenterBorder"):
                b = lane.get(key)
                if not b:
                    continue
                pts = [(p["x"], p["y"]) for p in (b.get("pathPoint") or [])]
                if len(pts) >= 2:
                    lines.append(pts)
    return lines


def pt_seg_dist(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def pt_lines_dist(p, lines, bbox_pad=6.0):
    best = 1e9
    px, py = p
    for pts in lines:
        xs = [q[0] for q in pts]
        ys = [q[1] for q in pts]
        if px < min(xs) - bbox_pad or px > max(xs) + bbox_pad:
            continue
        if py < min(ys) - bbox_pad or py > max(ys) + bbox_pad:
            continue
        for i in range(len(pts) - 1):
            d = pt_seg_dist(p, pts[i], pts[i + 1])
            if d < best:
                best = d
    return best


def corners(x, y, yaw_deg):
    psi = math.radians(yaw_deg)
    hx, hy = math.cos(psi), -math.sin(psi)   # 车头方向
    lx, ly = math.sin(psi), math.cos(psi)    # 车左方向
    out = []
    for a, b in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        out.append((x + a * HALF_LEN * hx + b * HALF_WID * lx,
                    y + a * HALF_LEN * hy + b * HALF_WID * ly))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True, help="场景包目录名，如 chelianghuiru")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--root", default=r"D:\无人算法\场景包\scenes")
    args = ap.parse_args()

    mapd = os.path.join(args.root, args.scene,
                        "SmartCar_Data", "StreamingAssets", "mapfile", "mapd")
    lines = load_borders(mapd)
    print(f"borders: {len(lines)} 条")
    rows = list(csv.DictReader(open(args.csv, encoding="utf-8")))
    print(f"csv frames: {len(rows)}")

    worst = []
    for row in rows:
        x = float(row["x"])
        y = float(row["y"])
        yaw = float(row["yaw"])
        dc = pt_lines_dist((x, y), lines)
        dk = min(pt_lines_dist(c, lines) for c in corners(x, y, yaw))
        worst.append((dk, dc, float(row["t"]), x, y))
    worst.sort()
    print("min corner dist: %.3f m" % worst[0][0])
    print("min center dist: %.3f m" % min(w[1] for w in worst))
    for w in worst[:6]:
        print("  t=%5.2fs corner=%.3f center=%.3f pos=(%.2f,%.2f)" % (w[2], w[0], w[1], w[3], w[4]))


if __name__ == "__main__":
    main()
