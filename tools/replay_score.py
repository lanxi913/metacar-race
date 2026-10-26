# -*- coding: utf-8 -*-
"""离线判分回放器：吃 `run.csv`，按平台实测判据给出「逐元素得分预测」。

为什么需要它
------------
连续场景 4 一趟要 15~20 分钟（仿真比实时慢约 4.8 倍），而且只有"平台自己结束
场景"的那一趟才记分。靠真跑来验证改动 ⇒ 只能"攒着一起改" ⇒ 失去单变量归因能力
（2026-10-25 就是这么把"超车的 +10"改丢的）。这个脚本把验证从 20 分钟降到 1 秒。

判据来源（全部有实测出处，改之前先读 `docs/ACCEL-BONUS-STUDY.md`）
--------------------------------------------------------------
1. `+10 加速度控制`：**逐帧速度差 / dt ≤ 5 m/s²**，且**孤立单帧尖峰不算**
   （≥2 连续帧才算一次事件）。校准证据（p2/p4 共 24 个元素）：
   - p4 环岛在 t=15.87 s 有一帧 5.64 m/s²，平台照样给 +10；
   - p4 超车在 t=27.22~27.92 s 连续 11 帧 >5（峰值 8.48），平台不给。
   - p2/p4 其余 22 个元素全部一致。
   注意：`thr→0` 的收油阶跃、`v>v_tgt` 的超速帧都**不影响**加分（p2 环岛/转弯/
   匝道有几十上百帧超速，照样 +10）。
2. `-20 压道路边线`：车身四角到 **实线**（mapd `borderType` 1/2）的最小余量 < 0。
   虚线（`borderType` 6）可以跨。
3. 场景时间 < 0.5 s 的帧不计（只有元素 1 处在场景开头）。

用法
----
    .\\.venv\\Scripts\\python.exe tools\\replay_score.py logs\\scene4\\p4\\run.csv
    .\\.venv\\Scripts\\python.exe tools\\replay_score.py logs\\scene4\\p4\\run.csv --lines
"""
from __future__ import annotations

import argparse
import csv
import io
import math
import os
from collections import defaultdict

DEFAULT_SCENE_ROOT = r"D:\无人算法\场景包\scenes"
SOLID_BORDER_TYPES = (1, 2)      # 实测：1/2 是道路边线（实线），6 是车道分界（虚线）
EGO_LEN = 4.70
EGO_WID = 1.90
ACC_LIMIT = 5.0                  # 平台 |a| 预算
ACC_MIN_RUN = 2                  # 连续多少帧才算一次事件（单帧尖峰豁免）
START_EXEMPT_S = 0.5             # 场景开头的豁免窗口
BASE = {                         # 各元素基础分（平台明细实测）
    "车辆直行": 100.0, "连续转弯": 100.0, "环岛驶入、驶出": 100.0,
    "车辆跟随": 100.0, "十字路口通行": 200.0, "复杂路况超车": 200.0,
    "匝道行驶": 100.0, "车辆紧急停止": 100.0, "辅路驶入、驶出": 100.0,
    "车辆变道": 100.0, "车辆切出": 100.0, "平行车道泊车": 200.0,
}


# ---------------------------------------------------------------- csv 读取
def load_rows(path):
    rows = []
    with io.open(path, encoding="utf-8-sig", newline="") as f:
        for x in csv.DictReader(f):
            rows.append(dict(
                frame=int(x["frame"]),
                t=float(x["sim_t"]),
                name=x["sub_scene"],
                elem=int(x["elem_i"]),
                x=float(x["x"]), y=float(x["y"]), yaw=float(x["yaw"]),
                v=float(x["speed"]),
                thr=float(x["throttle"]), brk=float(x["brake"]),
                tgt=float(x["v_tgt"]),
            ))
    return rows


# ---------------------------------------------------------------- 加速度事件
def accel_events(rows):
    """逐元素找 |Δv/Δt| > ACC_LIMIT 且连续 ≥ACC_MIN_RUN 帧的事件。"""
    per = defaultdict(list)
    for r in rows:
        per[r["elem"]].append(r)
    out = {}
    for elem, seq in per.items():
        hits = []                                   # [(t, a)]
        for i in range(1, len(seq)):
            a, b = seq[i - 1], seq[i]
            dt = b["t"] - a["t"]
            if dt <= 0:
                continue
            if elem == 1 and b["t"] < START_EXEMPT_S:  # 场景开头豁免
                continue
            acc = (b["v"] - a["v"]) / dt
            hits.append((b["t"], acc))
        events = []
        run = []
        for t, acc in hits:
            if abs(acc) > ACC_LIMIT:
                run.append((t, acc))
            else:
                if len(run) >= ACC_MIN_RUN:
                    events.append(dict(kind="accel", t0=run[0][0], t1=run[-1][0],
                                       frames=len(run),
                                       peak=max(run, key=lambda z: abs(z[1]))[1]))
                run = []
        if len(run) >= ACC_MIN_RUN:
            events.append(dict(kind="accel", t0=run[0][0], t1=run[-1][0],
                               frames=len(run),
                               peak=max(run, key=lambda z: abs(z[1]))[1]))
        out[elem] = events
    return out


# ---------------------------------------------------------------- 压线判定
def _load_points(path):
    if not os.path.isfile(path):
        return []
    import json
    with io.open(path, encoding="utf-8") as f:
        data = json.load(f)
    pts = data["pathPoint"] if isinstance(data, dict) and "pathPoint" in data else data
    return [_xy_of(p) for p in pts]


def _xy_of(p):
    """mapd 的 pathPoint 有三种写法（dict / [x,y] / 带 .x 的对象），统一取出 (x, y)。"""
    if hasattr(p, "x"):
        return float(p.x), float(p.y)
    if isinstance(p, dict):
        return float(p["x"]), float(p["y"])
    return float(p[0]), float(p[1])


def load_solid_segments(mapd_path):
    import json
    segs = []
    with io.open(mapd_path, encoding="utf-8") as f:
        roads = json.load(f)
    for road in roads:
        for lane in road.get("laneData", []) or []:
            for side in ("LeftBorder", "RightBorder"):
                b = lane.get(side) or {}
                if int(b.get("borderType", 0)) not in SOLID_BORDER_TYPES:
                    continue
                pts = [_xy_of(p) for p in (b.get("pathPoint", []) or [])]
                for i in range(1, len(pts)):
                    segs.append((pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1]))
    return segs


class _Grid:
    """把线段按 4 m 网格分桶，避免几万段逐帧全算。"""

    def __init__(self, segs, cell=4.0):
        self.cell = cell
        self.bins = defaultdict(list)
        for s in segs:
            x0, y0, x1, y1 = s
            for gx in range(int(min(x0, x1) // cell), int(max(x0, x1) // cell) + 1):
                for gy in range(int(min(y0, y1) // cell), int(max(y0, y1) // cell) + 1):
                    self.bins[(gx, gy)].append(s)

    def near(self, x, y, radius=6.0):
        c = self.cell
        out = []
        for gx in range(int((x - radius) // c), int((x + radius) // c) + 1):
            for gy in range(int((y - radius) // c), int((y + radius) // c) + 1):
                out.extend(self.bins.get((gx, gy), ()))
        return out


def _dist_point_seg(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    if L2 <= 1e-12:
        return math.hypot(px - x0, py - y0)
    t = ((px - x0) * dx + (py - y0) * dy) / L2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def line_events(rows, segs, margin=0.0):
    """车身四角到实线的最小余量，逐元素返回最小的若干次越线。"""
    grid = _Grid(segs)
    per = defaultdict(list)
    for r in rows:
        per[r["elem"]].append(r)
    out = {}
    cache = {}
    for elem, seq in per.items():
        hits = []
        for r in seq:
            key = (round(r["x"], 2), round(r["y"], 2), round(r["yaw"], 1))
            if key in cache:
                d = cache[key]
            else:
                hx = math.cos(math.radians(r["yaw"]))
                hy = -math.sin(math.radians(r["yaw"]))
                lx, ly = -hy, hx                      # 左法向
                hl, hw = EGO_LEN / 2.0, EGO_WID / 2.0
                corners = []
                for a, b in ((hl, hw), (hl, -hw), (-hl, hw), (-hl, -hw)):
                    corners.append((r["x"] + hx * a + lx * b,
                                    r["y"] + hy * a + ly * b))
                d = 1e9
                for cx, cy in corners:
                    for s in grid.near(cx, cy):
                        dd = _dist_point_seg(cx, cy, *s)
                        if dd < d:
                            d = dd
                cache[key] = d
            if d < margin:
                hits.append((r["t"], d))
        if hits:
            worst = min(hits, key=lambda z: z[1])
            out[elem] = [dict(kind="line", t0=worst[0], t1=hits[-1][0],
                              frames=len(hits), margin=worst[1])]
        else:
            out[elem] = []
    return out


# ---------------------------------------------------------------- 汇总
def verdicts(rows, accel_ev, line_ev):
    names = {}
    for r in rows:
        names[r["elem"]] = r["name"]
    out = []
    for elem in sorted(names):
        name = names[elem]
        base = BASE.get(name, 100.0)
        ev = list(accel_ev.get(elem, [])) + list(line_ev.get(elem, []))
        bonus = 10.0
        notes = []
        for e in ev:
            if e["kind"] == "accel":
                bonus = 0.0
                notes.append("加速度超限 t=%.2f~%.2fs(%d帧 峰值%+.2f)"
                             % (e["t0"], e["t1"], e["frames"], e["peak"]))
            elif e["kind"] == "line":
                notes.append("压线 t=%.2f 余量%.2fm" % (e["t0"], e["margin"]))
        penalty = 0.0
        for e in ev:
            if e["kind"] == "line":
                penalty += 20.0
        out.append(dict(elem=elem, name=name, base=base, bonus=bonus,
                        penalty=penalty, verdict=base + bonus - penalty,
                        notes=notes))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_path")
    ap.add_argument("--mapd", default=None,
                    help="mapd 路径；默认按 route 指纹去场景包目录里找")
    ap.add_argument("--scene-root", default=DEFAULT_SCENE_ROOT)
    ap.add_argument("--lines", action="store_true", help="同时做压线几何复核（慢）")
    ap.add_argument("--route", default=None, help="用于指纹认场景的 route 路径")
    args = ap.parse_args()

    rows = load_rows(args.csv_path)
    accel_ev = accel_events(rows)

    line_ev = {}
    if args.lines:
        mapd = args.mapd
        if mapd is None:
            from metacar_race.scene4.parkslot import find_mapd
            route = []
            if args.route:
                route = _load_points(args.route)
            else:
                # 用 run.csv 的轨迹当指纹
                route = [(r["x"], r["y"]) for r in rows[::max(1, len(rows) // 2000)]]
            mapd = find_mapd(route, args.scene_root, tol=None)
        if not mapd:
            print("[replay] 找不到 mapd，跳过压线复核")
        else:
            segs = load_solid_segments(mapd)
            print("[replay] mapd=%s 实线段 %d 条" % (mapd, len(segs)))
            line_ev = line_events(rows, segs)

    res = verdicts(rows, accel_ev, line_ev)
    total = 0.0
    print("%-3s %-16s %5s %5s %6s %7s  %s" % ("#", "元素", "基础", "加分", "扣分", "预测", "事件"))
    for r in res:
        total += r["verdict"]
        print("%-3d %-16s %5.0f %5.0f %6.0f %7.2f  %s"
              % (r["elem"], r["name"], r["base"], r["bonus"], r["penalty"],
                 r["verdict"], "; ".join(r["notes"]) or "-"))
    print("%-20s %28.2f" % ("合计", total))


if __name__ == "__main__":
    main()
