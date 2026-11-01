# -*- coding: utf-8 -*-
"""港口迷雾路由 v2：用 lane 的 predecessor 建连通图，Dijkstra 出航点。

mapd 里每条 lane：
  id / beginPos / endPos / drivingType / predecessor[ids] / successor 可能没有
用 predecessor 反推：pred -> lane 是一条有向边，权重取 lane 长度。
再用几何近邻（端点相距 < 3 m）补充连边，防止链断。
"""
from __future__ import annotations

import heapq
import json
import math
from collections import defaultdict

MAPD = (r"C:\Users\lanxi\AppData\Roaming\com.i-mylab.metapower-desktop"
        r"\car\scenes\PORT-FOG-01\SmartCar_Data\StreamingAssets\mapfile\mapd")


class RoadMap:
    def __init__(self, path: str = MAPD, snap: float = 3.0) -> None:
        lanes = json.load(open(path, encoding="utf-8"))
        self.lanes = []
        self.index = {}
        for ln in lanes:
            b, e = ln.get("beginPos") or {}, ln.get("endPos") or {}
            a = (float(b.get("x", 0)), float(b.get("y", 0)))
            c = (float(e.get("x", 0)), float(e.get("y", 0)))
            rec = {"id": ln.get("id"), "a": a, "b": c,
                   "len": math.hypot(c[0] - a[0], c[1] - a[1]),
                   "pred": [p for p in (ln.get("predecessor") or [])]}
            self.index[rec["id"]] = len(self.lanes)
            self.lanes.append(rec)

        self.adj = defaultdict(list)
        for i, ln in enumerate(self.lanes):
            for pid in ln["pred"]:
                j = self.index.get(pid)
                if j is not None:
                    self.adj[j].append((i, ln["len"]))

        # 几何补充：i 的终点接近 j 的起点就连
        starts = [(i, ln["a"]) for i, ln in enumerate(self.lanes)]
        for i, ln in enumerate(self.lanes):
            ex, ey = ln["b"]
            for j, (sx, sy) in starts:
                if j == i:
                    continue
                if math.hypot(sx - ex, sy - ey) <= snap:
                    if all(t[0] != j for t in self.adj[i]):
                        self.adj[i].append((j, self.lanes[j]["len"]))

    def nearest_lane(self, xy) -> tuple[int, float]:
        best, bd = 0, 1e18
        for i, ln in enumerate(self.lanes):
            ax, ay = ln["a"]
            bx, by = ln["b"]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((xy[0] - ax) * vx + (xy[1] - ay) * vy) / L2))
            px, py = ax + t * vx, ay + t * vy
            d = math.hypot(xy[0] - px, xy[1] - py)
            if d < bd:
                bd, best = d, i
        return best, bd

    def route(self, start_xy, goal_xy):
        s, _ = self.nearest_lane(start_xy)
        g, _ = self.nearest_lane(goal_xy)
        dist = {s: 0.0}
        prev = {}
        pq = [(0.0, s)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, 1e18) + 1e-9:
                continue
            if u == g:
                break
            for v, w in self.adj.get(u, []):
                nd = d + w
                if nd < dist.get(v, 1e18) - 1e-9:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if g not in dist:
            return [], float("inf")
        chain = [g]
        while chain[-1] != s and chain[-1] in prev:
            chain.append(prev[chain[-1]])
        chain.reverse()
        pts = [self.lanes[i]["a"] for i in chain] + [self.lanes[chain[-1]]["b"]]
        total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                    for i in range(len(pts) - 1))
        return pts, total


if __name__ == "__main__":
    import sys

    rm = RoadMap()
    conn = sum(len(v) for v in rm.adj.values())
    print(f"lanes={len(rm.lanes)} 有向边={conn}")
    x, y = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) > 2 else (618.5, 120.8)
    tx, ty = (float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (631.0, -66.0)
    pts, total = rm.route((x, y), (tx, ty))
    print(f"({x:.0f},{y:.0f}) -> ({tx:.0f},{ty:.0f}): 航点 {len(pts)} 个, 总长 {total:.0f} m")
    for p in pts[:18]:
        print(f"   ({p[0]:7.1f},{p[1]:7.1f})")
    if len(pts) > 18:
        print("   ...")
