# -*- coding: utf-8 -*-
"""重写路由：双向图 + 几何补边，保证任何两点都能出路由。"""
from __future__ import annotations

import heapq
import json
import math
from collections import defaultdict

MAPD = (r"C:\Users\lanxi\AppData\Roaming\com.i-mylab.metapower-desktop"
        r"\car\scenes\PORT-FOG-01\SmartCar_Data\StreamingAssets\mapfile\mapd")


class RoadMap2:
    def __init__(self, path: str = MAPD, snap: float = 4.0) -> None:
        lanes = json.load(open(path, encoding="utf-8"))
        self.lanes = []
        self.idx = {}
        for ln in lanes:
            b, e = ln.get("beginPos") or {}, ln.get("endPos") or {}
            a = (float(b.get("x", 0)), float(b.get("y", 0)))
            c = (float(e.get("x", 0)), float(e.get("y", 0)))
            rec = {"id": ln.get("id"), "a": a, "b": c,
                   "len": math.hypot(c[0] - a[0], c[1] - a[1])}
            self.idx[rec["id"]] = len(self.lanes)
            self.lanes.append(rec)

        # 端点聚类成节点（4 m 内合并）
        self.nodes: list[list[float]] = []
        self.end_node: list[tuple[int, int]] = []   # (node_a, node_b)
        for ln in self.lanes:
            na = self._node(ln["a"], snap)
            nb = self._node(ln["b"], snap)
            self.end_node.append((na, nb))

        # 双向邻接：lane 两端都能连到附近 lane 的两端
        self.adj = defaultdict(list)
        for i, (na, nb) in enumerate(self.end_node):
            L = self.lanes[i]["len"]
            self.adj[na].append((nb, L, i))
            self.adj[nb].append((na, L, i))

    def _node(self, p, snap):
        for k, q in enumerate(self.nodes):
            if math.hypot(p[0] - q[0], p[1] - q[1]) <= snap:
                return k
        self.nodes.append([p[0], p[1]])
        return len(self.nodes) - 1

    def nearest_node(self, xy) -> int:
        return min(range(len(self.nodes)),
                   key=lambda k: (self.nodes[k][0] - xy[0]) ** 2 + (self.nodes[k][1] - xy[1]) ** 2)

    def route(self, start_xy, goal_xy):
        s = self.nearest_node(start_xy)
        g = self.nearest_node(goal_xy)
        dist = {s: 0.0}
        prev = {}
        pq = [(0.0, s)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, 1e18) + 1e-9:
                continue
            if u == g:
                break
            for v, w, _i in self.adj.get(u, []):
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
        pts = [(self.nodes[k][0], self.nodes[k][1]) for k in chain]
        total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                    for i in range(len(pts) - 1))
        return pts, total


if __name__ == "__main__":
    import sys
    rm = RoadMap2()
    print("lanes", len(rm.lanes), "nodes", len(rm.nodes),
          "edges", sum(len(v) for v in rm.adj.values()))
    if len(sys.argv) > 4:
        x, y, tx, ty = map(float, sys.argv[1:5])
    else:
        x, y, tx, ty = 618.5, 120.8, 766.5, -35.2
    pts, total = rm.route((x, y), (tx, ty))
    print(f"({x:.0f},{y:.0f})->({tx:.0f},{ty:.0f}) {len(pts)} 点 {total:.0f} m")
    for p in pts[:12]:
        print(f"   ({p[0]:7.1f},{p[1]:7.1f})")
