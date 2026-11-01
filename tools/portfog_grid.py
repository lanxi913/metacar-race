# -*- coding: utf-8 -*-
"""栅格 A* 路由：把 mapd 车道栅格化成可行驶区域（距车道 <=6 m），在栅格上搜路径。

不依赖车道拓扑连通性，任何两点只要都在路网附近就能规划。
"""
from __future__ import annotations

import heapq
import json
import math

MAPD = (r"C:\Users\lanxi\AppData\Roaming\com.i-mylab.metapower-desktop"
        r"\car\scenes\PORT-FOG-01\SmartCar_Data\StreamingAssets\mapfile\mapd")

CELL = 2.0
DRIVE_HALF_WIDTH = 10.0


class GridRoad:
    def __init__(self, path: str = MAPD) -> None:
        lanes = json.load(open(path, encoding="utf-8"))
        self.segs = []
        for ln in lanes:
            b, e = ln.get("beginPos") or {}, ln.get("endPos") or {}
            self.segs.append(((float(b.get("x", 0)), float(b.get("y", 0))),
                              (float(e.get("x", 0)), float(e.get("y", 0)))))
        xs = [p[0] for s in self.segs for p in s]
        ys = [p[1] for s in self.segs for p in s]
        self.x0, self.x1 = min(xs) - 12, max(xs) + 12
        self.y0, self.y1 = min(ys) - 12, max(ys) + 12
        self.nx = int((self.x1 - self.x0) / CELL) + 1
        self.ny = int((self.y1 - self.y0) / CELL) + 1
        self.free = self._rasterize()

    # --- 栅格 ---
    def _cell_of(self, x, y):
        return (int((x - self.x0) / CELL), int((y - self.y0) / CELL))

    def _xy_of(self, ix, iy):
        return (self.x0 + (ix + 0.5) * CELL, self.y0 + (iy + 0.5) * CELL)

    def _rasterize(self):
        free = bytearray(self.nx * self.ny)
        for (ax, ay), (bx, by) in self.segs:
            i0, j0 = self._cell_of(ax, ay)
            i1, j1 = self._cell_of(bx, by)
            steps = max(abs(i1 - i0), abs(j1 - j0)) + 1
            for s in range(steps + 1):
                t = s / steps if steps else 0
                cx = ax + (bx - ax) * t
                cy = ay + (by - ay) * t
                ci, cj = self._cell_of(cx, cy)
                r = int(DRIVE_HALF_WIDTH / CELL)
                for di in range(-r, r + 1):
                    for dj in range(-r, r + 1):
                        i, j = ci + di, cj + dj
                        if 0 <= i < self.nx and 0 <= j < self.ny:
                            free[j * self.nx + i] = 1
        return free

    def is_free(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny and self.free[j * self.nx + i] == 1

    def nearest_free(self, x, y):
        ci, cj = self._cell_of(x, y)
        if self.is_free(ci, cj):
            return ci, cj
        for r in range(1, 40):
            for di in range(-r, r + 1):
                for dj in (-r, r):
                    if self.is_free(ci + di, cj + dj):
                        return ci + di, cj + dj
            for dj in range(-r, r + 1):
                for di in (-r, r):
                    if self.is_free(ci + di, cj + dj):
                        return ci + di, cj + dj
        return ci, cj

    def astar(self, start, goal):
        s = self.nearest_free(*start)
        g = self.nearest_free(*goal)
        if s == g:
            return [self._xy_of(*s)]
        openq = [(0.0, s)]
        gscore = {s: 0.0}
        came = {}
        while openq:
            f, cur = heapq.heappop(openq)
            if cur == g:
                break
            ci, cj = cur
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    ni, nj = ci + di, cj + dj
                    if not self.is_free(ni, nj):
                        continue
                    step = math.hypot(di, dj) * CELL
                    ng = gscore[cur] + step
                    if ng < gscore.get((ni, nj), 1e18) - 1e-9:
                        gscore[(ni, nj)] = ng
                        came[(ni, nj)] = cur
                        h = math.hypot(ni - g[0], nj - g[1]) * CELL
                        heapq.heappush(openq, (ng + h, (ni, nj)))
        if g not in gscore:
            return []
        path = [g]
        while path[-1] != s and path[-1] in came:
            path.append(came[path[-1]])
        path.reverse()
        return [self._xy_of(i, j) for i, j in path]

    def simplify(self, path, tol=8.0):
        """道格拉斯-普克简化，减少航点。"""
        if len(path) < 3:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = min(i + 12, len(path) - 1)
            out.append(path[j])
            i = j
        if out[-1] != path[-1]:
            out.append(path[-1])
        return out


if __name__ == "__main__":
    import sys
    g = GridRoad()
    print(f"栅格 {g.nx}x{g.ny} @ {CELL} m, 可行驶格 "
          f"{sum(g.free)}/{len(g.free)} ({100*sum(g.free)/len(g.free):.1f}%)")
    x, y, tx, ty = (map(float, sys.argv[1:5]) if len(sys.argv) > 4
                    else (618.5, 120.8, 766.5, -35.2))
    p = g.astar((x, y), (tx, ty))
    L = sum(math.hypot(p[i+1][0]-p[i][0], p[i+1][1]-p[i][1]) for i in range(len(p)-1))
    print(f"({x:.0f},{y:.0f})->({tx:.0f},{ty:.0f}) 路径 {len(p)} 点 / {L:.0f} m")
    for q in g.simplify(p)[:12]:
        print(f"   ({q[0]:7.1f},{q[1]:7.1f})")
