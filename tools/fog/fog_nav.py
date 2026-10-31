# -*- coding: utf-8 -*-
"""港口迷雾导航：用静态障碍物做栅格代价图 + A*（不走那份断裂的路网）。

static_objects 有 225 个构件（bollard 最多，还有 warehouse / storage_tank /
quay_crane / construction_barrier / task_terminal / solid_obstacle），位置尺寸
都公开 ⇒ 开车前就能把"哪儿不能去"标出来：

  * 每个构件按 length×width 的外接圆膨胀 INFLATE_M（车半宽 + 余量）；
  * 地图边界外 3 m 设墙（越过边界 1 s 直接归零）；
  * A* 八邻域 + 对角不切角，代价带轻微"离障碍越近代价越高"的软惩罚；
  * 出身后再做视线剪枝（line-of-sight smoothing），把点列压成少数航点。

用法：
  python fog_nav.py --static ...\\static.json --route 618.5,120.8 720.5,-96
"""
import argparse
import heapq
import json
import math

RES = 1.0
INFLATE_M = 3.2          # 车半宽 1.09 + 余量
BORDER_M = 3.0
SOFT_M = 6.0             # 软惩罚宽度


class Grid:
    def __init__(self, static, res=RES, inflate=INFLATE_M):
        so = static.get("static_objects") or []
        xs = [o["position"]["x"] for o in so] or [0.0]
        ys = [o["position"]["y"] for o in so] or [0.0]
        self.x0 = min(xs) - BORDER_M - 5.0
        self.y0 = min(ys) - BORDER_M - 5.0
        self.x1 = max(xs) + BORDER_M + 5.0
        self.y1 = max(ys) + BORDER_M + 5.0
        self.res = res
        self.w = int((self.x1 - self.x0) / res) + 1
        self.h = int((self.y1 - self.y0) / res) + 1
        self.block = bytearray(self.w * self.h)     # 1 = 不可通行
        self.soft = [0.0] * (self.w * self.h)       # 软惩罚（0~1）

        for o in so:
            px, py = float(o["position"]["x"]), float(o["position"]["y"])
            L = float(o.get("length_m") or 0.0)
            W = float(o.get("width_m") or 0.0)
            ori = o.get("orientation_degrees") or {}
            yaw = math.radians(float(ori.get("z", 0.0) or 0.0))
            self._rect(px, py, L, W, yaw, inflate, block=True)
            self._rect(px, py, L, W, yaw, inflate + SOFT_M, soft=True)

    def _rect(self, cx, cy, L, W, yaw, margin, block=False, soft=False):
        """把一个（可旋转）矩形膨胀 margin 后标进栅格。

        长条构件（防波堤 QuayEdge、隔离墩）以前按"外接圆"膨胀，会把
        300 m 的长边变成一个半径 150 m 的大黑圆，整张图 68% 不可通行。
        """
        half_diag = 0.5 * math.hypot(L, W) + margin
        ix0, iy0 = self.to_cell(cx - half_diag, cy - half_diag)
        ix1, iy1 = self.to_cell(cx + half_diag, cy + half_diag)
        ca, sa = math.cos(-yaw), math.sin(-yaw)
        hl, hw = 0.5 * L, 0.5 * W
        for iy in range(max(0, iy0), min(self.h - 1, iy1) + 1):
            for ix in range(max(0, ix0), min(self.w - 1, ix1) + 1):
                px, py = self.to_xy(ix, iy)
                dx, dy = px - cx, py - cy
                u = ca * dx - sa * dy
                v = sa * dx + ca * dy
                du = max(abs(u) - hl, 0.0)
                dv = max(abs(v) - hw, 0.0)
                if math.hypot(du, dv) <= margin:
                    if block:
                        self.block[self.idx(ix, iy)] = 1
                    if soft:
                        self.soft[self.idx(ix, iy)] = 1.0

    # ---------- 栅格工具 ----------
    def idx(self, ix, iy):
        return iy * self.w + ix

    def to_cell(self, x, y):
        return int((x - self.x0) / self.res), int((y - self.y0) / self.res)

    def to_xy(self, ix, iy):
        return self.x0 + (ix + 0.5) * self.res, self.y0 + (iy + 0.5) * self.res

    def _disc(self, cx, cy, r, block=False, soft=False):
        ix0, iy0 = self.to_cell(cx - r, cy - r)
        ix1, iy1 = self.to_cell(cx + r, cy + r)
        for iy in range(max(0, iy0), min(self.h - 1, iy1) + 1):
            for ix in range(max(0, ix0), min(self.w - 1, ix1) + 1):
                px, py = self.to_xy(ix, iy)
                d = math.hypot(px - cx, py - cy)
                if d <= r:
                    if block:
                        self.block[self.idx(ix, iy)] = 1
                    if soft:
                        self.soft[self.idx(ix, iy)] = 1.0

    def passable(self, ix, iy):
        if ix < 1 or iy < 1 or ix >= self.w - 1 or iy >= self.h - 1:
            return False
        return not self.block[self.idx(ix, iy)]

    def cost(self, ix, iy):
        return 1.0 + 1.5 * self.soft[self.idx(ix, iy)]

    # ---------- A* ----------
    def astar(self, start_xy, goal_xy):
        sx, sy = self.to_cell(*start_xy)
        gx, gy = self.to_cell(*goal_xy)
        sx, sy = self._nearest_free(sx, sy)
        gx, gy = self._nearest_free(gx, gy)
        # `_nearest_free` 找不到空格子时返回 (None, None)；原来写 `(sx, sy) is None` 永远
        # 不成立（tuple 不是 None），会把 None 带进 A* 直接 TypeError 崩掉整局。逐元素判空。
        if sx is None or sy is None or gx is None or gy is None:
            return None
        start = (sx, sy)
        goal = (gx, gy)
        nb8 = ((1, 0), (-1, 0), (0, 1), (0, -1),
               (1, 1), (1, -1), (-1, 1), (-1, -1))
        openq = [(0.0, start)]
        came = {}
        gcost = {start: 0.0}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == goal:
                break
            cx, cy = cur
            for dx, dy in nb8:
                nx, ny = cx + dx, cy + dy
                if not self.passable(nx, ny):
                    continue
                if dx and dy and (not self.passable(cx + dx, cy)
                                  or not self.passable(cx, cy + dy)):
                    continue                       # 不切角
                step = math.hypot(dx, dy) * self.cost(nx, ny)
                ng = gcost[cur] + step
                node = (nx, ny)
                if node in gcost and ng >= gcost[node] - 1e-9:
                    continue
                gcost[node] = ng
                came[node] = cur
                hh = math.hypot(goal[0] - nx, goal[1] - ny)
                heapq.heappush(openq, (ng + hh, node))
        if goal not in gcost:
            return None
        chain = [goal]
        while chain[-1] != start:
            chain.append(came[chain[-1]])
        chain.reverse()
        pts = [start_xy] + [self.to_xy(ix, iy) for ix, iy in chain] + [goal_xy]
        return smooth(pts, self)

    def _nearest_free(self, ix, iy, r=12):
        if self.passable(ix, iy):
            return ix, iy
        for rr in range(1, r + 1):
            best = None
            for dy in range(-rr, rr + 1):
                for dx in range(-rr, rr + 1):
                    if max(abs(dx), abs(dy)) != rr:
                        continue
                    nx, ny = ix + dx, iy + dy
                    if self.passable(nx, ny):
                        d = math.hypot(dx, dy)
                        if best is None or d < best[0]:
                            best = (d, nx, ny)
            if best:
                return best[1], best[2]
        return None, None

    def line_free(self, a, b, step=0.6):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(d / step))
        for i in range(n + 1):
            x = a[0] + (b[0] - a[0]) * i / n
            y = a[1] + (b[1] - a[1]) * i / n
            ix, iy = self.to_cell(x, y)
            if not self.passable(ix, iy):
                return False
        return True


def smooth(pts, grid, look=20):
    """视线剪枝：能直连就跳过中间点。"""
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    i = 0
    n = len(pts)
    while i < n - 1:
        j = min(n - 1, i + look)
        while j > i + 1 and not grid.line_free(pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def path_length(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def load(static_path):
    with open(static_path, encoding="utf-8") as fh:
        return Grid(json.load(fh))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--static", required=True)
    ap.add_argument("--route", nargs=4, type=float,
                    metavar=("SX", "SY", "GX", "GY"))
    args = ap.parse_args()
    g = load(args.static)
    print("grid %dx%d res=%.1f block=%.1f%%"
          % (g.w, g.h, g.res, 100.0 * sum(g.block) / len(g.block)))
    if args.route:
        sx, sy, gx, gy = args.route
        p = g.astar((sx, sy), (gx, gy))
        if p is None:
            print("NO ROUTE")
        else:
            print("route waypoints=%d length=%.1f m straight=%.1f"
                  % (len(p), path_length(p), math.hypot(gx - sx, gy - sy)))
            for q in p:
                print("   (%.1f, %.1f)" % q)
