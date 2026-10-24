# -*- coding: utf-8 -*-
"""Hybrid A* 泊车路径规划（运行时现场规划，不写死坐标）。

输入全部来自场景：
  * 车位：障碍物列表里 type == PARKING_SLOT 的那个
  * 已停车辆：type == CAR 的障碍物
  * 边线：车位外侧边（车位中心 - 半宽）

输出：一串 (x, y, theta, direction) 位姿，theta 为数学角，direction 为 +1 前进 / -1 倒车。
"""
from __future__ import annotations

import heapq
import math


class Pose:
    __slots__ = ("x", "y", "th", "d")

    def __init__(self, x, y, th, d=1.0):
        self.x, self.y, self.th, self.d = x, y, th, d


def rect(x, y, th, length, width, pad=0.06):
    ca, sa = math.cos(th), math.sin(th)
    hl, hw = length / 2 + pad, width / 2 + pad
    return [
        (x + dx * ca - dy * sa, y + dx * sa + dy * ca)
        for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))
    ]


def sat_overlap(p1, p2):
    def axes(pts):
        out = []
        for i in range(len(pts)):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % len(pts)]
            ex, ey = x2 - x1, y2 - y1
            n = math.hypot(ex, ey)
            if n > 1e-9:
                out.append((-ey / n, ex / n))
        return out

    for ax in axes(p1) + axes(p2):
        a = [ax[0] * p[0] + ax[1] * p[1] for p in p1]
        b = [ax[0] * p[0] + ax[1] * p[1] for p in p2]
        if max(a) < min(b) or max(b) < min(a):
            return False
    return True


class Planner:
    def __init__(self, ego_len, ego_w, wheelbase, max_steer,
                 obstacles, side_line_x, ds=0.4, xy_res=0.15, th_res=math.pi / 36):
        self.ego_len = ego_len
        self.ego_w = ego_w
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.obstacles = obstacles          # list[list[(x,y)]]
        self.side_line_x = side_line_x
        self.ds = ds
        self.xy_res = xy_res
        self.th_res = th_res
        self.deltas = [-max_steer, -0.35, -0.15, 0.0, 0.15, 0.35, max_steer]

    # ---------- 基础 ----------
    def free(self, x, y, th):
        ego = rect(x, y, th, self.ego_len, self.ego_w)
        if min(p[0] for p in ego) < self.side_line_x + 0.02:
            return False
        return not any(sat_overlap(ego, o) for o in self.obstacles)

    def _key(self, x, y, th):
        return (round(x / self.xy_res), round(y / self.xy_res), round(th / self.th_res))

    def _step(self, x, y, th, delta, d):
        k = math.tan(delta) / self.wheelbase
        th2 = th + d * self.ds * k
        thm = 0.5 * (th + th2)
        return x + d * self.ds * math.cos(thm), y + d * self.ds * math.sin(thm), th2

    # ---------- 搜索 ----------
    def plan(self, start, goal, goal_tol=0.22, yaw_tol=math.radians(6.0),
             max_nodes=400000, heur_w=1.25):
        """start/goal: (x, y, theta)。返回 Pose 列表或 None。"""
        gx, gy, gth = goal
        nodes = [{"s": start, "p": -1, "d": 0.0}]
        heur = lambda x, y: math.hypot(x - gx, y - gy)
        heap = [(heur_w * heur(start[0], start[1]), 0.0, 0)]
        best = {self._key(*start): 0.0}
        cnt = 0

        while heap and cnt < max_nodes:
            _, g, idx = heapq.heappop(heap)
            cnt += 1
            x, y, th = nodes[idx]["s"]
            if best.get(self._key(x, y, th), 1e9) < g - 1e-9:
                continue
            if heur(x, y) < goal_tol and abs((th - gth + math.pi) % (2 * math.pi) - math.pi) < yaw_tol:
                path = []
                i = idx
                while i != -1:
                    s = nodes[i]["s"]
                    path.append(Pose(s[0], s[1], s[2], nodes[i]["d"]))
                    i = nodes[i]["p"]
                path.reverse()
                return path

            prev_d = nodes[idx]["d"]
            for d in (1.0, -1.0):
                for delta in self.deltas:
                    x2, y2, th2 = self._step(x, y, th, delta, d)
                    if not self.free(x2, y2, th2):
                        continue
                    cost = self.ds + 0.12 * abs(delta)
                    if prev_d and d != prev_d:
                        cost += 0.45
                    g2 = g + cost
                    k2 = self._key(x2, y2, th2)
                    if k2 in best and best[k2] <= g2 + 1e-9:
                        continue
                    best[k2] = g2
                    nodes.append({"s": (x2, y2, th2), "p": idx, "d": d})
                    heapq.heappush(heap, (g2 + heur_w * heur(x2, y2), g2, len(nodes) - 1))
        return None


def slot_goal(slot):
    """PARKING_SLOT 障碍物 -> 目标位姿（数学角）。"""
    th = math.radians(-slot.ori_z)   # 场景 yaw 顺时针为正
    return (slot.pos_x, slot.pos_y, th)


def side_line_from_slot(slot):
    """车位外侧边线（车位中心 - 半宽），车不能越过它。"""
    return slot.pos_x - slot.width / 2.0
