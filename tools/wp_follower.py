# -*- coding: utf-8 -*-
"""航点跟随器（纯追踪 + 到点切换），从 tools/probe_zone_lap.py 抽出来独立成模块，
供 tools/vla_zone_read.py 以及 tools/probes/ 下的探针共用。"""
from __future__ import annotations

import math

WHEELBASE_M = 2.6
MAX_STEER_RAD = 0.6


class WpFollower:
    """逐航点纯跟踪：目标 = 下一个航点（≥ lookahead 时取车前方 lookahead 处）。"""

    def __init__(self, wps, lookahead=6.0, accept=4.0):
        self.wps = [tuple(map(float, w)) for w in wps]
        self.lookahead = lookahead
        self.accept = accept
        self.seg = 0

    def done(self):
        return self.seg >= len(self.wps) - 1

    def remain_m(self, x, y):
        if self.done():
            return 0.0
        total = math.hypot(self.wps[self.seg][0] - x, self.wps[self.seg][1] - y)
        for i in range(self.seg, len(self.wps) - 1):
            (ax, ay), (bx, by) = self.wps[i], self.wps[i + 1]
            total += math.hypot(bx - ax, by - ay)
        return total

    def update(self, x, y):
        while not self.done():
            wx, wy = self.wps[self.seg]
            if math.hypot(wx - x, wy - y) < self.accept:
                self.seg += 1
            else:
                break

    def target(self, x, y):
        wx, wy = self.wps[self.seg]
        dx, dy = wx - x, wy - y
        d = math.hypot(dx, dy)
        if d <= self.lookahead or d < 1e-6:
            return wx, wy
        return x + dx / d * self.lookahead, y + dy / d * self.lookahead

    def steering(self, x, y, yaw_deg):
        if self.done():
            return 0.0
        tx, ty = self.target(x, y)
        dx, dy = tx - x, ty - y
        dist = max(math.hypot(dx, dy), 1e-3)
        rad = math.radians(-yaw_deg)
        ly = -dx * math.sin(rad) + dy * math.cos(rad)
        delta = math.atan((2.0 * ly / (dist * dist)) * WHEELBASE_M)
        return max(-1.0, min(1.0, -delta / MAX_STEER_RAD))


