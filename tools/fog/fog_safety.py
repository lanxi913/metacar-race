# -*- coding: utf-8 -*-
"""港口迷雾：防撞层（"别撞墙"）。

为什么必须有这一层：这个场景跟建筑/集装箱**碰一下就整车锁死**——`[collision]`
事件刷屏之后油门 0.62、倒挡都没反应，一局直接作废（10-29 实测 Car_C 撞
`CustomsInspectionWarehouse` 后在原地卡了 9 分钟）。所以撞墙不是扣分，是结束。

两层保护：

1. `Safety.path_clearance()` —— 开跑前把整条路线量一遍，离实体障碍 < 1.6 m 的
   路线直接判定"不建议走"，由调用方改用栅格 A* 绕行。
2. `Safety.guard()` —— 每帧按"再走 0.6 s 会在哪"预测，按剩余净空分级处置：
       净空 > 2.0 m   正常
       1.0~2.0 m      限速 1.2 m/s，并朝远离障碍的一侧加转向偏置
       < 1.0 m        直接刹停
   净空按"车体半宽 1.23 m"扣过，即真正贴到墙面的余量。

只用 static_objects（`fog_dump_static.py` 存下来的那份也能离线复算），不依赖场景。
"""
from __future__ import annotations

import math

CAR_HALF_W = 1.23                 # Car_C 车宽 2.45 m 的一半
HALF_LEN = 2.40                   # 车长 4.81 m 的一半

SOLID_CATS = {"warehouse", "storage_tank", "quay_crane", "construction_barrier",
              "building", "solid_obstacle", "task_terminal", "container"}

# 判定门限（扣掉车体半宽之后的自由净空，米）
#
# ⚠️ 不能按"空旷场地"来定：中央路口是一条 5.06 m 宽的闸口，车宽 2.45 m，
# 车摆在正中间时两侧净空只有 **1.10 m**（实测 Port_PassageBarrier 到中心线的距离）。
# 门限高于它，车在闸口里会被一直限油门/推来推去（10-29 22:53 实测卡在 (728.6,-26.5) 磨了十几秒）。
# 所以门限全部压在 1.10 以下：
#   净空 > 0.85 m   正常
#   0.35~0.85 m     限油门（不打方向）
#   0.60~0.85 m     额外朝远离一侧轻打方向（0.20）
#   < 0.35 m        硬刹
SAFE_GAP = 0.85
SLOW_GAP = 0.35
STEER_GAP = 0.60
WARN_PATH_M = 1.60                # 路线体检门限（含车体半宽）


def _rect(px, py, o):
    p = o["position"]
    cx, cy = float(p["x"]), float(p["y"])
    L = float(o.get("length_m") or 0.0)
    W = float(o.get("width_m") or 0.0)
    ori = o.get("orientation_degrees") or {}
    yaw = math.radians(float(ori.get("z", 0.0) or 0.0))
    return cx, cy, L, W, yaw


def rect_dist(px, py, cx, cy, L, W, yaw):
    """点到（可旋转）矩形外表面的距离；点在矩形内返回 0。"""
    dx, dy = px - cx, py - cy
    ca, sa = math.cos(-yaw), math.sin(-yaw)
    u = ca * dx - sa * dy
    v = sa * dx + ca * dy
    du = max(abs(u) - 0.5 * L, 0.0)
    dv = max(abs(v) - 0.5 * W, 0.0)
    return math.hypot(du, dv)


class Safety(object):
    def __init__(self, static, half_w=CAR_HALF_W):
        self.half_w = float(half_w)
        self.objs = []
        for o in static.get("static_objects") or []:
            if o.get("category") not in SOLID_CATS:
                continue
            cx, cy, L, W, yaw = _rect(None, None, o)
            self.objs.append((cx, cy, L, W, yaw, str(o.get("id") or "")))

    # ---------------------------------------------------------- 距离查询
    def nearest(self, x, y):
        """返回 (到障碍外表面的距离, 障碍 id, 障碍中心与尺寸) —— 未扣车体半宽。"""
        best = (1e9, "", None)
        for cx, cy, L, W, yaw, oid in self.objs:
            d = rect_dist(x, y, cx, cy, L, W, yaw)
            if d < best[0] - 1e-9:
                best = (d, oid, (cx, cy, L, W, yaw))
        return best

    def gap(self, x, y):
        """自由净空（米）：到最近实体障碍的距离 − 车体半宽。"""
        d, oid, info = self.nearest(x, y)
        return d - self.half_w, oid, info

    # ---------------------------------------------------------- 路线体检
    def path_clearance(self, pts, skip_tail=0.0, skip_head=0.0, step=0.5):
        """沿折线采样，返回 (最小净空, 位置, 障碍 id)。

        skip_head / skip_tail 用来跳过首尾几米：目标是信号牌、起点是刚验证完的信号牌时，
        那几米必然贴着实体（信号牌就长在 terminal 上），算进路线风险会误判。
        """
        total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                    for i in range(len(pts) - 1))
        lo = max(0.0, skip_head)
        hi = max(0.0, total - max(0.0, skip_tail))
        acc = 0.0
        best = (1e9, None, "")
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            if seg < 1e-9:
                continue
            n = max(1, int(seg / step))
            for k in range(n + 1):
                s = acc + seg * k / n
                if s < lo or s > hi:
                    continue
                x = ax + (bx - ax) * k / n
                y = ay + (by - ay) * k / n
                d, oid, _info = self.nearest(x, y)
                if d < best[0]:
                    best = (d, (x, y), oid)
            acc += seg
        return best

    # ---------------------------------------------------------- 运行时护栏
    def guard(self, x, y, yaw, speed, thr, brk, steer, horizon=0.6, allow_bias=True):
        """按 0.6 s 后的预测位置判断，返回可能被修正过的 (thr, brk, steer, info)。

        allow_bias=False 时只限油门/刹车，不往旁边推 —— 5 m 宽的闸口里两侧对称，
        推哪边都是往墙上推（23:23 实测 Car_B 在闸口内被推得横移）。
        """
        ph = math.radians(yaw)
        px = x + math.cos(ph) * max(0.0, speed) * horizon
        py = y + math.sin(ph) * max(0.0, speed) * horizon
        d_now, oid_now, info_now = self.nearest(x, y)
        d_ahead, oid_ahead, info_ahead = self.nearest(px, py)
        d = min(d_now, d_ahead)
        info = info_ahead if d_ahead <= d_now else info_now
        oid = oid_ahead if d_ahead <= d_now else oid_now
        gap = d - self.half_w
        if gap > SAFE_GAP or info is None:
            return thr, brk, steer, (gap, oid)

        # 朝远离障碍的一侧打方向：把"障碍中心→车"的向量投到车身横向
        cx, cy = info[0], info[1]
        rx, ry = x - cx, y - cy
        lateral = -math.sin(ph) * rx + math.cos(ph) * ry       # >0 障碍在左
        if gap <= SLOW_GAP:
            return 0.0, max(brk, 0.7), steer, (gap, oid)
        new_steer = steer
        if allow_bias and gap <= STEER_GAP and abs(lateral) > 0.05:
            bias = 0.20 if lateral > 0 else -0.20
            new_steer = max(-1.0, min(1.0, steer + bias))
        return min(thr, 0.10), max(brk, 0.05), new_steer, (gap, oid)
