# -*- coding: utf-8 -*-
"""港口迷雾：把"车道后继图"翻译成可以直接跟的航点。

fog_map.RoadGraph.route 会把每条道路的 begin/end 都塞进点列，导致
"先往回开 13.7 m 再掉头"这种病态航点（实测车在 (559,109) 掉了路基）。
这里改成：起点投影到当前车道 → 只沿链上每条车道的 **终点** 排点：

    [车当前位置] + [e1, e2, ..., en] + [目标点]

到路口的大转弯（相邻车道端点相差 15~20 m）交给纯跟踪自己转，
并给出每条边的"期望速度"（按转角降速）。
"""
import heapq
import math

SNAP_M = 35.0
TURN_PENALTY = 8.0


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Route(object):
    def __init__(self, pts, speeds):
        self.pts = pts
        self.speeds = speeds

    def __len__(self):
        return len(self.pts)

    def length(self):
        return sum(math.hypot(self.pts[i + 1][0] - self.pts[i][0],
                              self.pts[i + 1][1] - self.pts[i][1])
                   for i in range(len(self.pts) - 1))


def _heading(road):
    return math.atan2(road["end"][1] - road["begin"][1],
                      road["end"][0] - road["begin"][0])


def route(graph, start_xy, goal_xy, v_max=4.5):
    """返回 Route（点列 + 每个点的期望速度）或 None。"""
    # 起点：最近车道（投影）
    best = None
    for rid, road in graph.roads.items():
        t, p, d = graph.project(start_xy, rid)
        if d < SNAP_M and (best is None or d < best[0]):
            best = (d, rid, t)
    if best is None:
        return None
    _d, rid0, _t = best

    # 终点：最近车道
    best_g = None
    for rid, road in graph.roads.items():
        t, p, d = graph.project(goal_xy, rid)
        if d < SNAP_M and (best_g is None or d < best_g[0]):
            best_g = (d, rid)
    if best_g is None:
        return None
    rid_goal = best_g[1]

    # A*（有向后继 + 转角惩罚）
    def h(rid):
        e = graph.roads[rid]["end"]
        return math.hypot(e[0] - goal_xy[0], e[1] - goal_xy[1])

    openq = [(0.0, rid0)]
    came = {}
    gcost = {rid0: 0.0}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == rid_goal:
            break
        crow = graph.roads[cur]
        ch = _heading(crow)
        for nb in graph.adj.get(cur, ()):
            road = graph.roads[nb]
            turn = abs(wrap_pi(_heading(road) - ch))
            step = road["len"] + TURN_PENALTY * (turn / math.pi)
            ng = gcost[cur] + step
            if nb in gcost and ng >= gcost[nb] - 1e-9:
                continue
            gcost[nb] = ng
            came[nb] = cur
            heapq.heappush(openq, (ng + h(nb), nb))
    if rid_goal not in gcost:
        return None
    chain = [rid_goal]
    while chain[-1] != rid0:
        chain.append(came[chain[-1]])
    chain.reverse()

    pts = [start_xy]
    speeds = []
    prev_h = None
    for rid in chain:
        road = graph.roads[rid]
        (bx, by), (ex, ey) = road["begin"], road["end"]
        seg = math.hypot(ex - bx, ey - by)
        # 沿车道每 2 m 一个点：光给路口端点会让车"切弯"切到路肩上
        n = max(1, int(seg / 2.0))
        for i in range(1, n + 1):
            pts.append((bx + (ex - bx) * i / n, by + (ey - by) * i / n))
            hh = _heading(road)
            if prev_h is None:
                turn = 0.0
            else:
                turn = abs(wrap_pi(hh - prev_h))
            if turn > 1.2:
                v = min(v_max, 1.6)
            elif turn > 0.6:
                v = min(v_max, 2.2)
            elif turn > 0.25:
                v = min(v_max, 3.2)
            else:
                v = v_max
            speeds.append(v)
        prev_h = _heading(road)
    pts.append(goal_xy)
    speeds.append(min(v_max, 2.2))
    # 去重
    out_p, out_v = [pts[0]], []
    for i in range(1, len(pts)):
        if math.hypot(pts[i][0] - out_p[-1][0], pts[i][1] - out_p[-1][1]) > 1.0:
            out_p.append(pts[i])
            out_v.append(speeds[i - 1])
    if len(out_p) < 2:
        return None
    return Route(out_p, out_v)
