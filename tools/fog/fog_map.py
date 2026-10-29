# -*- coding: utf-8 -*-
"""港口迷雾：路网层（静态数据导出 + 道路图 + 路径生成）。

职责边界
  * 只读场景：导出 static（道路/静态设施/车辆规格），不在场景里开车。
  * 把 192 条道路建成有向图（道路 id 为节点，successor_ids / 端点距离为边），
    支持 nearest() 与 route()，供四辆车的执行器消费。
  * 不做任何经验常数：车道宽度、限速、车长全部来自场景数据
    （roads[].begin_pos/end_pos、vehicles[].length_m/maximum_speed_mps）。

用法
  # 导出（需要场景在跑）
  python -u fog_map.py --dump D:\\无人算法\\metacar-race\\logs\\fog_map
  # 离线自检（不需要场景）
  python -u fog_map.py --selftest D:\\无人算法\\metacar-race\\logs\\fog_map\\static.json
"""
import argparse
import heapq
import json
import math
import os
import time


# ---------------------------------------------------------------- 基础设施

def connect_retry(timeout_s=180.0, verbose=True):
    from metacar_multiagent import ScenarioTaskAPI
    api = ScenarioTaskAPI()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            api.connect()
            return api
        except Exception as exc:                      # noqa: BLE001
            if verbose:
                print("[map] connect retry: %s" % exc, flush=True)
            time.sleep(2.0)
    raise SystemExit("cannot connect 127.0.0.1:9100")


def to_plain(obj):
    """pydantic / dataclass / 基本类型统一转成可 json 的普通结构。"""
    if hasattr(obj, "model_dump"):
        return obj.model_dump(mode="json")
    if isinstance(obj, dict):
        return {k: to_plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_plain(v) for v in obj]
    if hasattr(obj, "__dict__") and not isinstance(obj, type):
        return {k: to_plain(v) for k, v in vars(obj).items() if not k.startswith("_")}
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return str(obj)


def xy(pos):
    """Vector3（或 dict）-> (x, y)。"""
    if pos is None:
        return None
    if isinstance(pos, dict):
        return (float(pos.get("x", 0.0)), float(pos.get("y", 0.0)))
    return (float(getattr(pos, "x", 0.0)), float(getattr(pos, "y", 0.0)))


def load_static(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def dump_static(api, out_dir):
    st = to_plain(api.get_scene_static_data())
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "static.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(st, fh, ensure_ascii=False, indent=1)
    return path, st


# ---------------------------------------------------------------- 道路图

class RoadGraph(object):
    """有向路网：节点=道路 id，边=successor（缺失时按端点距离 3 m 兜底）。"""

    SNAP_M = 3.0

    def __init__(self, static):
        self.roads = {}
        for r in static.get("roads", []):
            rid = r["id"]
            b = xy(r.get("begin_pos"))
            e = xy(r.get("end_pos"))
            if b is None or e is None:
                continue
            self.roads[rid] = {
                "id": rid,
                "begin": b,
                "end": e,
                "len": math.hypot(e[0] - b[0], e[1] - b[1]),
                "succ": list(r.get("successor_ids") or []),
                "pred": list(r.get("predecessor_ids") or []),
                "type": r.get("driving_type"),
            }
        self.adj = {}
        for rid, road in self.roads.items():
            nxt = [s for s in road["succ"] if s in self.roads]
            if not nxt:
                nxt = self._near_begin(road["end"], exclude=rid)
            self.adj[rid] = nxt

    def _near_begin(self, point, exclude=None, radius=None):
        radius = self.SNAP_M if radius is None else radius
        hits = []
        for rid, road in self.roads.items():
            if rid == exclude:
                continue
            d = math.hypot(road["begin"][0] - point[0], road["begin"][1] - point[1])
            if d <= radius:
                hits.append((d, rid))
        hits.sort()
        return [rid for _d, rid in hits]

    # --- 投影 ---------------------------------------------------------
    def project(self, pos, rid):
        road = self.roads[rid]
        (bx, by), (ex, ey) = road["begin"], road["end"]
        dx, dy = ex - bx, ey - by
        L2 = dx * dx + dy * dy
        if L2 <= 1e-9:
            return 0.0, (bx, by), math.hypot(pos[0] - bx, pos[1] - by)
        t = ((pos[0] - bx) * dx + (pos[1] - by) * dy) / L2
        t = max(0.0, min(1.0, t))
        px, py = bx + t * dx, by + t * dy
        return t, (px, py), math.hypot(pos[0] - px, pos[1] - py)

    def nearest(self, pos, radius=25.0):
        """返回 (road_id, t, 投影点, 距离)。找不到返回 None。"""
        best = None
        for rid in self.roads:
            t, p, d = self.project(pos, rid)
            if d <= radius and (best is None or d < best[3]):
                best = (rid, t, p, d)
        return best

    def nearest_many(self, pos, k=3, radius=30.0):
        hits = []
        for rid in self.roads:
            t, p, d = self.project(pos, rid)
            if d <= radius:
                hits.append((d, rid, t, p))
        hits.sort()
        return hits[:k]

    # --- 路由 ---------------------------------------------------------
    def route_roads(self, start_rid, goal_rid):
        """A*：以道路长度 + 转弯惩罚为代价。返回道路 id 列表。"""
        if start_rid == goal_rid:
            return [start_rid]

        def h(rid):
            e = self.roads[rid]["end"]
            g = self.roads[goal_rid]["begin"]
            return math.hypot(e[0] - g[0], e[1] - g[1])

        def heading(rid):
            road = self.roads[rid]
            return math.atan2(road["end"][1] - road["begin"][1],
                              road["end"][0] - road["begin"][0])

        openq = [(h(start_rid), 0.0, start_rid)]
        best = {start_rid: 0.0}
        prev = {}
        while openq:
            _f, g, rid = heapq.heappop(openq)
            if rid == goal_rid:
                path = [rid]
                while path[-1] in prev:
                    path.append(prev[path[-1]])
                return list(reversed(path))
            if g > best.get(rid, 1e18) + 1e-9:
                continue
            hcur = heading(rid)
            for nxt in self.adj.get(rid, []):
                turn = abs((heading(nxt) - hcur + math.pi) % (2 * math.pi) - math.pi)
                step = self.roads[rid]["len"] + 6.0 * (turn / math.pi)
                ng = g + step
                if ng < best.get(nxt, 1e18) - 1e-9:
                    best[nxt] = ng
                    prev[nxt] = rid
                    heapq.heappush(openq, (ng + h(nxt), ng, nxt))
        return None

    def route(self, start_xy, goal_xy, radius=30.0):
        """给两点返回折线航点列表 [(x, y), ...]（含起终点投影）。"""
        s = self.nearest(start_xy, radius=radius)
        g = self.nearest(goal_xy, radius=radius)
        if s is None or g is None:
            return None
        s_rid, s_t, s_p, _sd = s
        g_rid, g_t, g_p, _gd = g
        roads = self.route_roads(s_rid, g_rid)
        if roads is None:
            return None
        pts = [start_xy, s_p]
        for rid in roads:
            road = self.roads[rid]
            pts.append(road["begin"])
            pts.append(road["end"])
        pts.append(g_p)
        pts.append(goal_xy)
        clean = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > 1.0:
                clean.append(p)
        return clean


# ---------------------------------------------------------------- 统计

def summarize(static):
    from collections import Counter
    cats = Counter(o.get("category") for o in static.get("static_objects", []))
    print("[map] roads=%d vehicles=%d static_objects=%d"
          % (len(static.get("roads", [])), len(static.get("vehicles", [])),
             len(static.get("static_objects", []))))
    for cat, n in cats.most_common(12):
        print("    %-16s %d" % (cat, n))
    for v in static.get("vehicles", []):
        print("[map] veh %-6s role=%-6s L=%.2f W=%.2f vmax=%.2f"
              % (v.get("vehicle_id"), v.get("role"), v.get("length_m", 0.0),
                 v.get("width_m", 0.0), v.get("maximum_speed_mps", 0.0)))
    graph = RoadGraph(static)
    dead = [rid for rid, nxt in graph.adj.items() if not nxt]
    print("[map] graph nodes=%d edges=%d dead_ends=%d"
          % (len(graph.roads), sum(len(v) for v in graph.adj.values()), len(dead)))
    if dead:
        print("[map] dead end sample:", dead[:5])
    return graph


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump")
    ap.add_argument("--selftest")
    ap.add_argument("--route", nargs=4, type=float,
                    metavar=("X0", "Y0", "X1", "Y1"))
    args = ap.parse_args()

    if args.dump:
        api = connect_retry()
        try:
            path, static = dump_static(api, args.dump)
            print("[map] dumped -> %s" % path, flush=True)
        finally:
            try:
                api.close()
            except Exception:                          # noqa: BLE001
                pass
        summarize(static)
        return

    if args.selftest:
        static = load_static(args.selftest)
        graph = summarize(static)
        if args.route:
            x0, y0, x1, y1 = args.route
            pts = graph.route((x0, y0), (x1, y1))
            if pts is None:
                print("[map] route: no path")
            else:
                print("[map] route waypoints=%d total=%.1f m"
                      % (len(pts), sum(math.hypot(pts[i + 1][0] - pts[i][0],
                                                  pts[i + 1][1] - pts[i][1])
                                       for i in range(len(pts) - 1))))
                for p in pts[:8]:
                    print("    (%.1f, %.1f)" % p)
        return

    ap.error("need --dump or --selftest")


if __name__ == "__main__":
    main()
