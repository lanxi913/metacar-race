# -*- coding: utf-8 -*-
"""从 static.json 的路网找“闸口北出口 → 泊位B口”的合法车道链，并打印折线。"""

import json
import math
import sys

PATH = r"D:\无人算法\metacar-race\logs\fog_map\static.json"


def load():
    with open(PATH, encoding="utf-8") as fh:
        st = json.load(fh)
    roads = []
    for rd in st["roads"]:
        b = rd.get("begin_pos") or {}
        e = rd.get("end_pos") or {}
        if "x" not in b or "x" not in e:
            continue
        pts = []
        for ln in rd.get("lanes", []):
            pp = ln.get("path_points") or []
            if pp and isinstance(pp[0], dict):
                pts = [(p["x"], p["y"]) for p in pp]
                break
            lb = (ln.get("left_border") or {}).get("path_points") or []
            rb = (ln.get("right_border") or {}).get("path_points") or []
            if lb and rb and len(lb) == len(rb):
                pts = [((a["x"] + b2["x"]) / 2.0, (a["y"] + b2["y"]) / 2.0) for a, b2 in zip(lb, rb)]
                break
        if not pts:
            pts = [(b["x"], b["y"]), (e["x"], e["y"])]
        roads.append({
            "id": rd.get("id") or rd.get("name") or "",
            "p": (b["x"], b["y"]),
            "q": (e["x"], e["y"]),
            "pts": pts,
            "succ": list(rd.get("successor_ids") or []),
            "len": sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1)),
        })
    return roads


def nearest(roads, xy):
    best, bd = None, 1e9
    for i, r in enumerate(roads):
        for pt in (r["p"], r["q"]):
            d = math.hypot(pt[0] - xy[0], pt[1] - xy[1])
            if d < bd:
                bd, best = d, i
    return best, bd


def bfs(roads, s, t):
    import heapq
    dist = {s: 0.0}
    prev = {}
    pq = [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, 1e9):
            continue
        if u == t:
            break
        for vs in roads[u]["succ"]:
            v = None
            for i, r in enumerate(roads):
                if r["id"] == vs:
                    v = i
                    break
            if v is None:
                continue
            nd = d + roads[v]["len"]
            if nd < dist.get(v, 1e9):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if t not in dist:
        return None, None
    chain = [t]
    while chain[-1] != s:
        chain.append(prev[chain[-1]])
    chain.reverse()
    return chain, dist[t]


if __name__ == "__main__":
    roads = load()
    print("roads:", len(roads))
    a = sys.argv[1] if len(sys.argv) > 1 else "728.8,-9.7"
    b = sys.argv[2] if len(sys.argv) > 2 else "839.5,-63.7"
    src = tuple(float(v) for v in a.split(","))
    dst = tuple(float(v) for v in b.split(","))
    si, sd = nearest(roads, src)
    di, dd = nearest(roads, dst)
    print("src road %s (%.1f m)" % (roads[si]["id"], sd))
    print("dst road %s (%.1f m)" % (roads[di]["id"], dd))
    chain, total = bfs(roads, si, di)
    if not chain:
        print("NO PATH")
        raise SystemExit(1)
    print("path len %.1f m, %d roads" % (total, len(chain)))
    pts = []
    for i in chain:
        for pt in roads[i]["pts"]:
            if not pts or math.hypot(pt[0] - pts[-1][0], pt[1] - pts[-1][1]) > 0.2:
                pts.append(pt)
    print("waypoints:", len(pts))
    for pt in pts:
        print("  (%.1f, %.1f)," % pt)
