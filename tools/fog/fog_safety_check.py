# -*- coding: utf-8 -*-
"""港口迷雾：路径离墙距离体检（离线，不用开场景）。

用 fog_dump_static.py 存下来的 static.json，把任务里每一条要走的路线都算一遍
"离最近的实体障碍还有多远"。小于车半宽（1.23 m）+ 余量就是会撞的距离。

用法：
  python -u fog_safety_check.py --static ..\\..\\logs\\static1\\static.json
"""
import argparse
import json
import math

from fog_map import RoadGraph, load_static
import fog_route

CAR_HALF_W = 1.23          # Car_C 车宽 2.45 m
WARN_M = 1.60              # 车半宽 + 0.37 m 余量：低于它就算擦墙风险
HARD_M = 1.30              # 低于它必撞

# 这些类别是"实体"，撞上就判碰撞；bollard/lamp_post/sign_post 是细杆，
# 也列出来但单独统计（车压过细杆同样算碰撞，只是路线常常不得不贴近）
SOLID = {"warehouse", "storage_tank", "quay_crane", "construction_barrier",
         "building", "solid_obstacle", "task_terminal", "container"}
THIN = {"bollard", "lamp_post", "sign_post", "landmark"}


def rect_dist(px, py, o):
    """点到（可旋转）矩形的距离；在矩形内返回 0。"""
    p = o["position"]
    cx, cy = float(p["x"]), float(p["y"])
    L = float(o.get("length_m") or 0.0)
    W = float(o.get("width_m") or 0.0)
    ori = o.get("orientation_degrees") or {}
    yaw = math.radians(float(ori.get("z", 0.0) or 0.0))
    dx, dy = px - cx, py - cy
    ca, sa = math.cos(-yaw), math.sin(-yaw)
    u = ca * dx - sa * dy
    v = sa * dx + ca * dy
    du = max(abs(u) - 0.5 * L, 0.0)
    dv = max(abs(v) - 0.5 * W, 0.0)
    return math.hypot(du, dv)


def nearest_on_path(pts, obs, step=0.5):
    """沿折线每 step 米采样，返回 (最小距离, 采样点, 障碍)。"""
    best = (1e9, None, None)
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        n = max(1, int(seg / step))
        for k in range(n + 1):
            x = ax + (bx - ax) * k / n
            y = ay + (by - ay) * k / n
            for o in obs:
                d = rect_dist(x, y, o)
                if d < best[0]:
                    best = (d, (x, y), o)
    return best


def report(name, pts, solids, thins):
    d_s, at_s, o_s = nearest_on_path(pts, solids)
    d_t, at_t, o_t = nearest_on_path(pts, thins)
    flag = "撞" if d_s < HARD_M else ("擦" if d_s < WARN_M else "ok")
    print("%-22s %-3s 距实体 %5.2f m @(%6.1f,%6.1f) %-26s | 距细杆 %5.2f m %s"
          % (name, flag, d_s, at_s[0] if at_s else 0, at_s[1] if at_s else 0,
             (o_s or {}).get("id", "-"), d_t, (o_t or {}).get("id", "-")))
    return d_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--static", required=True)
    args = ap.parse_args()
    static = load_static(args.static)
    graph = RoadGraph(static)
    objs = static.get("static_objects") or []
    solids = [o for o in objs if o.get("category") in SOLID]
    thins = [o for o in objs if o.get("category") in THIN]
    print("实体障碍 %d 个、细杆 %d 个" % (len(solids), len(thins)))
    print("门限：<%.2f m 必撞、<%.2f m 擦墙风险" % (HARD_M, WARN_M))

    worst = []
    legs = [
        ("Car_C 起点->A", (618.5, 120.8), (587.5, 124.8)),
        ("A->B", (587.5, 124.8), (766.5, -34.5)),
        ("B->C", (766.5, -34.5), (720.5, -96.0)),
        ("C->路障区东", (720.5, -96.0), (766.5, -88.0)),
        ("运输车起->闸口南", (618.5, 120.8), (728.8, -42.0)),
        ("闸口北->泊位A", (728.8, 20.0), (630.9, -65.7)),
        ("闸口北->泊位B", (728.8, 20.0), (834.5, -65.7)),
    ]
    for name, a, b in legs:
        r = fog_route.route(graph, a, b)
        if r is None:
            print("%-22s 规划失败" % name)
            continue
        worst.append((report(name, r.pts, solids, thins), name, len(r.pts)))

    print()
    print("手工折线（stage_cross / stage_deliver 里写死的那些）：")
    manual = {
        "gate_line": [(728.79, -31.0), (728.79, -24.0), (728.79, -15.0), (728.79, -6.0),
                      (728.79, 4.0), (728.79, 20.0), (728.79, 46.0)],
        "south_in": [(728.8, -58.0), (728.8, -50.0), (728.8, -44.0), (728.8, -42.0)],
        "west_ca": [(728.8, -9.9), (728.8, 3.0), (724.0, 4.7), (718.0, 4.7), (714.6, 0.0),
                    (714.6, -20.0), (714.6, -33.2), (712.0, -46.0), (704.2, -46.2),
                    (680.0, -46.2), (655.0, -46.2), (648.7, -46.2), (644.0, -50.0),
                    (638.4, -53.2), (636.0, -58.0), (633.5, -62.0)],
        "west_cc": [(638.4, 120.8), (638.4, 60.0), (638.4, -20.0), (638.4, -48.0),
                    (643.0, -52.0), (641.0, -59.0)],
        "north_b": [(728.8, 6.0), (743.3, 4.7), (790.0, 4.7), (790.0, -30.0),
                    (812.0, -40.0), (830.0, -56.0), (834.0, -63.8)],
        "from_a": [(636.0, -58.5), (638.4, -40.0), (638.4, 4.7), (700.0, 4.7),
                   (790.0, 4.7), (800.0, -10.0), (812.0, -30.0), (838.5, -49.5),
                   (838.5, -57.0)],
    }
    for name, pts in manual.items():
        worst.append((report(name, pts, solids, thins), name, len(pts)))

    print()
    worst.sort()
    bad = [w for w in worst if w[0] < WARN_M]
    if bad:
        print("**需要处理的路线**：")
        for d, name, n in bad:
            print("   %-22s %.2f m" % (name, d))
    else:
        print("所有路线离实体障碍都 >= %.2f m" % WARN_M)


if __name__ == "__main__":
    main()
