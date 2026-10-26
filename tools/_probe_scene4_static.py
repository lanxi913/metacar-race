# -*- coding: utf-8 -*-
"""静态数据探测：连上场景后把「路线 / 元素 / 车道图」全部导出来，并和上一版快照对比。

为什么要它：2026-10-25 14:08 客户端把「连续场景4」的场景包更新了（SmartCar.exe 14:08:59
被重写），路线/元素/车道图可能都变了。变道和切弯两个策略都依赖这些几何，
所以在改任何一处之前先把「新版本长什么样、和上一版差在哪」量出来。

跑法（脚本先起、再点客户端「启动场景」，顺序不能反）：

    .venv\\Scripts\\python.exe tools\\_probe_scene4_static.py --out ...\\scenes\\lianxu4-new.json

产物：控制台打印摘要 + 一份 JSON 快照（含 route/sub_scenes/roads 统计 + 每个元素的车道摘要）。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import SceneAPI  # noqa: E402

from metacar_race.scene4.corridor import LaneCorridor  # noqa: E402

TYPE_NAME = {0: "无线", 1: "双黄", 2: "侧线", 3: "实线", 4: "停止线", 5: "斑马线", 6: "虚线"}


def g(obj, *names, default=None):
    for n in names:
        if obj is None:
            break
        if isinstance(obj, dict):
            if n in obj:
                return obj[n]
        elif hasattr(obj, n):
            return getattr(obj, n)
    return default


def as_int(val) -> int:
    """LineType 之类的 Enum 也能转成 int（`int(LineType.DASH_LINE)` 会抛 TypeError）。"""
    if val is None:
        return 0
    return int(getattr(val, "value", val))


def xy(p):
    return float(g(p, "x", "pos_x", default=0.0)), float(g(p, "y", "pos_y", default=0.0))


def route_stats(route):
    pts = [xy(p) for p in route]
    cum = [0.0] * len(pts)
    for i in range(1, len(pts)):
        cum[i] = cum[i - 1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
    return pts, cum


def s_of(pts, cum, p):
    """把点投影到路线上，返回里程。"""
    x, y = p
    best = None
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        vx, vy = bx - ax, by - ay
        L = math.hypot(vx, vy)
        if L <= 1e-9:
            continue
        ux, uy = vx / L, vy / L
        t = max(0.0, min(L, (x - ax) * ux + (y - ay) * uy))
        px, py = ax + ux * t, ay + uy * t
        d = math.hypot(x - px, y - py)
        if best is None or d < best[0]:
            best = (d, cum[i] + t)
    return 0.0 if best is None else best[1]


def lane_summary(static, pts, cum, s_list):
    """在若干里程处查走廊：本车道 id/宽度/左右线型，以及左右能否变道。"""
    try:
        cor = LaneCorridor(static, ego_width=1.81, margin=0.25)
    except Exception as exc:  # noqa: BLE001
        return [{"error": repr(exc)}]
    out = []
    for s in s_list:
        i = 0
        for k, c in enumerate(cum):
            if c >= s:
                i = k
                break
        x, y = pts[i]
        room = cor.room(x, y)
        nb_l = cor.neighbor(x, y, +1)
        nb_r = cor.neighbor(x, y, -1)
        out.append({
            "s": round(s, 1),
            "lane": None if room is None else room.lane_id,
            "width": None if room is None else round(room.lane_width, 2),
            "left_type": None if room is None else room.left_type,
            "right_type": None if room is None else room.right_type,
            "left_name": None if room is None else TYPE_NAME.get(room.left_type, room.left_type),
            "right_name": None if room is None else TYPE_NAME.get(room.right_type, room.right_type),
            "can_left": None if nb_l is None else round(nb_l["shift_m"], 2),
            "can_right": None if nb_r is None else round(nb_r["shift_m"], 2),
        })
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="", help="快照 JSON 路径（默认 accel_window_probe\\scenes\\<时间戳>.json）")
    ap.add_argument("--baseline", default="", help="上一版快照 JSON，给了就打印差异")
    args = ap.parse_args()

    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()

    route = list(g(static, "route", default=[]) or [])
    pts, cum = route_stats(route)
    print(f"[route] 点数 {len(pts)}，总长 {cum[-1]:.1f} m，起点 {pts[0]}, 终点 {pts[-1]}")
    step = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1)]
    if step:
        step_sorted = sorted(step)
        print(f"[route] 点距 中位 {step_sorted[len(step_sorted) // 2]:.3f} m，"
              f"最小 {step_sorted[0]:.3f}，最大 {step_sorted[-1]:.3f}")

    subs = list(g(static, "sub_scenes", default=[]) or [])
    sub_rows = []
    print(f"[elem] 共 {len(subs)} 个元素")
    for k, sc in enumerate(subs):
        name = str(g(sc, "name", default=""))
        sp = g(sc, "start_point", default=None)
        ep = g(sc, "end_point", default=None)
        s0 = s_of(pts, cum, xy(sp)) if sp is not None else float("nan")
        s1 = s_of(pts, cum, xy(ep)) if ep is not None else float("nan")
        sub_rows.append({"i": k + 1, "name": name, "s0": round(s0, 1), "s1": round(s1, 1),
                         "len": round(s1 - s0, 1)})
        print(f"  {k + 1:2d}. {name:<12} {s0:8.1f} ~ {s1:8.1f} m  长 {s1 - s0:7.1f} m")

    roads = list(g(static, "roads", default=[]) or [])
    lanes = []
    for rd in roads:
        lanes.extend(list(g(rd, "lanes", default=[]) or []))
    widths = [float(g(ln, "width", default=0.0) or 0.0) for ln in lanes]
    lt = Counter(as_int(g(g(ln, "left_border", default=None), "type", default=0)) for ln in lanes)
    rt = Counter(as_int(g(g(ln, "right_border", default=None), "type", default=0)) for ln in lanes)
    print(f"[map] 车道 {len(roads)} 条路 / {len(lanes)} 条车道；"
          f"宽度 中位 {sorted(widths)[len(widths) // 2]:.2f} m" if widths else "[map] 无车道")
    print(f"[map] 左边线类型 {dict(lt)}")
    print(f"[map] 右边线类型 {dict(rt)}")

    # 每个元素中点查一次走廊（了解边界有没有变）
    probes = []
    for row in sub_rows:
        mid = (row["s0"] + row["s1"]) / 2.0
        if math.isnan(mid):
            continue
        probes.extend(lane_summary(static, pts, cum, [row["s0"] + 3.0, mid]))
    print("[lane] 元素内走廊抽样（前 12 条）：")
    for p in probes[:12]:
        if "error" in p:
            print("  ", p)
            continue
        print(f"  s={p['s']:7.1f} lane={str(p['lane']):<5} w={p['width']} 左={p['left_name']} "
              f"右={p['right_name']} 可左={p['can_left']} 可右={p['can_right']}")

    snap = {
        "when": datetime.now().isoformat(timespec="seconds"),
        "route_points": len(pts),
        "route_len_m": round(cum[-1], 1),
        "route_first": pts[0],
        "route_last": pts[-1],
        "roads": len(roads),
        "lanes": len(lanes),
        "width_median": (sorted(widths)[len(widths) // 2] if widths else None),
        "left_types": dict(lt),
        "right_types": dict(rt),
        "sub_scenes": sub_rows,
        "lane_probe": probes,
        "lanes_raw": [
            {
                "road": str(g(rd, "id", default="")),
                "lane": str(g(ln, "id", default="")),
                "width": round(float(g(ln, "width", default=0.0) or 0.0), 2),
                "left_type": as_int(g(g(ln, "left_border", default=None), "type", default=0)),
                "right_type": as_int(g(g(ln, "right_border", default=None), "type", default=0)),
                "left_id": str(g(ln, "left_lane_id", default="") or ""),
                "right_id": str(g(ln, "right_lane_id", default="") or ""),
                "n_pts": len(list(g(ln, "path_points", default=[]) or [])),
            }
            for rd in roads for ln in (g(rd, "lanes", default=[]) or [])
        ],
    }
    out = Path(args.out) if args.out else (REPO.parent / "accel_window_probe" / "scenes"
                                           / f"scene4-{datetime.now():%Y%m%d-%H%M%S}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(snap, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[out] 快照 {out}")

    if args.baseline and Path(args.baseline).exists():
        base = json.loads(Path(args.baseline).read_text(encoding="utf-8"))
        print("\n[diff] 与上一版比：")
        for key in ("route_points", "route_len_m", "roads", "lanes", "width_median"):
            a, b = base.get(key), snap.get(key)
            print(f"  {key}: {a} -> {b}" + ("" if a == b else "   <-- 变了"))
        bn = {r["i"]: r for r in base.get("sub_scenes", [])}
        an = {r["i"]: r for r in snap["sub_scenes"]}
        for i in sorted(set(bn) | set(an)):
            b, a = bn.get(i), an.get(i)
            if b is None or a is None:
                print(f"  元素{i}: {b} -> {a}   <-- 增删")
                continue
            flag = "" if (b["name"] == a["name"] and abs(b["s0"] - a["s0"]) < 0.5
                          and abs(b["s1"] - a["s1"]) < 0.5) else "   <-- 变了"
            if flag:
                print(f"  元素{i}: {b['name']} {b['s0']}~{b['s1']} -> {a['name']} {a['s0']}~{a['s1']}{flag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
