# -*- coding: utf-8 -*-
"""离线核对「元素 1（车辆直行 8.2~59.2 m）」的车道与边界类型。

真机反馈「变道后压了双黄实线」，需要确认是
（a）整段左侧都是双黄（那就不该变道），还是
（b）段首是双黄、后面才变虚线（45% 单点取样骗了自己）。

数据 schema 与场景包一致：mapd 是 RoadInfo 列表，车道在 `laneData`，
边界在 `leftBorder/rightBorder` 的 `borderType` + `pathPoint`。

用法（metacar-race 目录下）：.venv\\Scripts\\python.exe tools\\_probe_elem1_corridor.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar_race.scene4.corridor import LaneCorridor  # noqa: E402

MAP = Path(r"C:\Users\Lenovo\AppData\Roaming\com.i-mylab.metapower-desktop\car\scenes\lianxu4"
           r"\SmartCar_Data\StreamingAssets\mapfile")
TYPE_NAME = {0: "-", 1: "双黄中线", 2: "侧线", 3: "实线", 4: "停止线", 5: "斑马线", 6: "虚线"}


def load(name: str):
    with (MAP / name).open(encoding="utf-8") as fh:
        return json.load(fh)


def build_static(raw_roads):
    roads = []
    for rd in raw_roads:
        lanes = []
        for ln in (rd.get("laneData") or []):
            lb = ln.get("leftBorder") or {}
            rb = ln.get("rightBorder") or {}
            lanes.append(NS(
                id=str(ln.get("id")),
                width=float(ln.get("width") or 0.0),
                path_points=[NS(x=float(p["x"]), y=float(p["y"])) for p in (ln.get("pathPoint") or [])],
                left_border=NS(type=int(lb.get("borderType") or 0),
                               path_points=[NS(x=float(p["x"]), y=float(p["y"]))
                                            for p in (lb.get("pathPoint") or [])]),
                right_border=NS(type=int(rb.get("borderType") or 0),
                                path_points=[NS(x=float(p["x"]), y=float(p["y"]))
                                             for p in (rb.get("pathPoint") or [])]),
                left_lane_id=str(ln.get("leftLane") or ""),
                right_lane_id=str(ln.get("rightLane") or ""),
            ))
        roads.append(NS(id=str(rd.get("id")), lanes=lanes))
    return roads


def main() -> int:
    roads = build_static(load("mapd"))
    route = [NS(x=float(p["x"]), y=float(p["y"])) for p in load("route")]
    static = NS(roads=roads, route=route, sub_scenes=[])
    cum = [0.0] * len(route)
    for i in range(1, len(route)):
        cum[i] = cum[i - 1] + math.hypot(route[i].x - route[i - 1].x, route[i].y - route[i - 1].y)
    n_lane = sum(len(r.lanes) for r in roads)
    print(f"roads={len(roads)} lanes={n_lane} route_pts={len(route)} total={cum[-1]:.1f} m")

    cor = LaneCorridor(static, ego_width=1.81, margin=0.25)

    def idx_at(s: float) -> int:
        lo, hi = 0, len(cum) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if cum[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        return lo

    print("\n元素 1（车辆直行 8.2~59.2 m）逐点走廊：")
    for s in range(8, 61, 4):
        i = idx_at(s)
        p = route[i]
        room = cor.room(p.x, p.y)
        if room is None:
            print(f"  s={s:3d} 无车道 (x={p.x:.1f}, y={p.y:.1f})")
            continue
        nb_l = cor.neighbor(p.x, p.y, +1)
        nb_r = cor.neighbor(p.x, p.y, -1)
        nl = "-" if nb_l is None else f"{nb_l['shift_m']:.2f}m/{nb_l['lane_id']}"
        nr = "-" if nb_r is None else f"{nb_r['shift_m']:.2f}m/{nb_r['lane_id']}"
        print(f"  s={s:3d} lane={room.lane_id:<4} 宽={room.lane_width:.2f} "
              f"左={TYPE_NAME.get(room.left_type, room.left_type):<4} "
              f"右={TYPE_NAME.get(room.right_type, room.right_type):<4} "
              f"可左变={nl:<12} 可右变={nr}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
