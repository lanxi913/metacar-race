# -*- coding: utf-8 -*-
"""从"已有的巡检照片 + 位姿日志"离线重建车辆盘点（不再跑仿真）。

输入：某个 vla_img* 目录（文件名含 t/x/y 的 640x480 帧）+ 对应的探针 json（含 ticks 的 pos/yaw）
输出：inventory json：{"region": [...], "cars": [{x, y, ori, p_violation, ...}]}

朝向不是猜的：对全部车点做 PCA 得到"排"的方向；车长轴与排方向垂直。
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, r"D:\无人算法\metacar-race")
from vla2.vision import detect_cars, horizon_row, imread_uni  # noqa: E402

A_PX = 334.0
F_PX = 554.0
RE_T = re.compile(r"_t(\d+)_")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True, help="探针 json（含 ticks）")
    ap.add_argument("--imgs", required=True, help="图片目录")
    ap.add_argument("--out", required=True)
    ap.add_argument("--cam", default="cam0")
    ap.add_argument("--step", type=int, default=2)
    args = ap.parse_args()

    data = json.loads(Path(args.json).read_text(encoding="utf-8"))
    by_t = {round(float(tk["t"]) * 10): tk for tk in (data.get("ticks") or [])}
    imgs = {}
    for p in Path(args.imgs).glob(f"*_{args.cam}.png"):
        m = RE_T.search(p.name)
        if m:
            imgs[int(m.group(1))] = p

    raw = []
    for i, (dec, p) in enumerate(sorted(imgs.items())):
        if i % args.step:
            continue
        tk = by_t.get(dec)
        if tk is None:
            continue
        ex, ey = float(tk["pos"][0]), float(tk["pos"][1])
        yaw = float(tk["yaw"])
        img = imread_uni(p)
        if img is None:
            continue
        h0 = horizon_row(img)
        for d in detect_cars(img):
            v = d["y"] + d["h"]
            if v <= h0 + 6:
                continue
            dist = A_PX / (v - h0)
            if not (1.0 <= dist <= 55.0):
                continue
            u = d["x"] + d["w"] / 2.0
            lat = (u - 320.0) * dist / F_PX
            th = math.radians(yaw)
            fx, fy = math.cos(th), -math.sin(th)
            lx, ly = math.sin(th), math.cos(th)
            raw.append((ex + dist * fx + lat * lx, ey + dist * fy + lat * ly, d["area"], dist))
    print(f"原始检测 {len(raw)}")
    if not raw:
        return 1

    pts = np.array([(r[0], r[1]) for r in raw], float)
    used = np.zeros(len(pts), bool)
    objs = []
    for i in range(len(pts)):
        if used[i]:
            continue
        m = np.hypot(pts[:, 0] - pts[i, 0], pts[:, 1] - pts[i, 1]) < 1.2
        used |= m
        idx = np.where(m)[0]
        w = np.array([max(raw[j][2], 1.0) for j in idx], float)
        dmin = min(raw[j][3] for j in idx)
        objs.append({
            "x": float(np.average(pts[idx, 0], weights=w)),
            "y": float(np.average(pts[idx, 1], weights=w)),
            "n": int(len(idx)),
            "dmin": float(dmin),
        })

    # 排方向：所有车点的主轴
    P = np.array([[o["x"], o["y"]] for o in objs], float)
    C = P - P.mean(axis=0)
    cov = C.T @ C / max(len(P) - 1, 1)
    w, v = np.linalg.eigh(cov)
    axis = v[:, int(np.argmax(w))]
    rows_deg = math.degrees(math.atan2(-axis[1], axis[0])) % 180.0   # 场景航向：y 向下为 90°
    car_ori = (rows_deg + 90.0) % 180.0
    print(f"排方向 {rows_deg:.1f}°（主轴解释方差 {w.max() / max(w.sum(), 1e-9):.2f}）"
          f" ⇒ 车长轴 {car_ori:.1f}°")

    x0, y0, x1, y1 = 6.1, 6.1, 46.1, 46.1
    cars = []
    for o in objs:
        inside = (x0 - 1.0 <= o["x"] <= x1 + 1.0) and (y0 - 1.0 <= o["y"] <= y1 + 1.0)
        legal = (-3.0 <= o["y"] <= 3.0) and (11.2 <= o["x"] <= 41.2)
        cars.append({
            "x": round(o["x"], 2), "y": round(o["y"], 2), "ori": round(car_ori, 1),
            "p_violation": 0.95 if inside else (0.05 if legal else 0.5),
            "p_label": 0.0, "label": "", "n_obs": o["n"], "dmin": round(o["dmin"], 2),
        })
    cars.sort(key=lambda c: (round(c["y"] / 4), c["x"]))
    print(f"合并后 {len(cars)} 辆车（禁停区内 {sum(1 for c in cars if c['p_violation'] > 0.9)} 辆）")
    for k, c in enumerate(cars, 1):
        print(f"  #{k} ({c['x']:+7.2f},{c['y']:+7.2f}) p_viol={c['p_violation']:.2f} obs={c['n_obs']} dmin={c['dmin']}")

    out = {"region": [x0, y0, x1, y1], "cars": cars,
           "note": "离线重建：位置来自相机反投影，朝向来自排方向主轴"}
    Path(args.out).write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print("->", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
