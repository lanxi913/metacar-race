# -*- coding: utf-8 -*-
"""离线自检：拿已有巡检图验 `vla2/vision.py` 的检测 + 地面分类 + 读号。

    python tools/vla2_selftest.py <图片或目录> [--save 输出目录] [--limit N]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vla2.vision import (detect_cars, ground_class, imread_uni,  # noqa: E402
                         imwrite_uni, read_label)


def run_one(path: Path, save: Path | None):
    img = imread_uni(path)
    if img is None:
        print(f"{path.name}: read fail")
        return
    dets = detect_cars(img)
    vis = img.copy()
    print(f"\n{path.name}: {len(dets)} cars")
    for d in dets:
        g = ground_class(img, d)
        text, score, raw = read_label(img, d)
        col = (0, 0, 255) if g.get("grid") else (0, 200, 0)
        cv2.rectangle(vis, (d["x"], d["y"]), (d["x"] + d["w"], d["y"] + d["h"]), col, 2)
        tag = f"{text or '?'} s{score:.2f} dark{g.get('dark')} cy{g.get('cyan')} {raw}"
        cv2.putText(vis, tag[:44], (max(0, d["x"] - 40), max(14, d["y"] - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
        print(f"  {d['w']:>3}x{d['h']:<3} @{d['x']:>3},{d['y']:<3} "
              f"grid={g.get('grid')} dark={g.get('dark')} cyan={g.get('cyan')} "
              f"label={text} raw={raw} score={score}")
    if save is not None:
        save.mkdir(parents=True, exist_ok=True)
        imwrite_uni(save / path.name.replace(".png", "_v2.jpg"), vis)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--save", default=None)
    ap.add_argument("--limit", type=int, default=12)
    ap.add_argument("--step", type=int, default=8)
    args = ap.parse_args()
    root = Path(args.src)
    files = [root] if root.is_file() else sorted(root.glob("*cam0.png"))[:: args.step][: args.limit]
    save = Path(args.save) if args.save else None
    for p in files:
        run_one(p, save)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
