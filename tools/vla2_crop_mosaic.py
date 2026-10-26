# -*- coding: utf-8 -*-
"""把各巡检目录里“最大的若干车框”裁出来拼成一张图，用来看清车头/车侧到底有没有车牌和编号。"""
from __future__ import annotations

import glob
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, r"C:\Users\Administrator\Documents\deepseek-harness\default-workspace")
from vla2.vision import detect_cars, imread_uni, imwrite_uni  # noqa: E402


def main() -> int:
    pats = sys.argv[1].split(";")
    out = sys.argv[2]
    want = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    files: list[str] = []
    for p in pats:
        files += sorted(glob.glob(p))
    cands: list[tuple[int, str, tuple[int, int, int, int]]] = []
    for f in files[:: max(1, len(files) // 400)]:
        img = imread_uni(Path(f))
        if img is None:
            continue
        for b in detect_cars(img):
            if b["w"] * b["h"] >= 4000:          # 只看近的（面积够大）
                cands.append((b["area"], f, (b["x"], b["y"], b["w"], b["h"])))
    cands.sort(reverse=True)
    picked, seen = [], set()
    for a, f, box in cands:
        if f in seen:
            continue
        seen.add(f)
        picked.append((f, box))
        if len(picked) >= want:
            break
    tiles = []
    for f, (x, y, w, h) in picked:
        img = imread_uni(Path(f))
        pad = int(0.15 * max(w, h))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(img.shape[1], x + w + pad), min(img.shape[0], y + h + pad)
        crop = img[y0:y1, x0:x1]
        crop = crop if crop.shape[0] >= crop.shape[1] else crop
        scale = 240.0 / max(1, crop.shape[0])
        crop = np.asarray(_resize(crop, scale))
        tiles.append(crop)
        print(f"{Path(f).name} box=({x},{y},{w},{h})")
    H = max(t.shape[0] for t in tiles)
    W = sum(t.shape[1] for t in tiles) + 6 * len(tiles)
    canvas = np.full((H, W, 3), 30, np.uint8)
    x = 0
    for t in tiles:
        canvas[: t.shape[0], x:x + t.shape[1]] = t
        x += t.shape[1] + 6
    imwrite_uni(Path(out), canvas)
    print("->", out, canvas.shape)
    return 0


def _resize(img, scale: float):
    import cv2
    return cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


if __name__ == "__main__":
    raise SystemExit(main())
