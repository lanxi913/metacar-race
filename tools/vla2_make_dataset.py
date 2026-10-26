# -*- coding: utf-8 -*-
"""从已有巡检帧自动打标，生成 YOLO 数据集（class 0 = 车，class 1 = 蓝色车牌/车顶小牌）。

标注来源：
- 车：现成的颜色连通域检测器 `vla2.vision.detect_cars`（已实测少误检、能抓到近车）
- 车牌：HSV 蓝牌掩膜 + 形状过滤（宽 6~60 px、高 4~28 px、宽高比 1.1~5、填充率 ≥0.45）

用法：
    python tools/vla2_make_dataset.py --out D:\\vla_ds --dirs "D:\\...\\vla_imgss5;D:\\...\\vla_imgss6" --per-dir 120
"""
from __future__ import annotations

import argparse
import glob
import random
from pathlib import Path

import cv2
import numpy as np

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vla2.vision import detect_cars, imread_uni, imwrite_uni  # noqa: E402

PLATE_HUE = (100, 135)      # OpenCV H：蓝→紫蓝
PLATE_S_MIN = 90
PLATE_V_MIN = 45


def plate_boxes(img: np.ndarray) -> list[tuple[int, int, int, int]]:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (PLATE_HUE[0], PLATE_S_MIN, PLATE_V_MIN), (PLATE_HUE[1], 255, 255))
    k = np.ones((2, 2), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if not (6 <= w <= 60 and 4 <= h <= 28):
            continue
        ar = w / max(1, h)
        if not (1.1 <= ar <= 5.0):
            continue
        if a / float(w * h) < 0.45:
            continue
        out.append((int(x), int(y), int(w), int(h)))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--dirs", required=True, help="分号分隔的目录")
    ap.add_argument("--per-dir", type=int, default=120)
    ap.add_argument("--val-ratio", type=float, default=0.15)
    ap.add_argument("--min-area", type=int, default=380)
    args = ap.parse_args()

    out = Path(args.out)
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        (out / sub).mkdir(parents=True, exist_ok=True)

    files: list[str] = []
    for d in args.dirs.split(";"):
        g = sorted(glob.glob(str(Path(d) / "*cam0.png")))
        if not g:
            print(f"[skip] {d} 没有 cam0 帧")
            continue
        step = max(1, len(g) // args.per_dir)
        files += g[::step][: args.per_dir]
    print(f"共 {len(files)} 帧")
    random.seed(7)
    random.shuffle(files)

    n_car = n_plate = 0
    for i, f in enumerate(files):
        img = imread_uni(Path(f))
        if img is None:
            continue
        h, w = img.shape[:2]
        lines = []
        for b in detect_cars(img):
            if b["area"] < args.min_area:
                continue
            cx = (b["x"] + b["w"] / 2.0) / w
            cy = (b["y"] + b["h"] / 2.0) / h
            lines.append(f"0 {cx:.6f} {cy:.6f} {b['w'] / w:.6f} {b['h'] / h:.6f}")
            n_car += 1
        for (x, y, bw, bh) in plate_boxes(img):
            cx = (x + bw / 2.0) / w
            cy = (y + bh / 2.0) / h
            lines.append(f"1 {cx:.6f} {cy:.6f} {bw / w:.6f} {bh / h:.6f}")
            n_plate += 1
        split = "val" if (i % 100) < args.val_ratio * 100 else "train"
        stem = Path(f).stem
        imwrite_uni(out / "images" / split / f"{stem}.jpg", img)
        (out / "labels" / split / f"{stem}.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    yml = out / "data.yaml"
    yml.write_text(
        f"path: {out.as_posix()}\ntrain: images/train\nval: images/val\n"
        f"names:\n  0: car\n  1: plate\n",
        encoding="utf-8",
    )
    print(f"完成：车框 {n_car} 个，车牌框 {n_plate} 个 -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
