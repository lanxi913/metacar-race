"""车顶编号牌检测（黄底、隔离、矩形、牌面有暗字）。

关键在「隔离」：沙地也是黄的，但它们连成大片；真正的牌是悬在天空/蓝地上的独立小块。
"""

from __future__ import annotations

import cv2
import numpy as np


def yellow_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    m = ((h >= 16) & (h <= 45) & (s >= 70) & (v >= 95)).astype(np.uint8) * 255
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def find_plates(bgr: np.ndarray, min_w: int = 12, max_w: int = 95,
                ring_frac: float = 0.22) -> list[tuple[int, int, int, int, int]]:
    """返回 (u, v, w, h, area)，按面积降序。"""
    m = yellow_mask(bgr)
    n, _, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    hgt, wid = bgr.shape[:2]
    out = []
    for i in range(1, n):
        u, v, w, h, a = (int(stats[i, 0]), int(stats[i, 1]), int(stats[i, 2]),
                         int(stats[i, 3]), int(stats[i, 4]))
        if not (min_w <= w <= max_w) or not (6 <= h <= 42):
            continue
        if a < 90 or a > 2600:
            continue
        ar = w / max(h, 1)
        if not (1.2 <= ar <= 5.0):
            continue
        if a / float(w * h) < 0.6:
            continue
        # 环带测试：牌四周应当是「非黄」（天空/车身/地面涂装）
        px, py = int(0.5 * w), int(0.5 * h)
        x0, y0 = max(0, u - px), max(0, v - py)
        x1, y1 = min(wid, u + w + px), min(hgt, v + h + py)
        ring = m[y0:y1, x0:x1]
        if ring.size == 0:
            continue
        inner = np.zeros_like(ring)
        inner[v - y0: v - y0 + h, u - x0: u - x0 + w] = 255
        ring_only = cv2.bitwise_and(ring, cv2.bitwise_not(inner))
        frac = float((ring_only > 0).mean())
        if frac > ring_frac:
            continue
        # 牌面有暗字
        roi = cv2.cvtColor(bgr[v:v + h, u:u + w], cv2.COLOR_BGR2GRAY)
        dark = float((roi < 95).mean())
        if not (0.03 <= dark <= 0.7):
            continue
        out.append((u, v, w, h, a))
    out.sort(key=lambda z: -z[4])
    return out
