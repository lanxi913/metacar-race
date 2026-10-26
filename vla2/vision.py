# -*- coding: utf-8 -*-
"""VLA 停车区违规检测（任务二）—— 视觉感知层。

三件事，全部"现场量出来"，不写死任何一辆车：
1. `detect_cars`  车漆连通域（砂地低饱和、网格涂装被剔除、天空按地平线裁掉）
2. `ground_class` 车体接地一圈的地面属性：亮砂地（合法）/ 暗沥青+青色网格（限制区）
3. `read_label`   车身侧面白字贴纸 `X-dd`：白掩膜 → 列投影切字 → 归一化模板匹配
   （字母位只比字母模板、数字位只比数字模板，避开 3/S、0/O 混淆）
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------- 基础工具


def imread_uni(path):
    """OpenCV 的 imread 在中文路径下会失败（narrow-char），改用 numpy 读字节再解码。"""
    try:
        buf = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return cv2.imdecode(buf, cv2.IMREAD_COLOR) if buf.size else None


def imwrite_uni(path, img, quality=90) -> bool:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if ok:
        buf.tofile(str(path))
    return bool(ok)


def horizon_row(bgr: np.ndarray) -> int:
    """地平线行：天空高饱和、砂地低饱和，第一行平均饱和度掉下来就是地平线。"""
    s = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 1].astype(np.float32)
    row_s = s.mean(axis=1)
    h = bgr.shape[0]
    for y in range(int(h * 0.15), int(h * 0.85)):
        if row_s[y] < 75.0 and row_s[y:min(h, y + 12)].mean() < 75.0:
            return y
    return int(h * 0.45)


def colour_masks(bgr: np.ndarray):
    """返回（车漆掩膜, 网格涂装掩膜）。只在路面区域里找。OpenCV 的 H 是 0~180。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0].astype(np.int16)
    s = hsv[:, :, 1].astype(np.int16)
    v = hsv[:, :, 2].astype(np.int16)
    y0 = horizon_row(bgr)
    ground = np.zeros(bgr.shape[:2], bool)
    ground[y0 + 2:, :] = True
    hatch = (((h >= 78) & (h <= 110)) | ((h >= 18) & (h <= 34))) & (s > 120) & (v > 120) & ground
    car = (s > 110) & (v > 25) & (v < 235) & ground & ~hatch
    car = car.astype(np.uint8) * 255
    car = cv2.morphologyEx(car, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    car = cv2.morphologyEx(car, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return car, hatch.astype(np.uint8) * 255


# ---------------------------------------------------------------- 车辆检测


def detect_cars(bgr: np.ndarray, min_area=380, max_area=60000):
    """返回 [{'x','y','w','h','area'}]，按面积从大到小。"""
    car, _ = colour_masks(bgr)
    H, W = bgr.shape[:2]
    n, _lab, stats, _cent = cv2.connectedComponentsWithStats(car, 8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_area or area > max_area:
            continue
        if w < 14 or w > 300:
            continue
        if area / float(w * h) < 0.40:          # 车体是实心的，砂砾碎斑是毛刺
            continue
        if y + h > H * 0.94:                    # 贴到画面最下缘的是车前地面
            continue
        if not (0.25 <= h / max(w, 1) <= 4.0):
            continue
        out.append(dict(x=int(x), y=int(y), w=int(w), h=int(h), area=int(area)))
    out.sort(key=lambda d: -d["area"])
    return out


def ground_class(bgr: np.ndarray, box, pad_ratio=0.35):
    """车体接地一圈的地面属性。

    返回 dict(dark=亮度中位数, cyan=青色占比, grid=是否判为限制区)。
    暗沥青（网格区）亮度 ≈ 90~115，亮砂地（合法区）≈ 145~200；
    青色网格线在限制区覆盖率 ≈ 0.15~0.25，砂地 ≈ 0.00~0.03。
    """
    car, hatch = colour_masks(bgr)
    v = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 2]
    H, W = v.shape
    x, y, w, h = box["x"], box["y"], box["w"], box["h"]
    pad = max(6, int(pad_ratio * w))
    x0, x1 = max(0, x - pad), min(W, x + w + pad)
    y0 = min(H - 2, y + int(h * 0.55))          # 车体中下部 → 接地附近
    y1 = min(H, y + h + max(8, int(0.9 * h)))
    if y1 <= y0 + 2 or x1 <= x0 + 2:
        return dict(dark=-1.0, cyan=0.0, grid=None, n=0)
    vv = v[y0:y1, x0:x1].astype(np.float32)
    hh = hatch[y0:y1, x0:x1] > 0
    cc = car[y0:y1, x0:x1] > 0
    keep = ~cc
    px = vv[keep]
    dark = float(np.median(px)) if px.size > 20 else -1.0
    cyan = float((hh & keep).mean()) if hh.size else 0.0
    grid = None
    if px.size > 20:
        grid = bool(dark < 125.0 or cyan > 0.10)
    return dict(dark=round(dark, 1), cyan=round(cyan, 3), grid=grid, n=int(px.size))


# ---------------------------------------------------------------- 贴纸编号

_DIGITS = "0123456789"
_LETTERS = "ABCDEFGHJKLMNPQRSTUVWXYZ"      # 去掉易混的 I / O
_FONT_CANDIDATES = [
    r"C:\Windows\Fonts\arialbd.ttf",
    r"C:\Windows\Fonts\arial.ttf",
    r"C:\Windows\Fonts\seguisb.ttf",
    r"C:\Windows\Fonts\consolab.ttf",
]
_GW, _GH = 34, 32


def _norm_glyph(crop):
    h, w = crop.shape[:2]
    if h == 0 or w == 0:
        return None
    nw = min(_GW, max(1, int(round(w * _GH / float(h)))))
    g = cv2.resize(crop, (nw, _GH), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((_GH, _GW), np.uint8)
    x0 = (_GW - nw) // 2
    canvas[:, x0:x0 + nw] = g
    return canvas


def _render(chars):
    font_path = next((p for p in _FONT_CANDIDATES if Path(p).exists()), None)
    if font_path is None:
        return {}
    out = {}
    for ch in chars:
        img = Image.new("L", (_GW * 3, _GH * 3), 0)
        ImageDraw.Draw(img).text((_GW * 0.5, _GH * 0.6), ch, fill=255,
                                 font=ImageFont.truetype(font_path, int(_GH * 1.55)))
        a = np.array(img)
        ys, xs = np.nonzero(a > 60)
        if xs.size == 0:
            continue
        n = _norm_glyph(a[ys.min():ys.max() + 1, xs.min():xs.max() + 1])
        if n is not None:
            out[ch] = n
    return out


_TEMPLATES = None


def _templates():
    global _TEMPLATES
    if _TEMPLATES is None:
        _TEMPLATES = dict(letter=_render(_LETTERS), digit=_render(_DIGITS))
    return _TEMPLATES


def _match(crop, table):
    g = _norm_glyph(crop)
    if g is None or not table:
        return "?", -2.0
    g = g.astype(np.float32)
    g = (g - g.mean()) / (g.std() + 1e-6)
    best, bch = -2.0, "?"
    for ch, t in table.items():
        tt = t.astype(np.float32)
        tt = (tt - tt.mean()) / (tt.std() + 1e-6)
        sc = float((g * tt).mean())
        if sc > best:
            best, bch = sc, ch
    return bch, round(best, 3)


def _white_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return (((hsv[:, :, 2] > 175) & (hsv[:, :, 1] < 55)).astype(np.uint8)) * 255


def _glyph_candidates(mask):
    """把白掩膜里"像字"的连通域挑出来（白线、大块反光会被形状过滤掉）。"""
    n, _lab, stats, _cent = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if h < 7 or h > 70:
            continue
        if w < 1 or w > 60:
            continue
        if area / float(w * h) < 0.10:
            continue
        out.append(dict(x=int(x), y=int(y), w=int(w), h=int(h), area=int(area)))
    return out


def _clusters(cands, gap_ratio=0.9, band_ratio=0.45):
    """按"同一行 + 水平相邻"把字聚成一组，返回 [[cand,...], ...]（按 x 排序）。"""
    cands = sorted(cands, key=lambda c: c["x"])
    used = [False] * len(cands)
    groups = []
    for i, a in enumerate(cands):
        if used[i]:
            continue
        grp = [a]
        used[i] = True
        changed = True
        while changed:
            changed = False
            gy = [g["y"] + g["h"] * 0.5 for g in grp]
            gh = max(g["h"] for g in grp)
            cy = float(np.mean(gy))
            for j, b in enumerate(cands):
                if used[j]:
                    continue
                bc = b["y"] + b["h"] * 0.5
                if abs(bc - cy) > band_ratio * gh:
                    continue
                if abs(b["h"] - gh) > 0.45 * gh:
                    continue
                near = any(abs(b["x"] - (g["x"] + g["w"])) <= gap_ratio * gh
                           or abs(g["x"] - (b["x"] + b["w"])) <= gap_ratio * gh
                           or abs(b["x"] - g["x"]) <= 0.6 * gh for g in grp)
                if near:
                    grp.append(b)
                    used[j] = True
                    changed = True
        if len(grp) >= 2:
            groups.append(sorted(grp, key=lambda c: c["x"]))
    return groups


def _read_group(mask, group):
    tab = _templates()
    heights = sorted(g["h"] for g in group)
    med = heights[len(heights) // 2]
    chars, scores = [], []
    dash_seen = False
    for i, g in enumerate(group):
        crop = mask[g["y"]:g["y"] + g["h"], g["x"]:g["x"] + g["w"]]
        if g["h"] < 0.45 * med and g["w"] >= 1.2 * g["h"]:
            chars.append("-")
            scores.append(1.0)
            dash_seen = True
            continue
        if not dash_seen and i == 0:
            table = tab["letter"]
        elif dash_seen:
            table = tab["digit"]
        else:
            table = dict(tab["letter"], **tab["digit"])
        ch, sc = _match(crop, table)
        chars.append(ch)
        scores.append(sc)
    text = "".join(chars)
    return text, (float(np.mean(scores)) if scores else -2.0)


def read_label(bgr: np.ndarray, box=None, margin_ratio=2.0, min_score=0.55):
    """读贴纸编号 `X-dd`：白光斑 → 形状过滤 → 同行聚类 → 位置感知模板匹配。

    box 给车框就只在框附近找，否则扫全图。返回 (text|None, score, glyphs_text)。
    """
    if box is not None:
        x, y, w, h = box["x"], box["y"], box["w"], box["h"]
        mx, my = int(margin_ratio * w), int(1.4 * h)
        x0, x1 = max(0, x - mx), min(bgr.shape[1], x + w + mx)
        y0, y1 = max(0, y - my), min(bgr.shape[0], y + h + my)
        roi = bgr[y0:y1, x0:x1]
    else:
        roi = bgr
    if roi.size == 0:
        return None, -2.0, ""
    mask = _white_mask(roi)
    groups = _clusters(_glyph_candidates(mask))
    best = (None, -2.0, "")
    for grp in groups:  # noqa: SIM110
        if not (2 <= len(grp) <= 6):
            continue
        text, score = _read_group(mask, grp)
        # 连字符太细会被形状过滤掉，这里补两种合法形态
        if len(text) == 3 and text[0] in _LETTERS and text[1:].isdigit():
            text = text[0] + "-" + text[1:]
        ok = (len(text) == 4 and text[0] in _LETTERS and text[1] == "-"
              and text[2:].isdigit() and score >= min_score)
        if ok and score > best[1]:
            best = (text, round(score, 3), text)
        elif best[0] is None and score > best[1]:
            best = (None, round(score, 3), text)
    return best

def read_label_loc(bgr, min_score=0.65):
    """全图读贴纸，返回 (text|None, score, box) —— box 是字符簇在原图里的外接框。"""
    if bgr is None or getattr(bgr, "size", 0) == 0:
        return None, -2.0, None
    mask = _white_mask(bgr)
    groups = _clusters(_glyph_candidates(mask))
    best = (None, -2.0, None)
    for grp in groups:
        if not (2 <= len(grp) <= 6):
            continue
        text, score = _read_group(mask, grp)
        if len(text) == 3 and text[0] in _LETTERS and text[1:].isdigit():
            text = text[0] + "-" + text[1:]
        ok = (len(text) == 4 and text[0] in _LETTERS and text[1] == "-"
              and text[2:].isdigit() and score >= min_score)
        if not ok:
            continue
        x0 = min(g["x"] for g in grp)
        y0 = min(g["y"] for g in grp)
        x1 = max(g["x"] + g["w"] for g in grp)
        y1 = max(g["y"] + g["h"] for g in grp)
        if score > best[1]:
            best = (text, round(score, 3), (int(x0), int(y0), int(x1 - x0), int(y1 - y0)))
    return best
