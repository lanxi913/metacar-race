# -*- coding: utf-8 -*-
"""车侧编号识别（第二版）：白字掩膜 → 字形聚类 → 合成字体多尺度模板投票。

为什么重写：现场实测（`vla_m6` 的 33 张截图）说明
  * 车侧编号是白字（V 高、S 低），在 640x480 里高度只有 9~15 px；
  * 旧实现把整簇字归一化后直接模板匹配，`5` 会被认成 `B`（得分 0.66），
    随后格式校验把结果丢掉 ⇒ 读号成功率 0。
本版针对三点改造：
  1) **逐字形**匹配（不对整簇做缩放），归一化到 24×32 后与合成模板比较；
  2) 模板用 3 种粗体字 × 高度 10~44 px，覆盖车侧编号在 2~8 m 的实际像素高度；
  3) 返回每个候选的得分与字形数，由调用方做**多帧投票**，票数不够就不报
     —— 用户要求“数量不能多报也不能少报”，宁可不报也不许报错。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
FONTS = (cv2.FONT_HERSHEY_DUPLEX, cv2.FONT_HERSHEY_TRIPLEX, cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_COMPLEX)
GLYPH_W, GLYPH_H = 24, 32
_TEMPLATES: dict[str, np.ndarray] | None = None


def _norm_glyph(mask: np.ndarray) -> np.ndarray:
    """把字形图（0/255）裁到外接框后缩放到 24×32，上下左右各留 2 px 边距。"""
    ys, xs = np.nonzero(mask)
    if len(ys) == 0:
        return np.zeros((GLYPH_H, GLYPH_W), np.float32)
    m = mask[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    inner_h, inner_w = GLYPH_H - 4, GLYPH_W - 4
    sh = inner_h / max(1, m.shape[0])
    sw = inner_w / max(1, m.shape[1])
    s = min(sh, sw)
    nh, nw = max(1, int(round(m.shape[0] * s))), max(1, int(round(m.shape[1] * s)))
    r = cv2.resize(m, (nw, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((GLYPH_H, GLYPH_W), np.float32)
    y0 = (GLYPH_H - nh) // 2
    x0 = (GLYPH_W - nw) // 2
    out[y0:y0 + nh, x0:x0 + nw] = (r > 127).astype(np.float32)
    return out


def _build_templates() -> dict[str, np.ndarray]:
    """合成模板：每个字符 × 3 字体 × 高度 10~44 px，二值化后按同样方式归一化。"""
    global _TEMPLATES
    if _TEMPLATES is not None:
        return _TEMPLATES
    tpl: dict[str, list[np.ndarray]] = {}
    for ch in CHARS + "-":
        out = []
        for font in FONTS:
            for h in (7, 8, 9, 10, 11, 12, 13, 14, 16, 18, 20, 22, 26, 30, 36, 44):
                scale = h / 20.0
                thick = max(1, int(round(h / 12.0)))
                canvas = np.zeros((h * 2, h * 3), np.uint8)
                cv2.putText(canvas, ch, (h // 4, int(h * 1.4)), font, scale, 255, thick, cv2.LINE_AA)
                m = (canvas > 110).astype(np.uint8) * 255
                if m.any():
                    out.append(_norm_glyph(m))
        tpl[ch] = out
    _TEMPLATES = {k: np.stack(v) for k, v in tpl.items() if v}
    return _TEMPLATES


def white_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, (0, 0, 150), (180, 70, 255))


def _cluster_glyphs(mask: np.ndarray, min_h: int = 6, max_h: int = 90):
    """找出“同一行、同高度”的字形簇；返回 [[(x,y,w,h,mask_crop), ...], ...]。"""
    n, _lab, stats, _cent = cv2.connectedComponentsWithStats(mask, 8)
    cand = []
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if h < min_h or h > max_h:
            continue
        if w < 2 or w > 3.0 * h:
            continue
        if a / float(w * h) < 0.22:
            continue
        cand.append((x, y, w, h, i))
    cand.sort(key=lambda t: t[0])
    clusters: list[list[tuple]] = []
    for c in cand:
        x, y, w, h, i = c
        placed = False
        for cl in clusters:
            x0, y0, w0, h0, _i = cl[-1]
            same_h = abs(h - h0) <= max(2, 0.35 * h0)
            same_row = abs((y + h / 2) - (y0 + h0 / 2)) <= max(2, 0.40 * h0)
            near = (x - (x0 + w0)) <= max(4, 0.9 * h0)
            if same_h and same_row and near:
                cl.append(c)
                placed = True
                break
        if not placed:
            clusters.append([c])
    out = []
    for cl in clusters:
        if len(cl) < 2:
            continue
        glyphs = []
        for x, y, w, h, i in cl:
            sub = np.zeros_like(mask)
            sub[y:y + h, x:x + w] = mask[y:y + h, x:x + w]
            glyphs.append((x, y, w, h, sub))
        out.append(glyphs)
    return out


DIGITS = "0123456789"
LETTERS = "ABCDEFGHJKLMNPQRSTUVWXYZ"   # 去掉 I/O（和 1/0 在 10 px 下分不开，靠位置约束已经能区分）


def _match_glyph(mask: np.ndarray, allowed: str | None = None) -> tuple[str, float]:
    tpl = _build_templates()
    g = _norm_glyph(mask)
    best_ch, best_sc = "", 0.0
    for ch, arrs in tpl.items():
        if allowed is not None and ch not in allowed:
            continue
        num = (arrs * g).sum(axis=(1, 2))
        den = np.maximum(arrs.sum(axis=(1, 2)), 1e-6)
        sc = float(np.max(num / den))
        if sc > best_sc:
            best_sc, best_ch = sc, ch
    return best_ch, best_sc


def read_labels(bgr: np.ndarray, min_score: float = 0.62, max_candidates: int = 4):
    """返回候选列表 [(text, score, n_glyph, (x,y,w,h))]，按得分降序。text 形如 `B-05`。"""
    mask = white_mask(bgr)
    if mask.sum() == 0:
        return []
    out = []
    for glyphs in _cluster_glyphs(mask):
        # 只认“字母 + 两位数字”的形态：先按宽度剔除分隔符 '-'（细长条）
        body = [g for g in glyphs if g[2] > 0.45 * g[3]]
        if len(body) not in (2, 3):
            continue
        chars, scores = [], []
        for k, (x, y, w, h, sub) in enumerate(body):
            allowed = LETTERS if k == 0 else DIGITS
            ch, sc = _match_glyph(sub, allowed)
            chars.append(ch)
            scores.append(sc)
        raw = "".join(chars)
        glyphs = body
        text = _to_label(raw)
        score = float(np.mean(scores)) if scores else 0.0
        if not text or score < min_score:
            continue
        xs = [g[0] for g in glyphs]
        ys = [g[1] for g in glyphs]
        ws = [g[2] for g in glyphs]
        hs = [g[3] for g in glyphs]
        box = (min(xs), min(ys), max(xs) - min(xs) + ws[-1], max(hs))
        out.append((text, score, len(glyphs), box))
    out.sort(key=lambda t: -t[1])
    return out[:max_candidates]


def _to_label(raw: str) -> str:
    import re

    t = "".join(c for c in raw.upper() if c.isalnum() or c == "-")
    if re.fullmatch(r"[A-Z]\d{2}", t):
        return f"{t[0]}-{t[1:]}"
    if re.fullmatch(r"[A-Z]-\d{2}", t):
        return t
    return ""


if __name__ == "__main__":
    import sys
    from pathlib import Path

    for p in sys.argv[1:]:
        img = cv2.imread(str(p))
        res = read_labels(img) if img is not None else []
        print(Path(p).name, "->", [(t, round(s, 3), n, b) for t, s, n, b in res][:3])


# ---------------- 车牌优先（现场结论：车侧大字在 8~12 m 只有 9~12 px，车牌更稳） ----------------
def blue_plate(bgr: np.ndarray):
    """找车顶/车头那块蓝底牌子，返回 (x, y, w, h) 或 None。

    现场观测：牌子是深蓝底 + 白字，HSV 大致 H∈[100,130]、S≥90、V∈[70,200]；
    面积占车框的 1/60 ~ 1/6，宽高比 1.6~3.5。
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (100, 90, 60), (130, 255, 210))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, _l, st, _c = cv2.connectedComponentsWithStats(m, 8)
    best = None
    h_img, w_img = bgr.shape[:2]
    area_img = float(h_img * w_img)
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < 60 or a > 0.25 * area_img:
            continue
        ar = w / max(1.0, float(h))
        if ar < 1.5 or ar > 4.0:
            continue
        if h < 6 or h > 0.35 * h_img:
            continue
        score = a * (1.0 - abs(ar - 2.4) / 3.0)
        if best is None or score > best[0]:
            best = (score, x, y, w, h)
    return None if best is None else best[1:]


def read_plate(bgr: np.ndarray, min_score: float = 0.60, up: int = 10):
    """先定位车牌再读：返回 [(text, score, n_glyph, (x,y,w,h))]，坐标是 bgr 坐标系。"""
    box = blue_plate(bgr)
    if box is None:
        return []
    x0, y0, w0, h0 = box
    pad_x, pad_y = int(0.15 * w0), int(0.15 * h0)
    X0, Y0 = max(0, x0 - pad_x), max(0, y0 - pad_y)
    X1, Y1 = min(bgr.shape[1], x0 + w0 + pad_x), min(bgr.shape[0], y0 + h0 + pad_y)
    roi = bgr[Y0:Y1, X0:X1]
    if roi.size == 0:
        return []
    roi = cv2.resize(roi, None, fx=up, fy=up, interpolation=cv2.INTER_CUBIC)
    g = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    g = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4)).apply(g)
    # 车牌内白字：取比局部中位数亮的连通块
    med = float(np.median(g))
    _t, mask = cv2.threshold(g, max(med + 12.0, 120.0), 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    clusters = _cluster_glyphs(mask, min_h=int(0.35 * g.shape[0]), max_h=int(0.85 * g.shape[0]))
    out = []
    for glyphs in clusters:
        parts, scores = [], []
        for k, (gx, gy, gw, gh, gm) in enumerate(glyphs):
            allowed = LETTERS if k == 0 else DIGITS
            ch, sc = _match_glyph(gm, allowed=allowed)
            parts.append(ch)
            scores.append(sc)
        if len(parts) < 3:
            continue
        parts, scores = parts[:3], scores[:3]
        text = _to_label(''.join(parts))
        if not text:
            continue
        x = glyphs[0][0] // up + X0
        y = min(gy for _gx, gy, _gw, _gh, _gm in glyphs) // up + Y0
        w = (glyphs[-1][0] + glyphs[-1][2] - glyphs[0][0]) // up
        h = max(_gh for _gx, _gy, _gw, _gh, _gm in glyphs) // up
        out.append((text, float(np.mean(scores)), len(parts), (x, y, w, h)))
    out.sort(key=lambda t: -t[1])
    return [t for t in out if t[1] >= min_score]


def read_any(bgr: np.ndarray, min_score: float = 0.60):
    """车牌优先，读不到再退回车身大字。"""
    r = read_plate(bgr, min_score=min_score)
    if r:
        return r
    return read_labels(bgr, min_score=min_score)
