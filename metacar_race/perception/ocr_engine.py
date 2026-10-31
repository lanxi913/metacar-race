# -*- coding: utf-8 -*-
"""OCR 引擎封装：优先用现成库，没有就退回 Windows 自带 OCR。

本机实测（2026-10-25）：venv 里只有 cv2 + numpy，没有 tesseract/paddle/rapidocr，
所以默认走 `tools/win_ocr.ps1`（Windows.Media.Ocr，零安装）。

对外只有一个函数：
    read_text(bgr, min_conf=None) -> str     # 多行用 \\n 连接
诊断用：
    backends() -> dict                        # 各后端是否可用
"""
from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parents[2]
WIN_OCR_PS1 = REPO / "tools" / "win_ocr.ps1"

# Windows PowerShell 5.1 对 WinRT 支持最稳，优先用它而不是 pwsh 7
_POWERSHELL = shutil.which("powershell.exe") or shutil.which("powershell") or shutil.which("pwsh")

_rapid = None
_tess = None


def _try_rapid():
    global _rapid
    if _rapid is None:
        try:
            from rapidocr_onnxruntime import RapidOCR  # type: ignore
            _rapid = RapidOCR()
        except Exception:
            _rapid = False
    return _rapid


def _try_tess():
    global _tess
    if _tess is None:
        try:
            import pytesseract  # type: ignore
            _tess = pytesseract
        except Exception:
            _tess = False
    return _tess


def backends() -> dict:
    return {
        "windows_ocr": bool(_POWERSHELL) and WIN_OCR_PS1.exists(),
        "rapidocr": bool(_try_rapid()),
        "tesseract": bool(_try_tess()),
    }


def _upscale(bgr, scale: float = 2.0):
    """小字放大再 OCR，命中率明显更高。"""
    if scale == 1.0:
        return bgr
    h, w = bgr.shape[:2]
    return cv2.resize(bgr, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_CUBIC)


def _ocr_windows(bgr, lang: str = "zh-Hans-CN") -> str:
    if not (_POWERSHELL and WIN_OCR_PS1.exists()):
        return ""
    with tempfile.TemporaryDirectory(prefix="vlaocr_") as td:
        img = Path(td) / "in.png"
        out = Path(td) / "out.txt"
        cv2.imwrite(str(img), bgr)
        cmd = [
            _POWERSHELL, "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", str(WIN_OCR_PS1),
            "-Path", str(img),
            "-Out", str(out),
        ]
        if lang:
            cmd += ["-Lang", lang]
        try:
            subprocess.run(cmd, capture_output=True, timeout=25)
        except Exception:
            return ""
        if not out.exists():
            return ""
        return out.read_text(encoding="utf-8-sig", errors="ignore").strip()


def _ocr_rapid(bgr) -> str:
    eng = _try_rapid()
    if not eng:
        return ""
    try:
        res, _ = eng(bgr)
    except Exception:
        return ""
    if not res:
        return ""
    # res: [[box, text, conf], ...]，按 y 排一下更像原文
    items = sorted(res, key=lambda r: (min(p[1] for p in r[0]), min(p[0] for p in r[0])))
    return "\n".join(str(r[1]) for r in items).strip()


def _ocr_tess(bgr) -> str:
    tess = _try_tess()
    if not tess:
        return ""
    try:
        return (tess.image_to_string(bgr, lang="chi_sim") or "").strip()
    except Exception:
        return ""


_CJK = r"\u4e00-\u9fff"

def clean(text: str) -> str:
    """清洗 Windows OCR 的典型噪声：汉字间空格、全角/半角混用、孤立标点。"""
    import re
    s = (text or "").replace("\r", "").strip()
    if not s:
        return ""
    # 全角 → 半角（数字/字母/常见标点）
    table = {"．": ".", "，": ",", "：": ":", "；": ";", "（": "(", "）": ")",
             "／": "/", "－": "-", "％": "%", "　": ""}
    for a, b in table.items():
        s = s.replace(a, b)
    # 汉字之间的空格去掉（Windows OCR 常按字分隔）
    prev = None
    while prev != s:
        prev = s
        s = re.sub(r"(?<=[%s])\s+(?=[%s])" % (_CJK, _CJK), "", s)
    # 行首行尾的孤立标点
    s = re.sub(r"(?m)^[\s,.;:、]+|[\s,.;:、]+$", "", s)
    s = re.sub(r"[ \t]{2,}", " ", s)
    return "\n".join(line for line in s.split("\n") if line.strip()).strip()


def _score(text: str) -> float:
    """候选文本打分：汉字越多越好，标点噪声越少越好。"""
    if not text:
        return -1e9
    cjk = 0
    noise = 0
    for ch in text:
        if "\u4e00" <= ch <= "\u9fff":
            cjk += 1
        elif ch.isalnum() or ch in ",.:;%),(/-":
            continue
        elif ch in " \n\t":
            continue
        else:
            noise += 1
    return cjk * 2.0 - noise * 3.0 - text.count(" ") * 0.2


def find_yellow_sign(bgr, pad: int = 10):
    """把任务牌（亮黄色块）从画面里框出来。牌子在远处只占几十像素，
    整幅图缩放 OCR 基本认不准；先框出来再放大，准确率高得多。"""
    try:
        import cv2
        import numpy as np
    except Exception:
        return None
    if bgr is None or getattr(bgr, "size", 0) == 0:
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([15, 70, 110], dtype=np.uint8)
    upper = np.array([42, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 1500:
        return None
    x, y, w, h = cv2.boundingRect(c)
    h_img, w_img = bgr.shape[:2]
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1, y1 = min(w_img, x + w + pad), min(h_img, y + h + pad)
    return bgr[y0:y1, x0:x1]


def read_sign(bgr, lang: str = "zh-Hans-CN") -> str:
    """优先对"框出来的牌子"做多尺度 OCR，失败再退回整幅图。"""
    best, best_s = "", -1e18
    crop = find_yellow_sign(bgr)
    # 只要裁到牌子就只认这块（每次 OCR 约 0.6~1.5 s，多调几次会把 100 s 预算吃掉）
    scales = (3.0, 2.0) if crop is not None else (2.5, 3.5)
    img = crop if crop is not None else bgr
    for sc in scales:
        raw = read_text(img, lang=lang, scale=sc)
        txt = clean(raw)
        s = _score(txt)
        if s > best_s:
            best, best_s = txt, s
        if best_s > 40:
            break
    return best

def read_text_best(bgr, lang: str = "zh-Hans-CN", scales=(1.0, 2.0, 3.0)) -> str:
    """多尺度 OCR + 清洗 + 选优。任务牌字号未知，多试几个尺度更稳。"""
    best, best_s = "", -1e18
    for sc in scales:
        raw = read_text(bgr, lang=lang, scale=sc)
        txt = clean(raw)
        s = _score(txt)
        if s > best_s:
            best, best_s = txt, s
    return best

def read_text(bgr, lang: str = "zh-Hans-CN", scale: float = 2.0) -> str:
    """对一张 BGR 图做 OCR；所有后端都失败时返回空串。"""
    if bgr is None or getattr(bgr, "size", 0) == 0:
        return ""
    img = _upscale(bgr, scale)
    for fn in (_ocr_rapid, _ocr_tess, lambda im: _ocr_windows(im, lang)):
        try:
            txt = fn(img)
        except Exception:
            txt = ""
        if txt:
            return txt
    return ""
