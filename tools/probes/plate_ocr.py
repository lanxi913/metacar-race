"""车顶编号牌的裁剪预处理 + Windows OCR + 编号解析。

现场实测（2026-10-26）
----------------------
* 直接把大图丢给 Windows OCR 基本读不出来；**紧裁剪 + Otsu + 放大到短边 ~90 px + 白边**
  才能稳定出字（同一块牌：gray 配方出 `004`，otsu 配方出 `C'04`）。
* 编号形如 `C-04`、`C-01`（OCR 常把它念成 `C'04` / `C04` / `604`），解析时按字母+两位数字归一。

对外接口
--------
    crop_variants(bgr, box) -> dict[str, ndarray]
    ocr_dir(ps1, dir, out_tsv) -> dict[str, str]
    parse_codes(text) -> list[str]
    read_plate(bgr, box, ps1, workdir) -> tuple[str, str]   # (code, raw_text)
"""

from __future__ import annotations

import re
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np

CODE_RE = re.compile(r"([A-Za-z])\s*[-–—·'’.,:：]?\s*(\d{1,2})")
DIGITS_RE = re.compile(r"(\d{2})")

HERE = Path(__file__).resolve().parent
PS1 = HERE / "ocr_batch.ps1"


def _norm_gray(gray: np.ndarray, target_h: int = 90) -> np.ndarray:
    h, w = gray.shape[:2]
    scale = max(1.0, target_h / max(h, 1))
    scale = min(scale, 600.0 / max(w, 1))
    if abs(scale - 1.0) > 0.03:
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    return gray


def crop_variants(bgr: np.ndarray, box: tuple[int, int, int, int], pad: int = 2,
                  target_h: int = 90) -> dict[str, np.ndarray]:
    """box = (u, v, w, h)。返回 {"otsu": ..., "gray": ...}，都已加白边。"""
    u, v, w, h = box
    x0, y0 = max(0, u - pad), max(0, v - pad)
    x1, y1 = min(bgr.shape[1], u + w + pad), min(bgr.shape[0], v + h + pad)
    roi = bgr[y0:y1, x0:x1]
    if roi.size == 0:
        return {}
    gray = _norm_gray(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), target_h)
    _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    out = {"otsu": otsu, "gray": gray}
    for k, img in list(out.items()):
        out[k] = cv2.copyMakeBorder(img, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=255)
    return out


def ocr_dir(dirpath: str | Path, out_tsv: str | Path, ps1: str | Path = PS1,
            lang: str = "zh-Hans-CN") -> dict[str, str]:
    """调一次 PowerShell，把目录里所有 png 都 OCR 掉，返回 {文件名: 文本}。"""
    dirpath, out_tsv = Path(dirpath), Path(out_tsv)
    cmd = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
           "-File", str(ps1), "-Dir", str(dirpath), "-Out", str(out_tsv),
           "-Lang", lang, "-Filter", "*.png"]
    subprocess.run(cmd, capture_output=True, timeout=180)
    res: dict[str, str] = {}
    if out_tsv.exists():
        for line in out_tsv.read_text(encoding="utf-8").splitlines():
            if "\t" not in line:
                continue
            name, _, text = line.partition("\t")
            res[name] = text.strip()
    return res


def parse_codes(text: str) -> list[str]:
    """从 OCR 文本里解析 `X-01` 形式的编号；容忍 OCR 把 - 念成 ' · . , 等。"""
    if not text:
        return []
    out = []
    for m in CODE_RE.finditer(text):
        out.append(f"{m.group(1).upper()}-{int(m.group(2)):02d}")
    return out


def read_plate(bgr: np.ndarray, box: tuple[int, int, int, int], ps1: str | Path = PS1,
               workdir: str | Path | None = None) -> tuple[str, str]:
    """单块牌：裁剪 + 多配方 OCR，返回 (归一化编号或空串, 原始文本)。"""
    variants = crop_variants(bgr, box)
    if not variants:
        return "", ""
    tmp = Path(workdir) if workdir else Path(tempfile.mkdtemp(prefix="plate_"))
    tmp.mkdir(parents=True, exist_ok=True)
    for k, img in variants.items():
        cv2.imencode(".png", img)[1].tofile(str(tmp / f"{k}.png"))
    texts = ocr_dir(tmp, tmp / "out.tsv", ps1)
    best = ""
    raw = ""
    for k in ("otsu", "gray"):
        t = texts.get(f"{k}.png", "")
        if t and not raw:
            raw = t
        for code in parse_codes(t):
            best = best or code
    return best, raw


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="单图调试：给图 + --box")
    ap.add_argument("--box", default="", help="u,v,w,h")
    args = ap.parse_args()
    if args.image and args.box:
        img = cv2.imdecode(np.fromfile(args.image, dtype=np.uint8), cv2.IMREAD_COLOR)
        box = tuple(int(x) for x in args.box.split(","))
        code, raw = read_plate(img, box)  # type: ignore[arg-type]
        print("编号:", code, "原始:", raw)
        return 0
    print("用法: --image <png> --box u,v,w,h")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
