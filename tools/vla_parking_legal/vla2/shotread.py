# -*- coding: utf-8 -*-
"""VLA 任务二：截图 + 读号（识别模块，唯一允许改动的部分）。

对外只暴露 `ShotReader`：
    reader = ShotReader(shot_dir=...)
    reader.observe(bgr, x, y, yaw, tag="t00010")   # 每帧一次（可在线程里跑）
    reader.stats()                                  # 只读快照
    reader.report()                                 # 汇总（内部会收尾）
    reader.submit_ids(inside)                       # 最终提交清单（去重排序）
    reader.report_full()                            # 逐车明细

去重策略（现场结论）：旧办法"每帧反投影世界点、2.5 m 内算同一辆"在 6~12 m 上
误差 1~2 m，把一辆车拆成好几簇（实测 21 簇 / 真值 14 辆）。现改为
"图像域帧间跟踪 → 轨迹结束后世界里 1.0 m 内才合并"，见 `vla2/cars.py`。
"""

from __future__ import annotations

import os
import re
from pathlib import Path

import cv2
import numpy as np

from .cars import CarTracker
from .ocr2 import read_any

# ---------------- 现场参数 ----------------
# 优先用环境变量 / 队友的微调权重；本机没有 best.pt 时退回通用 COCO yolov8n。
# 通用权重对红色轿车也能分框，比颜色连通域在“三辆车挨在一起”时可靠。
_WEIGHTS_CANDIDATES = [
    os.environ.get("VLA_YOLO_WEIGHTS", ""),
    r"D:\vla_runs\vla2_car\weights\best.pt",
    r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\models\yolov8n.pt",
]
WEIGHTS = next((p for p in _WEIGHTS_CANDIDATES if p and Path(p).exists()),
               _WEIGHTS_CANDIDATES[1])
CONF_MIN = 0.25                                     # YOLO 置信度下限
PAD_RATIO = 0.08                                    # 截图外扩比例
IMGSZ = 384


def imread_uni(path):
    return cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)


def imwrite_uni(path, img, quality=92):
    ext = str(path).rsplit(".", 1)[-1].lower() if "." in str(path) else "jpg"
    if ext in ("jpg", "jpeg"):
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    else:
        ok, buf = cv2.imencode(".png", img)
    if ok:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        buf.tofile(str(path))
    return ok


def norm_label(raw: str) -> str:
    """把 OCR 原始串规范成 `B-05` 形式；不合格式返回空串。"""
    if not raw:
        return ""
    s = re.sub(r"[^0-9A-Za-z]", "", str(raw)).upper()
    m = re.fullmatch(r"([A-Z])(\d{2})", s)
    if m:
        return f"{m.group(1)}-{m.group(2)}"
    m = re.fullmatch(r"([A-Z])(\d)", s)
    if m:
        return f"{m.group(1)}-0{m.group(2)}"
    return ""


class YoloCars:
    """车辆/车牌检测器（微调后的 yolov8n）。"""

    def __init__(self, weights: str = WEIGHTS, conf: float = CONF_MIN,
                 imgsz: int = IMGSZ, iou: float = 0.60) -> None:
        from ultralytics import YOLO

        self.model = YOLO(weights)
        self.conf = conf
        self.imgsz = imgsz
        self.iou = iou
        self.names = getattr(self.model, "names", {}) or {}

    def detect(self, bgr: np.ndarray):
        """返回 [(x, y, w, h, conf)]，只保留 car 类（车牌只做辅助，不单独算一辆车）。"""
        if bgr is None or bgr.size == 0:
            return []
        res = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf,
                                 iou=self.iou, verbose=False)
        out = []
        for r in res:
            boxes = getattr(r, "boxes", None)
            if boxes is None:
                continue
            for b in boxes:
                cls_id = int(b.cls[0]) if getattr(b, "cls", None) is not None else 0
                name = str(self.names.get(cls_id, cls_id)).lower()
                # COCO 里这三类都可能是轿车/面包车；只收 car 会漏掉被识成 truck 的车。
                if not any(k in name for k in ("car", "truck", "bus")):
                    continue
                x0, y0, x1, y1 = [float(v) for v in b.xyxy[0].tolist()]
                cf = float(b.conf[0]) if getattr(b, "conf", None) is not None else 1.0
                x, y = int(max(0, x0)), int(max(0, y0))
                w, h = int(max(2, x1 - x0)), int(max(2, y1 - y0))
                out.append((x, y, w, h, cf))
        return out


class ShotReader:
    """对外接口不变（observe/report/report_full/submit_ids），内部是"跟踪+合并"。"""

    def __init__(self, detector=None, shot_dir=None) -> None:
        self.det = detector or YoloCars()
        self.trk = CarTracker()
        self.shot_dir = Path(shot_dir) if shot_dir else None
        if self.shot_dir:
            self.shot_dir.mkdir(parents=True, exist_ok=True)
        self._t = 0.0

    # ---------- 每帧 ----------
    def observe(self, img, x, y, yaw, tag="") -> int:
        if img is None:
            return 0
        h, w = img.shape[:2]
        boxes = self.det.detect(img)
        self._t += 1.0 / 30.0

        sharp = []
        for (bx, by, bw, bh, _c) in boxes:
            if bh < 24:
                continue
            crop = img[max(0, by):min(h, by + bh), max(0, bx):min(w, bx + bw)]
            if crop.size:
                g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                sharp.append(float(cv2.Laplacian(g, cv2.CV_64F).var()))
        sharp_med = float(np.median(sharp)) if sharp else 0.0

        todo = self.trk.observe(boxes, self._t, x, y, yaw, w, h, sharp_med)
        for tr, box, z in todo:
            bx, by, bw, bh = box
            pad_x, pad_y = int(bw * PAD_RATIO), int(bh * PAD_RATIO)
            x0, y0 = max(0, bx - pad_x), max(0, by - pad_y)
            x1, y1 = min(w, bx + bw + pad_x), min(h, by + bh + pad_y)
            crop = img[y0:y1, x0:x1]
            if crop.size == 0:
                continue
            g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            if float(cv2.Laplacian(g, cv2.CV_64F).var()) < max(20.0, 0.5 * sharp_med):
                continue                                  # 清晰度门：糊的不截
            if self.shot_dir:
                p = self.shot_dir / f"{tag or 'f'}_{int(z * 10):03d}m_{bw}x{bh}.jpg"
                imwrite_uni(p, crop.copy())
            for lab, score, _n, gbox in read_any(crop, min_score=0.62):
                self.trk.add_read(tr, norm_label(lab), float(score), int(gbox[3]))
        return len(boxes)

    # ---------- 汇总 ----------
    def stats(self):
        """只读快照：主线程写日志用，绝不做合并（finish 会改状态，只能在识别线程里调）。"""
        trk = self.trk
        alive = sum(1 for t in trk.tracks if t.alive)
        return {"cars": len(trk.cars) + alive, "shots": trk.n_shots,
                "reads": trk.n_reads, "frames": trk.n_frames, "boxes": trk.n_boxes}

    def report(self) -> dict:
        r = self.trk.report()
        return {"cars": r["cars"], "shots": r["shots"], "read_ok": r["labeled"],
                "read_rate": r["read_rate"], "labels": r["labels"],
                "frames": r["frames"], "reads": r["reads"]}

    def report_full(self):
        return self.trk.report_full()

    def submit_ids(self, inside):
        return self.trk.submit_ids(inside)
