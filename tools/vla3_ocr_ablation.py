# -*- coding: utf-8 -*-
"""离线消融：RapidOCR 整帧读号 vs 距离（S1 阶段，绝不连仿真）。

用法（不启动场景，纯离线）：
    python -X utf8 tools\vla3_ocr_ablation.py --src ..\..\logs\vla_rec2 --step 8
    python -X utf8 tools\vla3_ocr_ablation.py --src D:\..\vla_imgss5 --step 4

输入两种来源：
  1) 录制目录（含 raw.bin + index.json）：index 每项 [t,x,y,offset,len]
  2) 图片目录（文件名形如 s5_t00120_x032_y024_cam0.png，从中解析 t/x/y）

输出：逐帧表 + 按距离分档的命中率/耗时统计 + 每档的编号投票分布。
"""
import argparse
import json
import os
import re
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from vla2 import cars as C  # noqa: E402

PAT = re.compile(r"([A-Za-z])\s*[-–—·'’.,:：]?\s*(\d{1,2})")
NAME_PAT = re.compile(r"_t(\d+)_x(-?\d+)_y(-?\d+)")


def load_raw_dir(d, cam=-1):
    """单路录制（raw.bin + meta.index）或多路录制（raw_cam{k}.bin + meta.cams[k].index）。"""
    meta = json.load(open(os.path.join(d, "index.json"), encoding="utf-8"))
    h, w, _ = meta["shape"]
    if "cams" in meta:
        key = str(cam if cam >= 0 else max(int(k) for k in meta["cams"]))
        path = os.path.join(d, "raw_cam%s.bin" % key)
        index = meta["cams"][key]["index"]
    else:
        path = os.path.join(d, "raw.bin")
        index = meta["index"]
    fh = open(path, "rb")

    def gen():
        for t, x, y, off, ln in index:
            fh.seek(off)
            yield t, x, y, np.frombuffer(fh.read(ln), np.uint8).reshape(h, w, 3).copy()

    return gen()


def load_img_dir(d):
    names = sorted(n for n in os.listdir(d) if n.lower().endswith((".png", ".jpg")))
    items = []
    for n in names:
        m = NAME_PAT.search(n)
        if not m:
            continue
        items.append((n, int(m.group(1)) / 10.0, float(m.group(2)) / 10.0, float(m.group(3)) / 10.0))
    items.sort(key=lambda r: r[1])

    def gen():
        for n, t, x, y in items:
            fr = cv2.imdecode(np.fromfile(os.path.join(d, n), dtype=np.uint8), cv2.IMREAD_COLOR)
            if fr is not None:
                yield t, x, y, fr

    return gen()


def build_ocr():
    from rapidocr_onnxruntime import RapidOCR
    try:
        return RapidOCR(intra_op_num_threads=1)
    except TypeError:
        return RapidOCR()


def ocr_frame(engine, bgr):
    """整帧送 OCR，返回 [(text, score, (u1,v1,u2,v2))]。"""
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    t0 = time.perf_counter()
    out = engine(rgb)
    dt = time.perf_counter() - t0
    res = out[0] if isinstance(out, tuple) else out
    hits = []
    for item in (res or []):
        try:
            pts, text, score = item[0], item[1], float(item[2])
        except Exception:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        hits.append((str(text), score, (min(xs), min(ys), max(xs), max(ys))))
    return hits, dt


def build_yolo(path):
    from ultralytics import YOLO
    m = YOLO(path)
    return m


def yolo_boxes(model, bgr, conf=0.30):
    r = model.predict(bgr, imgsz=384, conf=conf, verbose=False)[0]
    out = []
    names = r.names
    for b in r.boxes:
        cls = int(b.cls[0])
        if names.get(cls, "car") != "car":
            continue
        u1, v1, u2, v2 = [float(v) for v in b.xyxy[0].tolist()]
        out.append((u1, v1, u2, v2, float(b.conf[0])))
    return out


def bucket_of(d):
    if d < 4.0:
        return "<4m"
    if d < 6.0:
        return "4-6m"
    if d < 8.0:
        return "6-8m"
    if d < 12.0:
        return "8-12m"
    return ">12m"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", default="")
    ap.add_argument("--step", type=int, default=8)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--weights", default=r"D:\vla_runs\vla2_car\weights\best.pt")
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--no-yolo", action="store_true")
    ap.add_argument("--cam", type=int, default=-1, help="多路录制时选哪一路")
    args = ap.parse_args()

    src = args.src
    if os.path.isfile(os.path.join(src, "raw.bin")) or any(
            n.startswith("raw_cam") for n in os.listdir(src)):
        gen = load_raw_dir(src, args.cam)
    else:
        gen = load_img_dir(src)

    engine = build_ocr()
    model = None if args.no_yolo else build_yolo(args.weights)

    rows = []
    n = 0
    for t, x, y, frame in gen:
        n += 1
        if (n - 1) % max(1, args.step):
            continue
        if args.limit and len(rows) >= args.limit:
            break
        h, w = frame.shape[:2]
        boxes = [] if model is None else yolo_boxes(model, frame)
        hits, dt = ocr_frame(engine, frame)
        cars_d = []
        for (u1, v1, u2, v2, cf) in boxes:
            bw, bh = int(u2 - u1), int(v2 - v1)
            d = C.distance_of(bw, bh, int(v2), w, h)
            cars_d.append(d)
        found = []
        for text, score, box in hits:
            m = PAT.search(text.replace(" ", ""))
            if not m:
                continue
            wid = "%s-%02d" % (m.group(1).upper(), int(m.group(2)))
            cu = (box[0] + box[2]) / 2.0
            cv_ = (box[1] + box[3]) / 2.0
            inside = [i for i, (u1, v1, u2, v2, cf) in enumerate(boxes)
                      if u1 <= cu <= u2 and v1 <= cv_ <= v2]
            d_text = cars_d[inside[0]] if inside else float("nan")
            found.append((wid, score, d_text, int(box[3] - box[1])))
        rows.append({"t": t, "x": x, "y": y, "cars": len(boxes),
                     "car_d": [round(v, 2) for v in cars_d],
                     "ocr_texts": len(hits), "found": found, "ocr_ms": round(dt * 1000, 1)})

    # ---------- 统计 ----------
    ms = [r["ocr_ms"] for r in rows]
    buckets = {}
    votes = {}
    for r in rows:
        for wid, score, d, gh in r["found"]:
            b = bucket_of(d) if d == d else "no-box"
            buckets.setdefault(b, {"frames": 0, "ids": 0, "sum_score": 0.0, "best": 0.0})
            e = buckets[b]
            e["ids"] += 1
            e["sum_score"] += score
            e["best"] = max(e["best"], score)
            votes.setdefault(b, {}).setdefault(wid, 0)
            votes[b][wid] += 1
    for r in rows:
        seen = set()
        for wid, score, d, gh in r["found"]:
            b = bucket_of(d) if d == d else "no-box"
            if b not in seen:
                buckets[b]["frames"] += 1
                seen.add(b)

    print("\n=== 帧级 ===")
    print("frames=%d  ocr_ms avg=%.0f p50=%.0f max=%.0f" % (
        len(rows), np.mean(ms), np.percentile(ms, 50), np.max(ms)))
    print("\n=== 按距离分档（距 ego 的估计距离） ===")
    print("%-8s %6s %6s %8s %8s  %s" % ("bucket", "ids", "frames", "avgScore", "best", "top ids"))
    order = ["<4m", "4-6m", "6-8m", "8-12m", ">12m", "no-box"]
    for b in order:
        if b not in buckets:
            continue
        e = buckets[b]
        top = sorted(votes[b].items(), key=lambda kv: -kv[1])[:6]
        print("%-8s %6d %6d %8.3f %8.3f  %s" % (
            b, e["ids"], e["frames"], e["sum_score"] / max(1, e["ids"]), e["best"],
            ", ".join("%s×%d" % (k, v) for k, v in top)))

    print("\n=== 逐帧（前 40 行） ===")
    for r in rows[:40]:
        print("t=%6.1f (%5.1f,%5.1f) cars=%d d=%s ocr=%d -> %s (%.0fms)" % (
            r["t"], r["x"], r["y"], r["cars"], r["car_d"],
            r["ocr_texts"], [(f[0], round(f[1], 2), (round(f[2], 1) if f[2] == f[2] else None)) for f in r["found"]],
            r["ocr_ms"]))

    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump({"rows": rows, "buckets": {k: v for k, v in buckets.items()},
                       "votes": {k: v for k, v in votes.items()}}, fh, ensure_ascii=False, indent=1)
        print("-> %s" % args.out)


if __name__ == "__main__":
    main()
