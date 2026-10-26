# -*- coding: utf-8 -*-
"""把真车跑次的 run.csv 画成“俯视轨迹图 + 速度曲线”，用来肉眼回放这一圈。"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np

REGION = (6.2, 6.1, 46.2, 46.1)      # 禁停区（来自 vla_extension.regions）
E_REF = 2.6                          # 贴边距离
R_CORNER = 6.5


def ref_polyline(x0, y0, x1, y1, inset=E_REF, r=R_CORNER, ds=0.5):
    """与 vla2/stack.py 的 build_reference 同一套几何：矩形内缩 + 四角圆角。"""
    ax0, ay0, ax1, ay1 = x0 + inset, y0 + inset, x1 - inset, y1 - inset
    pts = []

    def line(p, q):
        n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / ds))
        for i in range(n):
            t = i / n
            pts.append((p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t))

    def arc(c, a0, a1):
        n = max(2, int(abs(a1 - a0) * r / ds))
        for i in range(n):
            a = a0 + (a1 - a0) * i / n
            pts.append((c[0] + r * math.cos(a), c[1] + r * math.sin(a)))

    line((ax0 + r, ay0), (ax1 - r, ay0))
    arc((ax1 - r, ay0 + r), -math.pi / 2, 0)
    line((ax1, ay0 + r), (ax1, ay1 - r))
    arc((ax1 - r, ay1 - r), 0, math.pi / 2)
    line((ax1 - r, ay1), (ax0 + r, ay1))
    arc((ax0 + r, ay1 - r), math.pi / 2, math.pi)
    line((ax0, ay1 - r), (ax0, ay0 + r))
    arc((ax0 + r, ay0 + r), math.pi, 1.5 * math.pi)
    return pts


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--size", type=int, default=760)
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv, encoding="utf-8")))
    xs = np.array([float(r["x"]) for r in rows])
    ys = np.array([float(r["y"]) for r in rows])
    vs = np.array([abs(float(r["v"])) for r in rows])
    ts = np.array([float(r["t"]) for r in rows])

    pad = 5.0
    lo_x, hi_x = min(xs.min(), REGION[0]) - pad, max(xs.max(), REGION[2]) + pad
    lo_y, hi_y = min(ys.min(), REGION[1]) - pad, max(ys.max(), REGION[3]) + pad
    span = max(hi_x - lo_x, hi_y - lo_y)
    S = args.size
    sc = (S - 60) / span

    def P(x, y):
        # 场景 y 向上，图像 y 向下
        return (int(30 + (x - lo_x) * sc), int(S - 30 - (y - lo_y) * sc))

    img = np.full((S, S, 3), 245, np.uint8)
    # 禁停区
    cv2.rectangle(img, P(REGION[0], REGION[1]), P(REGION[2], REGION[3]), (60, 60, 200), 2)
    cv2.putText(img, "NO-PARK", P(REGION[0] + 0.6, REGION[3] - 1.2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 200), 2)
    # 参考线
    ref = ref_polyline(*REGION)
    for i in range(len(ref) - 1):
        cv2.line(img, P(*ref[i]), P(*ref[i + 1]), (170, 170, 60), 2)
    # 轨迹（按速度着色）
    for i in range(len(xs) - 1):
        v = vs[i]
        col = (0, 140, 0) if v < 1.0 else (0, 90, 210) if v < 2.0 else (200, 60, 0)
        cv2.line(img, P(xs[i], ys[i]), P(xs[i + 1], ys[i + 1]), col, 3)
    # 每 5 s 打点
    for tt in range(0, int(ts[-1]) + 1, 5):
        i = int(np.argmin(np.abs(ts - tt)))
        cv2.circle(img, P(xs[i], ys[i]), 4, (0, 0, 0), -1)
        if tt % 10 == 0:
            cv2.putText(img, f"{tt}s", P(xs[i], ys[i] + 1.4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.circle(img, P(xs[0], ys[0]), 8, (255, 0, 255), 2)
    cv2.putText(img, "START", P(xs[0] + 1.0, ys[0] - 1.2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    cv2.circle(img, P(xs[-1], ys[-1]), 8, (0, 128, 255), -1)
    cv2.putText(img, "END", P(xs[-1] + 1.0, ys[-1] - 1.2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 255), 1)

    # 速度曲线（右下角）
    gw, gh = 300, 110
    ox, oy = S - gw - 20, 20
    cv2.rectangle(img, (ox, oy), (ox + gw, oy + gh), (255, 255, 255), -1)
    cv2.rectangle(img, (ox, oy), (ox + gw, oy + gh), (120, 120, 120), 1)
    for i in range(len(vs) - 1):
        x1 = ox + int(gw * i / len(vs))
        x2 = ox + int(gw * (i + 1) / len(vs))
        y1 = oy + gh - 10 - int((gh - 30) * min(vs[i], 5.0) / 5.0)
        y2 = oy + gh - 10 - int((gh - 30) * min(vs[i + 1], 5.0) / 5.0)
        cv2.line(img, (x1, y1), (x2, y2), (0, 90, 210), 2)
    cv2.putText(img, "v (m/s)  max %.1f" % vs.max(), (ox + 6, oy + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.putText(img, f"t={ts[-1]:.1f}s  frames={len(rows)}", (ox + 6, oy + gh - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    Path(args.out).write_bytes(buf)
    print("->", args.out, img.shape, f"points={len(rows)} vmax={vs.max():.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
