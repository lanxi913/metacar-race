# -*- coding: utf-8 -*-
"""用离线自测扫描转向增益，找"走直线最直"的一组参数。

自测是运动学仿真，跑完 5 项只要几秒，比进场景试快两个数量级。
目标：直线 / 走廊的"最大横向误差"最小，同时直角弯别退化。
"""
import importlib
import io
import os
import re
import sys

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
sys.path.insert(0, FOG)
sys.path.insert(0, r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800")

import fog_run5  # noqa: E402

CANDIDATES = [
    # (LAT_DAMP, CURV_FF, LOOKAHEAD_B, LOOKAHEAD_A)
    (0.10, 0.7, 0.45, 1.0),   # 现状
    (0.06, 0.7, 0.45, 1.0),
    (0.04, 0.7, 0.45, 1.0),
    (0.10, 0.5, 0.45, 1.0),
    (0.10, 0.9, 0.45, 1.0),
    (0.10, 0.7, 0.60, 1.0),
    (0.10, 0.7, 0.30, 1.0),
    (0.06, 0.6, 0.50, 1.0),
    (0.04, 0.5, 0.55, 1.0),
    (0.08, 0.6, 0.50, 1.2),
]

buf = io.StringIO()
real_stdout = sys.stdout


def run_once(lat, curv, lkb, lka):
    fog_run5.LAT_DAMP = lat
    fog_run5.CURV_FF = curv
    fog_run5.LOOKAHEAD_B = lkb
    fog_run5.LOOKAHEAD_A = lka
    buf.truncate(0)
    buf.seek(0)
    sys.stdout = buf
    try:
        fog_run5._selftest()
    finally:
        sys.stdout = real_stdout
    txt = buf.getvalue()
    out = {}
    for line in txt.splitlines():
        m = re.search(r"\[(PASS|FAIL)\]\s+(\S+).*最大横向=([\d.]+)m.*纠偏次数=(\d+)", line)
        if m:
            out[m.group(2)] = (float(m.group(3)), int(m.group(4)), m.group(1))
    return out


rows = []
for lat, curv, lkb, lka in CANDIDATES:
    res = run_once(lat, curv, lkb, lka)
    straight = res.get("直线", (99.0, 99, "FAIL"))
    corner = res.get("直角弯", (99.0, 99, "FAIL"))
    gate = res.get("闸口走廊", (99.0, 99, "FAIL"))
    drift = res.get("走廊-偏3m", (99.0, 99, "FAIL"))
    ok = all(v[2] == "PASS" for v in res.values())
    score = straight[0] + drift[0] + 0.5 * gate[0]
    rows.append((score, ok, lat, curv, lkb, lka, straight, corner, gate, drift))

rows.sort(key=lambda r: (not r[1], r[0]))
print("%-5s %-6s %-6s %-6s %-6s | %-22s %-22s %s" % ("ok", "LAT", "CURV", "LKB", "LKA", "直线(横向/纠偏)", "直角弯", "评分"))
for score, ok, lat, curv, lkb, lka, st, co, ga, dr in rows:
    print("%-5s %-6.2f %-6.2f %-6.2f %-6.1f | %6.2fm/%d %-10s %6.2fm/%d %8.2f"
          % ("PASS" if ok else "FAIL", lat, curv, lkb, lka,
             st[0], st[1], "", co[0], co[1], score))
