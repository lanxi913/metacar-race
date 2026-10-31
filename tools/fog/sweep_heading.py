# -*- coding: utf-8 -*-
"""扫航向估计的弦长（COLD_START_M），找"直线最稳、弯道不钝"的取值。

航向是 `atan2(位移弦)` 每走 COLD_START_M 米重算一次。弦太短 → 噪声大、车画龙；
太长 → 弯道反应迟钝。离线自测几秒就能扫完。
"""
import io
import re
import sys

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
sys.path.insert(0, FOG)
sys.path.insert(0, r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800")

import fog_run5  # noqa: E402

buf = io.StringIO()
real = sys.stdout


def run_once(m):
    fog_run5.COLD_START_M = m
    buf.truncate(0)
    buf.seek(0)
    sys.stdout = buf
    try:
        fog_run5._selftest()
    finally:
        sys.stdout = real
    out = {}
    for line in buf.getvalue().splitlines():
        g = re.search(r"\[(PASS|FAIL)\]\s+(\S+).*最大横向=([\d.]+)m.*纠偏次数=(\d+)", line)
        if g:
            out[g.group(2)] = (float(g.group(3)), int(g.group(4)), g.group(1))
    return out


print("%-6s | %-16s %-16s %-16s %-16s %s" % ("弦长", "直线", "直角弯", "闸口走廊", "走廊偏3m", "全过?"))
for m in (1.0, 1.5, 2.0, 2.5, 3.0, 4.0):
    r = run_once(m)
    st = r.get("直线", (99, 9, "FAIL"))
    co = r.get("直角弯", (99, 9, "FAIL"))
    ga = r.get("闸口走廊", (99, 9, "FAIL"))
    dr = r.get("走廊-偏3m", (99, 9, "FAIL"))
    ok = all(v[2] == "PASS" for v in r.values())
    print("%-6.1f | %6.2fm/%-2d %-6s %6.2fm/%-2d %-6s %6.2fm/%-2d %-6s %6.2fm/%-2d %-6s %s"
          % (m, st[0], st[1], st[2], co[0], co[1], co[2],
             ga[0], ga[1], ga[2], dr[0], dr[1], dr[2], "PASS" if ok else "FAIL"))
