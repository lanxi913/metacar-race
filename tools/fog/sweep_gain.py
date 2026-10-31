# -*- coding: utf-8 -*-
"""扫转向总增益 STEER_GAIN：把 steer 输出整体乘一个系数。

现场 trace 实测：平均 |steer| 0.53~0.60，几乎贴满舵上限（0.6），转向过零 25~30 次/100m
—— 增益开太大、长期饱和，所以画龙。这里找出"降多少还能贴住弯"。
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

# 如果代码里还没有 STEER_GAIN，就动态补一个（临时用于扫描）
if not hasattr(fog_run5, "STEER_GAIN"):
    fog_run5.STEER_GAIN = 1.0


def run_once(gain):
    fog_run5.STEER_GAIN = gain
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


print("%-6s | %-16s %-16s %-16s %-16s %s" % ("增益", "直线", "直角弯", "闸口走廊", "走廊偏3m", "全过?"))
for gain in (1.0, 0.8, 0.6, 0.45, 0.35, 0.25):
    r = run_once(gain)
    st = r.get("直线", (99, 9, "FAIL"))
    co = r.get("直角弯", (99, 9, "FAIL"))
    ga = r.get("闸口走廊", (99, 9, "FAIL"))
    dr = r.get("走廊-偏3m", (99, 9, "FAIL"))
    ok = all(v[2] == "PASS" for v in r.values())
    print("%-6.2f | %6.2fm/%-2d %-6s %6.2fm/%-2d %-6s %6.2fm/%-2d %-6s %6.2fm/%-2d %-6s %s"
          % (gain, st[0], st[1], st[2], co[0], co[1], co[2],
             ga[0], ga[1], ga[2], dr[0], dr[1], dr[2], "PASS" if ok else "FAIL"))
