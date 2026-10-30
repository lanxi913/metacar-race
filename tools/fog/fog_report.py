# -*- coding: utf-8 -*-
"""把一局的 run.log 压成一屏结论（迭代时只看这个）。

用法：python fog_report.py <run_dir>
"""
import os
import re
import sys
from collections import Counter

# Windows 控制台默认 GBK，中文/符号会直接抛 UnicodeEncodeError —— 这里强制 UTF-8
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:                                          # noqa: BLE001
    pass

PAT = [
    ("submit", re.compile(r"\[\s*([\d.]+)\]\s+(?:submit|弧线提交) (Signal_[ABC]) (\S+) d=([\d.]+).*?-> (\w+)")),
    ("scan", re.compile(r"\[\s*([\d.]+)\].*scan\(1\) -> .*granted': (\w+)")),
    ("granted", re.compile(r"\[\s*([\d.]+)\]\s+\[cross\] (\w+) GRANTED")),
    ("released", re.compile(r"\[\s*([\d.]+)\]\s+\[rel\] (\w+) @\(([-\d.]+),([-\d.]+)\).*-> (True|False)")),
    ("cross_ok", re.compile(r"\[\s*([\d.]+)\]\s+\[cross\] (\w+) 通行 (成功|失败)")),
    ("deliver", re.compile(r"\[\s*([\d.]+)\]\s+\[deliver\] (\w+) deliver (\w+)\((\d)\) granted=(\w+) msg=(.*)")),
    ("level", re.compile(r"\[\s*([\d.]+)\]\s+\[deliver\] Berth_(\w) obs .*level=lv(\d)")),
    ("viol", re.compile(r"\[\s*([\d.]+)\]\s+\[violation\] #(\d+) \[(\w+)\] (.*)")),
    ("pre", re.compile(r"\[\s*([\d.]+)\]\s+\[pre\]")),
]


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    path = os.path.join(d, "run.log")
    if not os.path.exists(path):
        print("没有 run.log：%s" % path)
        return 1
    lines = open(path, encoding="utf-8", errors="replace").read().splitlines()

    sigs, scans, grants, rels, cors, dels, levels = {}, [], {}, {}, [], [], []
    viols = []
    for ln in lines:
        m = PAT[0][1].search(ln)
        if m:
            sigs[m.group(2)] = (float(m.group(1)), m.group(3), m.group(5))
            continue
        m = PAT[1][1].search(ln)
        if m:
            scans.append((float(m.group(1)), m.group(2)))
            continue
        m = PAT[2][1].search(ln)
        if m:
            grants.setdefault(m.group(2), float(m.group(1)))
            continue
        m = PAT[3][1].search(ln)
        if m:
            rels[m.group(2)] = (float(m.group(1)), m.group(5), float(m.group(3)), float(m.group(4)))
            continue
        m = PAT[4][1].search(ln)
        if m:
            cors.append((m.group(2), m.group(3), float(m.group(1))))
            continue
        m = PAT[5][1].search(ln)
        if m:
            dels.append((m.group(2), m.group(3), m.group(5), m.group(6)))
            continue
        m = PAT[6][1].search(ln)
        if m:
            levels.append((m.group(2), int(m.group(3)), float(m.group(1))))
            continue
        m = PAT[7][1].search(ln)
        if m:
            viols.append((float(m.group(1)), m.group(3), m.group(4)))
            continue

    print("=== 一局结论 ===")
    print("信号：", end="")
    for k in ("Signal_A", "Signal_B", "Signal_C"):
        if k in sigs:
            t, key, ok = sigs[k]
            print("%s %5.1fs(%s,%s) " % (k[-1], t, key, ok), end="")
        else:
            print("%s --- " % k[-1], end="")
    print("  3/3=%s" % (len(sigs) == 3))
    if scans:
        print("扫描：%.1fs granted=%s" % scans[0])
    if grants:
        print("授权：", grants)
    if rels:
        print("出口：", {k: (v[0], v[1]) for k, v in rels.items()})
    if cors:
        print("通行：", cors)
    if levels:
        print("泊位等级：", levels[-3:])
    if dels:
        print("交付：", dels[-3:])
    if viols:
        c = Counter("%s | %s" % (v[1], v[2][:52]) for v in viols)
        print("违规 %d 条：" % len(viols))
        for k, n in c.most_common(6):
            print("   %2d× %s" % (n, k))
    else:
        print("违规：0")
    print("末行：%s" % (lines[-1][:100] if lines else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
