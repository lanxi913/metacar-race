# -*- coding: utf-8 -*-
"""解析场景自带的 mapd 路网，输出统计与路网图（供规划用）。"""
from __future__ import annotations

import json
import math
import os
from collections import defaultdict

MAPD = (r"C:\Users\lanxi\AppData\Roaming\com.i-mylab.metapower-desktop"
        r"\car\scenes\PORT-FOG-01\SmartCar_Data\StreamingAssets\mapfile\mapd")
OUT = r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\docs\portfog_map.json"

raw = json.load(open(MAPD, encoding="utf-8"))
print("lanes:", len(raw))

xs, ys = [], []
types = defaultdict(int)
for ln in raw:
    b, e = ln.get("beginPos") or {}, ln.get("endPos") or {}
    xs += [b.get("x", 0), e.get("x", 0)]
    ys += [b.get("y", 0), e.get("y", 0)]
    types[ln.get("drivingType")] += 1
print(f"范围: x {min(xs):.1f}..{max(xs):.1f}  y {min(ys):.1f}..{max(ys):.1f}")
print("drivingType 分布:", dict(types))

# 建节点图（按 0.5 m 量化合并重合点）
def key(p):
    return (round(p.get("x", 0) * 2) / 2, round(p.get("y", 0) * 2) / 2)


nodes = {}
edges = []
for ln in raw:
    b, e = ln.get("beginPos") or {}, ln.get("endPos") or {}
    kb, ke = key(b), key(e)
    nodes.setdefault(kb, len(nodes))
    nodes.setdefault(ke, len(nodes))
    L = math.hypot(ke[0] - kb[0], ke[1] - kb[1])
    edges.append({"id": ln.get("id"), "a": kb, "b": ke, "len": L,
                  "type": ln.get("drivingType"), "pred": ln.get("predecessor") or []})

print(f"节点: {len(nodes)}  边: {len(edges)}  总长: {sum(e['len'] for e in edges):.0f} m")

json.dump({"nodes": [[k[0], k[1]] for k in nodes],
           "edges": edges}, open(OUT, "w", encoding="utf-8"), ensure_ascii=False)

# 打印几条样例
print("\n样例车道:")
for e in edges[:5]:
    print(f"  {e['id']}: {e['a']} -> {e['b']} {e['len']:.1f} m type={e['type']}")
print("\nsaved ->", OUT, os.path.getsize(OUT), "bytes")
