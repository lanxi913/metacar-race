# -*- coding: utf-8 -*-
"""从 Assembly-CSharp.dll 里抽 UTF-16 字符串，摸清港口迷雾的协议字段。"""
import re
import sys

P = (r"C:\Users\lanxi\AppData\Roaming\com.i-mylab.metapower-desktop"
     r"\car\scenes\PORT-FOG-01\SmartCar_Data\Managed\Assembly-CSharp.dll")

start = int(sys.argv[1]) if len(sys.argv) > 1 else 1_900_000
span = int(sys.argv[2]) if len(sys.argv) > 2 else 60_000

data = open(P, "rb").read()[start:start + span]
# 连续的可打印 UTF-16LE（ASCII + 中文）片段
out = []
i = 0
n = len(data) - 1
cur = bytearray()
while i < n:
    lo, hi = data[i], data[i + 1]
    ch = lo | (hi << 8)
    ok = (0x20 <= ch <= 0x7E) or (0x4E00 <= ch <= 0x9FFF) or ch in (0x3001, 0x3002, 0xFF08, 0xFF09)
    if ok:
        cur += bytes((lo, hi))
        i += 2
    else:
        if len(cur) >= 8:
            try:
                out.append(cur.decode("utf-16-le"))
            except Exception:
                pass
        cur = bytearray()
        i += 2

print(f"region {start}..{start+span}: {len(out)} strings")
seen = set()
for s in out:
    if s in seen:
        continue
    seen.add(s)
    print(repr(s))
