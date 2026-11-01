# -*- coding: utf-8 -*-
"""把 Assembly-CSharp.dll 里几段协议相关区域的可读串导出成文件（UTF-8）。"""
import sys

P = (r"C:\Users\lanxi\AppData\Roaming\com.i-mylab.metapower-desktop"
     r"\car\scenes\PORT-FOG-01\SmartCar_Data\Managed\Assembly-CSharp.dll")
OUT = r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\docs\portfog_protocol_strings.txt"

WINDOWS = [(1916000, 6000), (1939000, 6000), (1941000, 4000)]

data = open(P, "rb").read()
lines = []
for start, span in WINDOWS:
    blob = data[start:start + span]
    i, n = 0, len(blob) - 1
    cur = bytearray()
    while i < n:
        lo, hi = blob[i], blob[i + 1]
        ch = lo | (hi << 8)
        ok = (0x20 <= ch <= 0x7E) or (0x4E00 <= ch <= 0x9FFF) or ch in (0x3001, 0x3002)
        if ok:
            cur += bytes((lo, hi))
            i += 2
        else:
            if len(cur) >= 6:
                try:
                    lines.append(f"{start+i:08x}  {cur.decode('utf-16-le')}")
                except Exception:
                    pass
            cur = bytearray()
            i += 2

open(OUT, "w", encoding="utf-8").write("\n".join(lines))
print("wrote", OUT, "lines:", len(lines))
