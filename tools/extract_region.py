# -*- coding: utf-8 -*-
"""从 Assembly-CSharp.dll 指定偏移区抽 UTF-16 串（两种字节对齐都试）。"""
import io
import sys

P = (r"C:\Users\lanxi\AppData\Roaming\com.i-mylab.metapower-desktop"
     r"\car\scenes\PORT-FOG-01\SmartCar_Data\Managed\Assembly-CSharp.dll")
OUT = r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\docs\portfog_wait_strings.txt"

start = int(sys.argv[1])
end = int(sys.argv[2])
data = open(P, "rb").read()[start:end]

lines = []
for shift in (0, 1):
    blob = data[shift:]
    i, n = 0, len(blob) - 1
    cur = bytearray()
    while i < n:
        ch = blob[i] | (blob[i + 1] << 8)
        ok = (0x20 <= ch <= 0x7E) or (0x4E00 <= ch <= 0x9FFF) or ch in (0x3001, 0x3002)
        if ok:
            cur += blob[i:i + 2]
            i += 2
        else:
            if len(cur) >= 4:
                try:
                    s = cur.decode("utf-16-le")
                    if sum(c.isalnum() or c in "_ " for c in s) > len(s) * 0.6:
                        lines.append(s)
                except Exception:
                    pass
            cur = bytearray()
            i += 2

seen = set()
uniq = [x for x in lines if not (x in seen or seen.add(x))]
io.open(OUT, "w", encoding="utf-8").write("\n".join(uniq))
print("strings:", len(uniq), "->", OUT)
