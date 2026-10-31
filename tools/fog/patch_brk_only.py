# -*- coding: utf-8 -*-
"""只改一个变量：速度环刹车上限 0.90 -> 0.15（950 版本其余部分不动）。

依据 ACCEL_RESPONSE.md：从 5.2 m/s 起刹，0.15 → 1 秒窗 4.5 m/s²（不记），
0.20 → 5.8（记）。guard 的救命急刹不动、park_car 不动、v_cap 不动。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_run5.py")
s = io.open(p, encoding="utf-8").read()

old = "            b = clip(0.10 + 0.40 * (-dv - 0.25), 0.10, 0.90)"
new = (
    "            # 只改这一处：实测刹车 0.15 时 1 秒窗 4.5 m/s²（不记），0.20 就 5.8（记）。\n"
    "            b = clip(0.10 + 0.40 * (-dv - 0.25), 0.10, BRK_CAP)"
)
assert s.count(old) == 1, ("brake", s.count(old))
s = s.replace(old, new)

old2 = "ACCEL = 1.15          # 规划加速度（m/s²），用于剩余距离的刹车点"
new2 = old2 + "\nBRK_CAP = 0.15        # 速度环刹车上限（实测口径）"
assert s.count(old2) == 1, ("accel", s.count(old2))
s = s.replace(old2, new2)

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: 仅 BRK_CAP=0.15")
