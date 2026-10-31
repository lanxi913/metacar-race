# -*- coding: utf-8 -*-
"""起步软油门：每个 drive 的前 1.3 s 把油门限到 0.035（针对 fog-950 版本的锚点）。

依据（ACCEL_RESPONSE.md 实测曲线，从静止、DRIVE 档）：
    0.01 → 1.21 m/s / 1.3 s   1 秒窗 ≈ 0.9  → 不记
    0.02 → 3.64 m/s / 1.3 s   1 秒窗 ≈ 2.8  → 不记
    0.05 → 5.27 m/s / 0.9 s   1 秒窗 ≈ 5.9  → 记
    0.50 → 4.83 m/s / 0.4 s   1 秒窗 ≈ 7.9  → 记
现在的映射 `0.08 + 0.28*dv` 起步即顶到 0.50，所以每次起步必记一条。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_run5.py")
s = io.open(p, encoding="utf-8").read()

old = "            t = clip(0.08 + 0.28 * dv, 0.08, THR_MAX)"
new = (
    "            t = clip(0.08 + 0.28 * dv, 0.08, THR_MAX)\n"
    "            if self.t < START_SOFT_S:\n"
    "                # 起步 1.3 s 内限幅：实测 0.02~0.035 时 1 秒窗只有 2.8~4，不会记违规\n"
    "                t = min(t, START_THR)"
)
assert s.count(old) == 1, ("thr", s.count(old))
s = s.replace(old, new)

old2 = (
    "                # 一帧 0.1→0.42 会被判 excessive_acceleration，改成一帧 +0.07 往上爬\n"
    "                self.thr = min(0.42, self.thr + 0.07)"
)
new2 = (
    "                # 一帧 0.1→0.42 会被判 excessive_acceleration，改成一帧 +0.07 往上爬；\n"
    "                # 起步 1.3 s 内要守软油门上限，否则一踢就回到 0.42。\n"
    "                cap = START_THR if self.t < START_SOFT_S else 0.42\n"
    "                self.thr = min(cap, self.thr + 0.07)"
)
assert s.count(old2) == 1, ("kick", s.count(old2))
s = s.replace(old2, new2)

old3 = "ACCEL = 1.15          # 规划加速度（m/s²），用于剩余距离的刹车点"
new3 = (
    old3 + "\n"
    "START_SOFT_S = 1.30   # 起步软油门时长（s）\n"
    "START_THR = 0.035     # 起步期间油门上限（实测 1 秒窗 ≈4，不触发违规）"
)
assert s.count(old3) == 1, ("const", s.count(old3))
s = s.replace(old3, new3)

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: 起步软油门 0.035 / 1.3 s")
