# -*- coding: utf-8 -*-
"""把 LaneDriver 的巡航上限全局封顶到 4.40 m/s。

理由：平台按「1 秒窗内速度变化 > 5 m/s」判 excessive_acceleration（实测见
ACCEL_RESPONSE.md：刹车 0.15 → 4.5 不记，0.20 → 5.8 记）。只要任何一次
机动动作的 Δv ≤ 4.4，1 秒窗就永远踩不到 5。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_run5.py")
s = io.open(p, encoding="utf-8").read()

old = "        self.v_cap = float(v_cap)"
new = (
    "        # 全车队巡航上限 4.4：平台按「1 秒窗内速度变化 > 5 m/s」判违规，\n"
    "        # 只要单次机动的 Δv <= 4.4，就永远踩不到 5（实测见 ACCEL_RESPONSE.md）。\n"
    "        self.v_cap = min(float(v_cap), V_CAP_MAX)"
)
assert s.count(old) == 1, ("vcap", s.count(old))
s = s.replace(old, new)

old2 = "ACCEL = 1.15          # 规划加速度（m/s²），用于剩余距离的刹车点"
new2 = (
    old2 + "\n"
    "V_CAP_MAX = 4.40      # 全局巡航上限：保证单次速度变化 Δv < 5 m/s（违规判据的 1 秒窗）"
)
assert s.count(old2) == 1, ("accel", s.count(old2))
s = s.replace(old2, new2)

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: v_cap 封顶 4.40")
