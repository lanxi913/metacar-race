# -*- coding: utf-8 -*-
"""回滚全局 v_cap 封顶（实测反而更差：C 撞灯塔 9 次、B 到 568.9 s）。"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_run5.py")
s = io.open(p, encoding="utf-8").read()

old = (
    "        # 全车队巡航上限 4.4：平台按「1 秒窗内速度变化 > 5 m/s」判违规，\n"
    "        # 只要单次机动的 Δv <= 4.4，就永远踩不到 5（实测见 ACCEL_RESPONSE.md）。\n"
    "        self.v_cap = min(float(v_cap), V_CAP_MAX)"
)
new = (
    "        # 注：曾经把全局巡航封顶到 4.4 想一劳永逸压掉加速度违规，实测反而更差\n"
    "        # （C 的转向时序变了，撞灯塔 9 次、B 拖到 568.9 s），已回滚。\n"
    "        self.v_cap = float(v_cap)"
)
assert s.count(old) == 1, ("vcap", s.count(old))
s = s.replace(old, new)
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: 已回滚 v_cap 封顶")
