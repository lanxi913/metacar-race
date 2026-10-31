# -*- coding: utf-8 -*-
"""只加一项：转向输出低通 + 单帧变化限幅（不动航向估计、不动前视）。

理由：离线自测里直线横向误差只有 0.11 m，说明参考线与纯跟踪本身没问题；
现场看到"一扭一扭"，最可能是每帧转向解算里的噪声（位移航向、曲率前馈、
横向阻尼三项叠加）原样打给方向盘。这里只对输出做平滑，不改变控制律本身。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_run5.py")
s = io.open(p, encoding="utf-8").read()

old = "LAT_DAMP = 0.10      # 横向偏差进转向的阻尼系数（e>0 = 车在路径左侧 → 回正向右）"
new = (
    "STEER_LPF = 0.55     # 转向输出一阶低通（新解算占的比重，1.0=不过滤）\n"
    "STEER_SLEW = 0.10    # 转向单帧最大变化量（10 Hz 下 1.0/s，防抖）\n" + old
)
assert s.count(old) == 1, ("const", s.count(old))
s = s.replace(old, new)

old = "            steer = clip(steer, -LANE_STEER_LIMIT, LANE_STEER_LIMIT)"
new = (
    "            steer = clip(steer, -LANE_STEER_LIMIT, LANE_STEER_LIMIT)\n"
    "            # 输出平滑：低通 + 单帧限幅（只压抖动，不改控制律）\n"
    "            if self._steer_prev is not None:\n"
    "                steer = self._steer_prev + STEER_LPF * (steer - self._steer_prev)\n"
    "                steer = clip(steer, self._steer_prev - STEER_SLEW,\n"
    "                             self._steer_prev + STEER_SLEW)\n"
    "            self._steer_prev = steer"
)
assert s.count(old) == 1, ("steer", s.count(old))
s = s.replace(old, new)

old = "        self._stall = 0.0\n"
new = "        self._stall = 0.0\n        self._steer_prev = None\n"
assert s.count(old) >= 1, ("init", s.count(old))
s = s.replace(old, new, 1)

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: 只加转向输出低通 0.55 + 限幅 0.10")
