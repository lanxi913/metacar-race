# -*- coding: utf-8 -*-
"""一轮改完：让车走直线（消除"一扭一扭"）。

根因：
1. `_heading()` 每走 1.0 m 就用"最近两点连线"重算航向 —— 4.8 m/s 相当于每 0.2 s
   跳一次，噪声几度，直接进纯跟踪的转向解算；
2. 转向输出没有任何滤波/限幅，每帧的抖动原样打在方向盘上；
3. 前视距离随速度增长偏小（1.0 + 0.45v），高速时对航向噪声更敏感。

改法：
- 航向：弦长 1.0 m → 1.5 m，并加一阶低通（α=0.35），只在直行时更稳；
- 转向：一阶低通（新解算占 0.45）+ 单帧变化限幅 0.12；
- 前视：LOOKAHEAD_B 0.45 → 0.60（90° 弯的 LOOKAHEAD_A=1.0 不动，不影响贴弯）。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_run5.py")
s = io.open(p, encoding="utf-8").read()

# 1) 常量
old = "LOOKAHEAD_B = 0.45"
new = "LOOKAHEAD_B = 0.60"
assert s.count(old) == 1, ("lkb", s.count(old))
s = s.replace(old, new)

old = "COLD_START_M = 1.0   # 位移航向没测出来前直行的距离"
new = (
    "COLD_START_M = 1.5   # 位移航向测一次要走的距离（1.0 太短，噪声大，车会画龙）\n"
    "HEAD_LPF = 0.35      # 航向一阶低通系数（越小越稳，太小平弯道会滞后）\n"
    "STEER_LPF = 0.45     # 转向一阶低通：新解算占的比重\n"
    "STEER_SLEW = 0.12    # 转向单帧最大变化量（防抖）"
)
assert s.count(old) == 1, ("cold", s.count(old))
s = s.replace(old, new)

# 2) 航向低通
old = """        moved = math.hypot(x - self._ax, y - self._ay)
        if moved >= COLD_START_M:
            self._mh = math.degrees(math.atan2(y - self._ay, x - self._ax))
            self._ax, self._ay = x, y
            self._cold = 0.0"""
new = """        moved = math.hypot(x - self._ax, y - self._ay)
        if moved >= COLD_START_M:
            raw = math.degrees(math.atan2(y - self._ay, x - self._ax))
            if self._mh is None:
                self._mh = raw
            else:
                # 一阶低通：位移航向本身是"两点连线"，逐次跳几度，直接进转向就会画龙
                self._mh += HEAD_LPF * wrap180(raw - self._mh)
                self._mh = wrap180(self._mh)
            self._ax, self._ay = x, y
            self._cold = 0.0"""
assert s.count(old) == 1, ("head", s.count(old))
s = s.replace(old, new)

# 3) 转向低通 + 限幅（在 clip 到 LANE_STEER_LIMIT 之后、return 之前）
old = """            steer = clip(steer, -LANE_STEER_LIMIT, LANE_STEER_LIMIT)"""
new = """            steer = clip(steer, -LANE_STEER_LIMIT, LANE_STEER_LIMIT)
            # 转向平滑：低通 + 单帧限幅，别把每帧的抖动原样打给方向盘
            if self._steer_prev is not None:
                steer = self._steer_prev + STEER_LPF * (steer - self._steer_prev)
                steer = clip(steer, self._steer_prev - STEER_SLEW,
                             self._steer_prev + STEER_SLEW)
            self._steer_prev = steer"""
assert s.count(old) == 1, ("steer", s.count(old))
s = s.replace(old, new)

# 4) 初始化 _steer_prev
old = "        self._stall = 0.0\n"
new = "        self._stall = 0.0\n        self._steer_prev = None\n"
assert s.count(old) >= 1, ("init", s.count(old))
s = s.replace(old, new, 1)

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: 航向低通 + 转向低通/限幅 + 前视加长")
