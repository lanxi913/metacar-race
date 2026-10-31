# -*- coding: utf-8 -*-
"""1) 回滚 5e0d2f0 的转向增益（实测违规 15 条，比 c131140 的 8~12 差）；
2) 给 LaneDriver 加 10 Hz 轨迹记录（fog_run5.TRACE），供"画龙"定量分析。

记录字段：t, vid, x, y, steer, thr, brk, speed
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_run5.py")
s = io.open(p, encoding="utf-8").read()

# --- 1) 增益回滚 ---
for a, b in (("LAT_DAMP = 0.08", "LAT_DAMP = 0.10"),
             ("CURV_FF = 0.60", "CURV_FF = 0.70"),
             ("LOOKAHEAD_B = 0.50", "LOOKAHEAD_B = 0.45"),
             ("LOOKAHEAD_A = 1.2 ", "LOOKAHEAD_A = 1.0 ")):
    assert s.count(a) == 1, (a, s.count(a))
    s = s.replace(a, b)

# --- 2) 轨迹记录 ---
old = "ACCEL = 1.15          # 规划加速度（m/s²），用于剩余距离的刹车点"
new = (
    "TRACE = None          # 置成 list 就开始记 10 Hz 轨迹（t, vid, x, y, steer, thr, brk, speed）\n"
    + old
)
assert s.count(old) == 1, ("accel", s.count(old))
s = s.replace(old, new)

old = """        if goal <= 0.7 or (self.stop_xy is None and rem <= 0.5):"""
new = """        if TRACE is not None:
            try:
                TRACE.append((self.t, getattr(self, "vid", ""), x, y,
                              float(steer), float(self.thr), float(self.brk),
                              float(speed)))
            except Exception:                                  # noqa: BLE001
                pass

        if goal <= 0.7 or (self.stop_xy is None and rem <= 0.5):"""
assert s.count(old) == 1, ("trace", s.count(old))
s = s.replace(old, new)

# vid 记到 driver 上（没设就空着）
old = "        self._steer_prev = None"
if s.count(old) == 1:
    s = s.replace(old, "        self._steer_prev = None\n        self.vid = \"\"")

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: 增益已回滚 + TRACE 记录已加")
