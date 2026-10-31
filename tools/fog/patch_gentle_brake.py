# -*- coding: utf-8 -*-
"""正常行驶/停车一律轻刹：刹车封顶 0.20，同时把减速规划提前一倍。

要点：软刹车必须配"提前减速"，否则会冲过停车点、然后靠蹭补位，反而更容易撞。
    - 速度环刹车：0.90 -> 0.20（实测 0.20 从 5.2 m/s 起刹，1 秒窗 5.8，属于卡线；
      但配合提前减速后，实际使用很少顶到 0.20）
    - 接近段规划减速度：1.15 -> 1.20（几乎不变）。真正的提前量靠"v_des 用规划减速度
      算出的距离包线"，这里把它改成更保守的 0.9 m/s²，车会明显更早开始收油。
    - guard 的救命急刹不动（上一次封它，C 一局撞吊机 12 次）。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_run5.py")
s = io.open(p, encoding="utf-8").read()

old = "            b = clip(0.10 + 0.40 * (-dv - 0.25), 0.10, 0.90)"
new = (
    "            # 轻刹：封顶 BRK_CAP。实测刹车 0.15 从 5.2 m/s 起刹 1 秒窗 4.5（不记），\n"
    "            # 0.20 是 5.8。配合下面提前一倍的减速规划，实际很少顶到上限。\n"
    "            b = clip(0.10 + 0.40 * (-dv - 0.25), 0.10, BRK_CAP)"
)
assert s.count(old) == 1, ("brake", s.count(old))
s = s.replace(old, new)

old2 = "                        math.sqrt(max(2.0 * self.accel * max(goal - 0.4, 0.0), 0.0)))"
new2 = (
    "                        # 提前减速：规划减速度取 APPROACH_DECEL（比实际刹车能力小），\n"
    "                        # 车会更早开始收油，后面就不需要补一脚急刹。\n"
    "                        math.sqrt(max(2.0 * APPROACH_DECEL * max(goal - 0.4, 0.0), 0.0)))"
)
assert s.count(old2) == 1, ("appr", s.count(old2))
s = s.replace(old2, new2)

old3 = "ACCEL = 1.15          # 规划加速度（m/s²），用于剩余距离的刹车点"
new3 = (
    old3 + "\n"
    "BRK_CAP = 0.20        # 速度环刹车上限（轻刹；急刹只在 guard 里）\n"
    "APPROACH_DECEL = 0.75 # 接近段规划减速度：越小越早收油（提前量在这里）"
)
assert s.count(old3) == 1, ("const", s.count(old3))
s = s.replace(old3, new3)

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: BRK_CAP=0.20, APPROACH_DECEL=0.75")
