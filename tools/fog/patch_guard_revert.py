# -*- coding: utf-8 -*-
"""回滚 guard 急刹封顶。

教训：把 guard 的急刹封到 0.30 之后，C 在 172434 那局撞了 QuayCrane_3 12 次、
灯塔 4 次（16 条 collision）。防撞是最后一道防线，刹车能力必须保留；
柔化只应该作用在"正常行驶/正常停车"上。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_safety.py")
s = io.open(p, encoding="utf-8").read()

old = (
    "            # 急刹封顶 0.30：实测（ACCEL_RESPONSE.md）刹车 >0.15 时 1 秒窗就超 5 m/s²，\n"
    "            # 0.30 在\"能停住\"和\"不记违规\"之间；真正的避碰靠提前减速，不靠这一脚。\n"
    "            return 0.0, min(max(brk, 0.45 + 0.45 * need), 0.30), steer, (gap, oid)"
)
new = (
    "            # 注：曾把这一脚封到 0.30 想省违规，结果刹不住 ——\n"
    "            # C 一局撞 QuayCrane_3 12 次 + 灯塔 4 次（16 条 collision）。\n"
    "            # 防撞是最后一道防线，刹车能力保留；柔化只作用于正常行驶与正常停车。\n"
    "            return 0.0, max(brk, 0.45 + 0.45 * need), steer, (gap, oid)"
)
assert s.count(old) == 1, ("guard", s.count(old))
s = s.replace(old, new)
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: guard 急刹已恢复")
