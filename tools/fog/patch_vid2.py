# -*- coding: utf-8 -*-
"""给真正驱动车的那两个地方打上 vid（trace.csv 就能按车分析）。"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_mission.py")
s = io.open(p, encoding="utf-8").read()

old = "    drv = LaneDriver(r, ctx.wb, v_cap=4.6, stop_xy=stop_xy, stop_m=stop_m)\n"
new = ("    drv = LaneDriver(r, ctx.wb, v_cap=4.6, stop_xy=stop_xy, stop_m=stop_m)\n"
       "    drv.vid = vid\n")
assert s.count(old) == 1, ("a", s.count(old))
s = s.replace(old, new)

old2 = ("        j[\"drv\"] = LaneDriver(fog_route.Route(dense, [j.get(\"v_cap\", 4.0)] * len(dense)),\n"
        "                              ctx.wb, v_cap=j.get(\"v_cap\", 4.0),\n"
        "                              stop_xy=j.get(\"stop_xy\"), stop_m=j.get(\"stop_m\", 0.0))\n")
new2 = (old2 + "        j[\"drv\"].vid = j.get(\"vid\", \"\")\n")
assert s.count(old2) == 1, ("b", s.count(old2))
s = s.replace(old2, new2)

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: vid 已打标")
