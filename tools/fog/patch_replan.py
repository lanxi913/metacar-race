# -*- coding: utf-8 -*-
"""两件事：
1) 行驶途中**每 2 秒**检查一次动态障碍表，一出现新路障就地把当前这条腿重规划；
2) 提速：去掉起步软油门（START_SOFT_S=0），并把各段巡航上限抬高。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_mission.py")
s = io.open(p, encoding="utf-8").read()

# ---------- 1) 中途重规划 ----------
old = """            pos, yaw, speed = pose_of(ctx.api, vid)
            ctx.poll_events_throttled()"""
new = """            pos, yaw, speed = pose_of(ctx.api, vid)
            ctx.poll_events_throttled()
            # 每 2 s 看一眼动态障碍表：出现新路障就地把这条腿重规划
            if time.time() - float(j.get("_t_chk", 0.0)) > 2.0:
                j["_t_chk"] = time.time()
                try:
                    n0 = len(getattr(ctx, "dyn_avoid", None) or [])
                    sync_dyn_avoid(ctx)
                    n1 = len(getattr(ctx, "dyn_avoid", None) or [])
                    if n1 > n0:
                        goal = j.get("stop_xy") or j["pts"][-1]
                        seg = nav_path(ctx, pos, goal)
                        if seg and len(seg) >= 2:
                            dense = _dense(seg)
                            j["drv"] = LaneDriver(
                                fog_route.Route(dense, [j.get("v_cap", 4.0)] * len(dense)),
                                ctx.wb, v_cap=j.get("v_cap", 4.0),
                                stop_xy=j.get("stop_xy"), stop_m=j.get("stop_m", 0.0))
                            j["drv"].vid = vid
                            j["drv"].reset_heading()
                            j["pts"] = list(seg)
                            ctx.note("    [replan] 新路障 → %s 就地重规划（%d 点，去 (%.1f,%.1f)）"
                                     % (vid, len(dense), goal[0], goal[1]))
                except Exception as exc:                       # noqa: BLE001
                    ctx.note("    [replan] %s 重规划失败 %s" % (vid, exc))"""
assert s.count(old) == 1, ("loop", s.count(old))
s = s.replace(old, new)

# ---------- 2) 提速 ----------
old = "_job(ctx, \"Car_A\", gate_ready(ctx), 3.7, 2.0, 130.0)"
new = "_job(ctx, \"Car_A\", gate_ready(ctx), 4.8, 2.0, 130.0)"
assert s.count(old) == 1
s = s.replace(old, new)

old = "_job(ctx, \"Car_B\", gate_hold(ctx), 3.7, 2.0, 150.0, delay=8.0)"
new = "_job(ctx, \"Car_B\", gate_hold(ctx), 4.8, 2.0, 150.0, delay=8.0)"
assert s.count(old) == 1
s = s.replace(old, new)

old = "_job(ctx, \"Car_A\", (ca[0], ca[1] - 28.0), 3.7, 2.0, 130.0)"
new = "_job(ctx, \"Car_A\", (ca[0], ca[1] - 28.0), 4.8, 2.0, 130.0)"
assert s.count(old) == 1
s = s.replace(old, new)

old = "_job(ctx, \"Car_B\", (cb[0], cb[1] - 28.0), 4.1, 2.0, 130.0)"
new = "_job(ctx, \"Car_B\", (cb[0], cb[1] - 28.0), 4.8, 2.0, 130.0)"
assert s.count(old) == 1
s = s.replace(old, new)

old = "_job(ctx, \"Car_D\", (ca[0], ca[1] + 15.0), 3.9, 2.0, 175.0)"
new = "_job(ctx, \"Car_D\", (ca[0], ca[1] + 15.0), 4.6, 2.0, 175.0)"
assert s.count(old) == 1
s = s.replace(old, new)

old = "_job(ctx, \"Car_D\", (cb[0], cb[1] + 22.0), 3.9, 2.0, 175.0)"
new = "_job(ctx, \"Car_D\", (cb[0], cb[1] + 22.0), 4.6, 2.0, 175.0)"
assert s.count(old) == 1
s = s.replace(old, new)

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: 中途重规划 + 提速（v_cap 3.7/4.1/3.9 -> 4.8/4.6）")
