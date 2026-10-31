# -*- coding: utf-8 -*-
"""把闸口相关坐标从写死改成"跟着推导值走"：

- GATE_READY / GATE_HOLD → gate_ready(ctx) / gate_hold(ctx)
- 申请线 729.05 → gate_x(ctx) + 0.25（0.25 是实测出来的偏移：中间偏右才判 eligible）
- 退回去重进的目标 (728.8,-55) → (gate_x(ctx), -55)
并在 stage_cross 开头记一次自检日志。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_mission.py")
s = io.open(p, encoding="utf-8").read()

# 1) 加三个取值函数
anchor = "def gate_x(ctx):"
assert s.count(anchor) == 1
helpers = '''def gate_ready(ctx):
    """闸口南侧待命点（request_entry 从这里起步最顺）。"""
    return (gate_x(ctx), -55.0)


def gate_hold(ctx):
    """B 的备用待命点：在 A 后面 19 m，避免两车抢同一个点。"""
    return (gate_x(ctx), -74.0)


def gate_req_line(ctx, y):
    """申请线：中线 +0.25 m（实测靠左的 728.7 会判 not eligible）。"""
    return (gate_x(ctx) + 0.25, y)


'''
s = s.replace(anchor, helpers + anchor)

# 2) 使用点替换
reps = [
    ('_job(ctx, "Car_A", GATE_READY, 3.7, 2.0, 130.0)',
     '_job(ctx, "Car_A", gate_ready(ctx), 3.7, 2.0, 130.0)'),
    ('_job(ctx, "Car_B", GATE_HOLD, 3.7, 2.0, 150.0, delay=8.0)',
     '_job(ctx, "Car_B", gate_hold(ctx), 3.7, 2.0, 150.0, delay=8.0)'),
    ('detour = nav_path(ctx, pos0, (728.8, -55.0))',
     'detour = nav_path(ctx, pos0, gate_ready(ctx))'),
    ('drive_polyline(ctx, car, [(p_back[0], p_back[1]), (729.05, -46.0)],',
     'drive_polyline(ctx, car, [(p_back[0], p_back[1]), gate_req_line(ctx, -46.0)],'),
    ('drive_polyline(ctx, car, [(p_now[0], p_now[1]), (729.05, -30.0)],',
     'drive_polyline(ctx, car, [(p_now[0], p_now[1]), gate_req_line(ctx, -30.0)],'),
]
for a, b in reps:
    n = s.count(a)
    assert n >= 1, (a, n)
    s = s.replace(a, b)

# 3) stage_cross 开头自检
old = "    sync_dyn_avoid(ctx)\n    # 3.0b Car_C 扫完路障就彻底驻车："
new = ("    sync_dyn_avoid(ctx)\n    gate_x(ctx)                      # 自检：推导值 vs 写死值，差 >3 m 会告警\n"
       "    # 3.0b Car_C 扫完路障就彻底驻车：")
assert s.count(old) == 1, ("stage", s.count(old))
s = s.replace(old, new)

io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: 闸口坐标已改为跟随推导值")
