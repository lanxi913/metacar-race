# -*- coding: utf-8 -*-
"""加"闸口自动推导 + 启动自检"：

- `derive_gate(ctx)`：从路网里找出跨在东西两侧之间的那条短通道（闸口），
  返回中点坐标；找不到返回 None（调用方回退到写死的 728.8）。
- `gate_x(ctx)`：带缓存的取值，并在第一次使用时把"推导值 vs 写死值"写进日志。
  两者差超过 3 m 就是地图变了，日志里会打一行醒目的 `!! 闸口位置变了`。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_mission.py")
s = io.open(p, encoding="utf-8").read()

anchor = "def _pre(ctx, jobs, seconds, tag):"
assert s.count(anchor) == 1, ("anchor", s.count(anchor))

helper = '''DEFAULT_GATE_X = 728.8          # 训练场景实测的闸口中线（两侧墙在 726.26 / 731.32）
GATE_WARN_M = 3.0               # 推导值与写死值差超过这么多就告警（说明地图变了）


def derive_gate(ctx):
    """从路网里把闸口自动找出来：跨在东西两侧之间、近似水平、长度 5~25 m 的那条路。

    训练场景实测：闸口两侧的节点是 (723.5,-53.0) 和 (734.0,-53.0)，中点 728.75，
    和写死的 728.8 只差 5 cm —— 所以这个推导是可靠的，可以当地图变化时的兜底。
    """
    try:
        roads = ctx.graph.roads
    except Exception:                                          # noqa: BLE001
        return None
    best = None
    for rid, r in roads.items():
        b, e = r["begin"], r["end"]
        L = float(r.get("len") or 0.0)
        if not (5.0 <= L <= 25.0):
            continue
        if abs(b[1] - e[1]) > 4.0:          # 要求近似水平（闸口是横向通道）
            continue
        mx = 0.5 * (b[0] + e[0])
        my = 0.5 * (b[1] + e[1])
        if not (700.0 <= mx <= 760.0) or my > 10.0:
            continue
        lo, hi = min(b[0], e[0]), max(b[0], e[0])
        if lo > DEFAULT_GATE_X - 1.0 or hi < DEFAULT_GATE_X + 1.0:
            continue                       # 两端必须真跨在中线两侧
        if best is None or L < best[0]:
            best = (L, mx, my, rid)
    if best is None:
        return None
    return {"len": best[0], "x": best[1], "y": best[2], "rid": best[3]}


def gate_x(ctx):
    """闸口中线 x（带缓存 + 一次性自检日志）。"""
    if getattr(ctx, "gate_x", None) is not None:
        return ctx.gate_x
    g = derive_gate(ctx)
    if g is None:
        ctx.gate_x = DEFAULT_GATE_X
        ctx.note("[gate] 路网里没找到闸口 → 回退写死值 %.2f" % DEFAULT_GATE_X)
        return ctx.gate_x
    ctx.gate_x = float(g["x"])
    d = abs(ctx.gate_x - DEFAULT_GATE_X)
    ctx.note("[gate] 路网推导闸口 x=%.2f y=%.2f（%s，长 %.1f m），与写死值差 %.2f m"
             % (g["x"], g["y"], g["rid"], g["len"], d))
    if d > GATE_WARN_M:
        ctx.note("!! 闸口位置变了（差 %.1f m）—— 路线是按 x=%.1f 写的，"
                 "申请/待命点会自动跟着走，但助跑与出闸路线需要人工复核" % (d, DEFAULT_GATE_X))
    return ctx.gate_x


'''
s = s.replace(anchor, helper + anchor)
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: derive_gate / gate_x 已加入")
