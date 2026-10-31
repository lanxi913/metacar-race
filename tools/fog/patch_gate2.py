# -*- coding: utf-8 -*-
"""修正 derive_gate：闸口是**南北向**的一对车道（E/W 双车道），
不是横向路。用「竖直短路的 x 均值」当闸口中线。

训练场景实测：x=726.5（E 车道）与 x=731.0（W 车道）→ 均值 728.75，与写死 728.8 差 5 cm。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_mission.py")
s = io.open(p, encoding="utf-8").read()

start = s.index("def derive_gate(ctx):")
end = s.index("def gate_x(ctx):")
new_fn = '''def derive_gate(ctx):
    """从路网自动找闸口中线：闸口是南北向的一对车道（E/W），取它们的 x 均值。

    训练场景实测：E 车道 x=726.5、W 车道 x=731.0（y ∈ [-33,-15.5]），均值 728.75，
    和写死的 728.8 只差 5 cm —— 所以这个推导可以作为地图变化时的兜底。
    """
    try:
        roads = ctx.graph.roads
    except Exception:                                          # noqa: BLE001
        return None
    xs = []
    for rid, r in roads.items():
        b, e = r["begin"], r["end"]
        L = float(r.get("len") or 0.0)
        if not (4.0 <= L <= 30.0):
            continue
        if abs(b[0] - e[0]) > 2.0:          # 只要竖直路（南北向车道）
            continue
        x = 0.5 * (b[0] + e[0])
        if not (722.0 <= x <= 736.0):
            continue
        ylo, yhi = min(b[1], e[1]), max(b[1], e[1])
        if yhi < -40.0 or ylo > -10.0:      # 位于闸口那一段
            continue
        xs.append(x)
    if not xs:
        return None
    return {"x": sum(xs) / len(xs), "n": len(xs),
            "lo": min(xs), "hi": max(xs)}


'''
s = s[:start] + new_fn + s[end:]
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: derive_gate 改为竖直车道均值")
