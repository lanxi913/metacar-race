# -*- coding: utf-8 -*-
"""连续场景探针：dump 赛道、子场景（元素）划分与时限，不驱动车辆。

用法：
    .venv\\Scripts\\python.exe -u tools\\probe_continuous.py

输出同时写到 logs/continuous/<时间戳>.json，便于对比 5 个连续场景的构成。
"""
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "logs" / "continuous"


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...", flush=True)
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    route = [(float(p.x), float(p.y)) for p in static.route]
    seg = [0.0]
    for i in range(1, len(route)):
        seg.append(seg[-1] + math.hypot(route[i][0] - route[i - 1][0],
                                        route[i][1] - route[i - 1][1]))
    total = seg[-1] if seg else 0.0
    print(f"赛道 {len(route)} 点，总长 {total:.1f} m", flush=True)

    # 每个元素的起点投影到赛道上，算里程区间
    def project(pt):
        best_i, best_d = 0, 1e9
        for i, (x, y) in enumerate(route):
            d = (x - pt[0]) ** 2 + (y - pt[1]) ** 2
            if d < best_d:
                best_i, best_d = i, d
        return seg[best_i], best_i

    subs = []
    for s in static.sub_scenes:
        rec = {"name": s.name}
        if s.start_point is not None:
            rec["start"] = [float(s.start_point.x), float(s.start_point.y)]
            rec["start_s"], _ = project(rec["start"])
        if getattr(s, "end_point", None) is not None:
            rec["end"] = [float(s.end_point.x), float(s.end_point.y)]
            rec["end_s"], _ = project(rec["end"])
        for attr in ("time_limit", "limit_time", "duration"):
            v = getattr(s, attr, None)
            if v is not None:
                rec[attr] = float(v)
        subs.append(rec)

    print(f"\n元素 {len(subs)} 个：", flush=True)
    prev_end = 0.0
    for i, r in enumerate(subs):
        s0 = r.get("start_s", prev_end)
        s1 = r.get("end_s", s0)
        prev_end = s1
        tl = r.get("time_limit", r.get("limit_time", r.get("duration", None)))
        print(f"  {i + 1}. {r['name']:12s} 里程 {s0:7.1f} ~ {s1:7.1f} m"
              f"（{s1 - s0:7.1f} m）"
              + (f"  时限 {tl:.0f} s" if tl else ""), flush=True)

    it = api.main_loop()
    msg = None
    for m, _f in it:
        msg = m
        break
    st = msg.scene_status
    print(f"\n场景时限 {getattr(st, 'time_limit', '?')} s，"
          f"当前元素 {getattr(st, 'sub_scene_name', '?')}", flush=True)

    data = {"route_len_m": total, "route_points": len(route), "sub_scenes": subs,
            "scene_time_limit": float(getattr(st, "time_limit", 0) or 0),
            "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    OUT.mkdir(parents=True, exist_ok=True)
    p = OUT / f"probe_{time.strftime('%Y%m%d-%H%M%S')}.json"
    p.write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"已写入 {p}", flush=True)

    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    time.sleep(1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
