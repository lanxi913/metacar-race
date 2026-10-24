# -*- coding: utf-8 -*-
"""异形泊车静态数据探针：连接场景，dump 赛道/车道/车位/障碍/自车尺寸，不驱动车辆。

用法：
    .venv\\Scripts\\python.exe -u tools\\probe_angled_static.py
"""
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "logs" / "angled"


def enum_name(v):
    return getattr(v, "name", None) or str(v)


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...", flush=True)
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    route = [(float(p.x), float(p.y), float(getattr(p, "z", 0.0))) for p in static.route]
    segs = [{"name": s.name,
             "start": (float(s.start_point.x), float(s.start_point.y)) if s.start_point else None}
            for s in static.sub_scenes]
    print(f"赛道 {len(route)} 点, 子场景 {len(segs)}, 道路 {len(static.roads)}", flush=True)

    roads = []
    for rd in static.roads:
        lanes = []
        for ln in rd.lanes:
            lanes.append({
                "id": ln.id,
                "width": float(ln.width),
                "center": [(float(p.x), float(p.y)) for p in ln.path_points],
                "left": [(float(p.x), float(p.y)) for p in ln.left_border.path_points],
                "right": [(float(p.x), float(p.y)) for p in ln.right_border.path_points],
            })
        roads.append({"id": rd.id, "lanes": lanes})

    it = api.main_loop()
    msg = None
    for m, _f in it:
        msg = m
        break

    obs = []
    for ob in (msg.obstacles or []):
        obs.append({
            "id": getattr(ob, "id", None),
            "type": enum_name(getattr(ob, "type", None)),
            "pos_x": float(ob.pos_x), "pos_y": float(ob.pos_y), "ori_z": float(ob.ori_z),
            "length": float(ob.length), "width": float(ob.width),
            "extra": getattr(ob, "extra_info", None),
        })
    pose = msg.pose_gnss
    car = msg.main_vehicle
    data = {
        "route": route,
        "sub_scenes": segs,
        "roads": roads,
        "obstacles": obs,
        "ego": {
            "x": float(pose.pos_x), "y": float(pose.pos_y), "ori_z": float(pose.ori_z),
            "length": float(car.length), "width": float(car.width),
            "speed": float(car.speed), "gear": enum_name(car.gear),
        },
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    OUT.mkdir(parents=True, exist_ok=True)
    p = OUT / f"static_{time.strftime('%Y%m%d-%H%M%S')}.json"
    p.write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"已写入 {p}", flush=True)

    print("\n障碍物：", flush=True)
    for o in obs:
        print(f"  {o['type']:14s} id={o['id']} ({o['pos_x']:7.2f},{o['pos_y']:7.2f}) "
              f"ori={o['ori_z']:6.1f} {o['length']:.2f}x{o['width']:.2f}", flush=True)

    e = data["ego"]
    print(f"\n自车 ({e['x']:.2f},{e['y']:.2f}) ori={e['ori_z']:.1f} "
          f"长宽={e['length']:.2f}x{e['width']:.2f} gear={e['gear']}", flush=True)

    print(f"\n道路 {len(roads)} 条：", flush=True)
    for rd in roads:
        for ln in rd["lanes"]:
            c = ln["center"]
            print(f"  road {rd['id']} lane {ln['id']} 宽={ln['width']:.2f} "
                  f"中心线 {len(c)} 点 首{c[0] if c else None} 末{c[-1] if c else None}", flush=True)

    slots = [o for o in obs if "PARK" in o["type"].upper()]
    if slots and route:
        s = slots[0]
        near = [(x, y) for x, y, *_ in route if math.hypot(x - s["pos_x"], y - s["pos_y"]) < 30]
        print(f"\n车位 30 m 内赛道点 {len(near)} 个：", flush=True)
        for x, y in near:
            print(f"   ({x:8.2f},{y:8.2f})", flush=True)

    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    time.sleep(1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
