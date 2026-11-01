# -*- coding: utf-8 -*-
"""VLA 场景接口探针：进场景后**不动车**，把能拿到的东西全打出来。

用途（用户 2026-11-01 10:26 的要求）：进场景 → 先检查接口和任务 → 再规划路线 → 最后跑。
检查内容：
  1. `get_scene_static_data()`：route / roads / sub_scenes / vla_extension（buildings、regions）
  2. `main_loop` 每帧给的东西：pose / 主车 / obstacles（类型、位置、extra_info）/ 相机路数与分辨率 /
     scene_status（子场景名、已用时、时限、end_point）
  3. 把 obstacles 的变化（什么时候出现、什么类型）按秒记录 —— 判断"车辆到底会不会出现在 API 里"
  4. 4 路相机各存一张图，方便人眼确认场景里有什么

用法（先跑脚本，再点启动场景）：
    .venv\\Scripts\\python.exe tools/vla_probe.py --seconds 25 --shot tmp/probe
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="VLA 场景接口探针（不动车）")
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--shot", default="", help="前缀，存 4 路相机图")
    args = ap.parse_args()

    print("连接仿真场景...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print("=== SceneStaticData ===", flush=True)
    print(f"  route: {len(getattr(static, 'route', None) or [])} 点", flush=True)
    print(f"  roads: {len(getattr(static, 'roads', None) or [])} 条", flush=True)
    subs = list(getattr(static, "sub_scenes", None) or [])
    print(f"  sub_scenes: {[s.name for s in subs]}", flush=True)
    for s in subs:
        print(f"     {s.name}: start={getattr(s, 'start_point', None)} "
              f"end={getattr(s, 'end_point', None)}", flush=True)
    ext = getattr(static, "vla_extension", None)
    print(f"  vla_extension: {'有' if ext else '无'}", flush=True)
    if ext is not None:
        print(f"     buildings={len(ext.buildings)}", flush=True)
        for b in ext.buildings:
            print(f"       B {b.name} pos=({b.pos_x:.1f},{b.pos_y:.1f}) "
                  f"{b.length:.1f}x{b.width:.1f} ori={b.ori_z:.0f}", flush=True)
        print(f"     regions={len(ext.regions)}", flush=True)
        for r in ext.regions:
            print(f"       R id={r.id} type={r.type} pos=({r.pos_x:.1f},{r.pos_y:.1f}) "
                  f"{r.length:.1f}x{r.width:.1f} ori={r.ori_z:.0f}", flush=True)

    t0 = time.time()
    frame = 0
    last_log = -1.0
    seen_types: Counter = Counter()
    first_seen: dict[str, float] = {}
    shot_saved = False

    for msg, frames in api.main_loop():
        frame += 1
        now = time.time() - t0
        st = msg.scene_status
        pose = msg.pose_gnss
        car = msg.main_vehicle
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))

        if frame == 1:
            print("=== 第一帧 ===", flush=True)
            print(f"  pose=({pose.pos_x:.2f},{pose.pos_y:.2f}) ori_z={pose.ori_z:.1f} "
                  f"v={float(car.speed):.2f}", flush=True)
            print(f"  主车尺寸 {float(car.length):.2f}x{float(car.width):.2f}"
                  f"x{float(car.height):.2f}", flush=True)
            print(f"  scene_status: sub_scene={getattr(st, 'sub_scene_name', '')!r} "
                  f"used={getattr(st, 'used_time', 0):.1f} "
                  f"limit={getattr(st, 'time_limit', 0):.1f} "
                  f"end_point={getattr(st, 'end_point', None)}", flush=True)
            print(f"  相机: {len(frames or [])} 路", flush=True)
            for k, cf in enumerate(frames or []):
                shp = getattr(cf.frame, "shape", None)
                print(f"     cam{k}: shape={shp}", flush=True)
            print(f"  障碍物: {len(msg.obstacles)}", flush=True)
            for o in msg.obstacles[:20]:
                print(f"     id={o.id} {getattr(o.type, 'name', o.type)} "
                      f"({o.pos_x:.1f},{o.pos_y:.1f}) {o.length:.1f}x{o.width:.1f} "
                      f"extra={getattr(o, 'extra_info', None)!r}", flush=True)
            if args.shot:
                import cv2
                for k, cf in enumerate(frames or []):
                    try:
                        cv2.imwrite(f"{args.shot}_cam{k}.png", cf.frame)
                    except Exception:  # noqa: BLE001
                        pass
                shot_saved = True
                print(f"  已存 {len(frames or [])} 张图 → {args.shot}_cam*.png", flush=True)

        # 障碍物变化：类型 + 首次出现时刻
        for o in msg.obstacles or []:
            name = getattr(getattr(o, "type", None), "name", str(o.type))
            seen_types[name] += 1
            first_seen.setdefault(name, round(now, 1))

        if now - last_log >= 3.0:
            last_log = now
            print(f"[{now:5.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"v={float(car.speed):.2f} obstacles={len(msg.obstacles)} "
                  f"types={dict(seen_types)}", flush=True)

        if now > args.seconds:
            break

    print("=== 汇总 ===", flush=True)
    print("  obstacles 类型统计: " + json.dumps(dict(seen_types), ensure_ascii=False), flush=True)
    print("  首次出现时刻: " + json.dumps(first_seen, ensure_ascii=False), flush=True)
    print(f"  帧数 {frame}，相机 {'已存图' if shot_saved else '未存'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
