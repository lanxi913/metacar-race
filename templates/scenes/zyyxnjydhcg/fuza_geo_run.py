# -*- coding: utf-8 -*-
r"""场景元素 15「复杂路况超车」· 几何控制器版（无经验参数）

控制器：`metacar_race/control/geo_follower.py`
    速度剖面（曲率 + 规则加速度）、纯跟踪、障碍物换道避让（车道宽来自场景数据）、
    时限驱动的最低平均速度、前车净距保护。

场景实测几何（脚本运行时读，不写死）：
    route 148 点，x≈-1.88 直线到 y=-150；时限 30 s
    3 辆 CAR：本车道 (-1.90,-28.33)、左一 (1.62,-33.51)、左二 (5.14,-61.84)，ori 均 90°

用法：
    .venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\fuza_geo_run.py
    → 客户端 第 2 页 →「复杂路况超车」→ 点「启动场景」
"""
from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.geo_follower import GeoFollower  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-seconds", type=float, default=120.0)
    ap.add_argument("--jsonl", type=str, default="logs/fuza_geo.jsonl")
    args = ap.parse_args()

    print("等待场景连接（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"已连接。子场景: {[s.name for s in static.sub_scenes]}", flush=True)

    pts = [(float(p.x), float(p.y)) for p in static.route]
    widths = [float(ln.width) for rd in static.roads for ln in rd.lanes if float(ln.width) > 1.0]
    lane_w = statistics.median(widths) if widths else None
    ctrl = GeoFollower(pts, lane_width=lane_w)
    print(f"路线 {len(pts)} 点，{ctrl.path.total:.1f} m；车道宽（场景数据中位数）={lane_w}", flush=True)

    fh = open(args.jsonl, "w", encoding="utf-8")
    frame = 0
    try:
        for msg, _cam in api.main_loop():
            frame += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            time_left = None
            if getattr(st, "time_limit", 0) and float(st.time_limit) > 0:
                time_left = float(st.time_limit) - float(st.used_time)
            vc, info = ctrl.update(pose, float(car.speed), 1 / 30,
                                   obstacles=msg.obstacles, time_left=time_left)
            api.set_vehicle_control(vc)
            fh.write(f'{{"t":{st.used_time:.2f},"s":{info["s"]:.2f},"e":{info["e"]:.2f},'
                     f'"v":{float(car.speed):.2f},"vcmd":{info["v_cmd"]:.2f},'
                     f'"off":{ctrl.offset:.2f},"steer":{info["steer"]:.3f}}}\n')
            if frame % 15 == 1:
                print(f"[{st.used_time:5.1f}s] s={info['s']:6.1f} e={info['e']:+5.2f} "
                      f"off={ctrl.offset:+5.2f} v={float(car.speed):5.2f}/{info['v_cmd']:5.2f} "
                      f"κ={info['k']:+.3f} pos=({float(pose.pos_x):6.1f},{float(pose.pos_y):7.1f})",
                      flush=True)
            if ctrl.done or st.used_time >= args.max_seconds:
                print(f"结束：done={ctrl.done} s={info['s']:.1f}/{ctrl.path.total:.1f}", flush=True)
                break
    except KeyboardInterrupt:
        print("手动中断", flush=True)
    finally:
        fh.close()
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
