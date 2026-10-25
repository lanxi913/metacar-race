# -*- coding: utf-8 -*-
r"""场景元素 15「复杂路况超车」· 入口（实测 113.81 分）

2026-10-23 实测：总得分 **113.81**（任务分 + 加速度 10，剩余为场景加分），
开始 23:27:58 / 结束 23:28:44，用时 46 s。

场景：同向多车道，前方 3 辆车，需要判断哪条车道空闲并完成超越。

实现：`metacar_race/control/lane_follower.py`（车道级循迹 + 换道避让 + 前车跟随），
参数在 `metacar_race/config.py` 的 `LANE_*`。

用法：
    .venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\fuza_run.py
    → 客户端 第 2 页 →「复杂路况超车」→ 点「启动场景」
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.lane_follower import LaneFollower  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-seconds", type=float, default=200.0)
    args = ap.parse_args()

    print("等待场景连接（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"已连接。子场景: {[s.name for s in static.sub_scenes]}", flush=True)

    ctrl = LaneFollower(static)
    print(f"车道 {len(ctrl.lanes)} 条，终点 {ctrl.goal}", flush=True)

    dt = 1 / 30
    frame = 0
    last = ""
    try:
        for msg, _cam in api.main_loop():
            frame += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            vc, info = ctrl.update(pose, float(car.speed), msg.obstacles, dt)
            api.set_vehicle_control(vc)
            if info["phase"] != last:
                last = info["phase"]
                print(f"  → {info['phase']:5s} pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f}) "
                      f"v={float(car.speed):5.2f} s={info['s']:6.1f} e={info['e']:+5.2f} "
                      f"rem={info['rem']:5.1f}", flush=True)
            if frame % 30 == 1:
                print(f"[{st.used_time:5.1f}s] {info['phase']:5s} "
                      f"pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f}) "
                      f"v={float(car.speed):5.2f} rem={info['rem']:5.1f}", flush=True)
            if ctrl.done() or st.used_time >= args.max_seconds:
                print(f"结束：{info['phase']} frames={frame}", flush=True)
                break
    except KeyboardInterrupt:
        print("手动中断。", flush=True)
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
