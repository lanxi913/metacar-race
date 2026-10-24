# -*- coding: utf-8 -*-
r"""场景元素 16「上下坡驻车」· 入口（实测 200 分，任务满分）

2026-10-24 实测：总得分 **200.00**（位置扣分 0），用时 31 s。
加速度 +10 仍未拿到，原因与对策见 docs/scene-04-shangxiapo.md。

场景：80 m 爬坡（约 9% 坡），终点按 `scene_status.end_point` 判位置。

实现：`metacar_race/control/lane_follower.py`。
用法：
    .venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\shangxiapo_run.py
    → 客户端 第 2 页 →「上下坡驻车」→ 点「启动场景」
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
    ap.add_argument("--max-seconds", type=float, default=40.0)
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
    ep = None
    try:
        for msg, _cam in api.main_loop():
            frame += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            if ep is None and getattr(st, "end_point", None) is not None:
                ep = st.end_point
                ctrl.goal = (float(ep.x), float(ep.y))
                print(f"改用 end_point 作为目标: ({ctrl.goal[0]:.2f},{ctrl.goal[1]:.2f})", flush=True)
            vc, info = ctrl.update(pose, float(car.speed), msg.obstacles, dt)
            api.set_vehicle_control(vc)
            if info["phase"] != last:
                last = info["phase"]
                print(f"  → {info['phase']:5s} pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f},"
                      f"z={float(pose.pos_z):5.2f}) v={float(car.speed):5.2f} rem={info['rem']:5.1f}", flush=True)
            if frame % 30 == 1:
                print(f"[{st.used_time:5.1f}s] {info['phase']:5s} "
                      f"pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f},z={float(pose.pos_z):5.2f}) "
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
