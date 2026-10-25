# -*- coding: utf-8 -*-
r"""场景元素 17「丁字路口通行」· 入口（实测 200 分）

2026-10-24 实测：总得分 **200.00**（扣分 0），
开始 11:16:56 / 结束 11:17:10，用时 14 s；路线 53 m。

T 型路口：路线 53 m，含一次左转；**不做换道避让**（路口横移会压线）。

实现：`metacar_race/control/route_follower.py`，参数在 `metacar_race/config.py` 的 `ROUTE_*`。

用法：
    .venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\dingzi_run.py
    → 客户端 第 2 页 →「辅路驶入、驶出」→ 点「启动场景」
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.route_follower import RouteFollower  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-seconds", type=float, default=200.0)
    args = ap.parse_args()

    print("等待场景连接（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"已连接。子场景: {[s.name for s in static.sub_scenes]}", flush=True)

    ctrl = RouteFollower(static)
    print(f"路线 {len(ctrl.pts)} 点，总长 {ctrl.total:.1f} m，终点 {ctrl.goal}", flush=True)

    dt = 1 / 30
    frame = 0
    last = ""
    try:
        for msg, _cam in api.main_loop():
            frame += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            vc, info = ctrl.update(pose, float(car.speed), dt)
            api.set_vehicle_control(vc)
            if info["phase"] != last:
                last = info["phase"]
                print(f"  → {info['phase']:5s} pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f}) "
                      f"v={float(car.speed):5.2f} s={info['s']:6.1f}/{info['total']:.1f} "
                      f"rem={info['rem']:5.1f}", flush=True)
            if frame % 60 == 1:
                print(f"[{st.used_time:6.1f}s] {info['phase']:5s} v={float(car.speed):5.2f} "
                      f"s={info['s']:6.1f} rem={info['rem']:5.1f}", flush=True)
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

