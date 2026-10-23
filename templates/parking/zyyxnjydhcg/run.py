# -*- coding: utf-8 -*-
r"""垂直车道泊车 · 基线模板（实测 210 分 = 任务 200 + 加速度 10）

2026-10-23 实测：停位 (-7.80,-0.02) 误差 0.10 m、yaw 1.9°、43 s，平台记 210.00。

题面（场景元素 2.21，总分 200）：驶进与车道垂直的可用车位并停稳；
>200 s 判 0；压线 −20；碰对手车 −5×v；另加通用加速度分（|a|≤5 每题 +10）。

几何（同一场景实测）：
    自车起点 (-2.16, 30.00) yaw=90，沿 -Y 驶来
    车位 (-7.80,-0.12) 朝向 0，5.5 × 2.5，两侧各停一辆车
    → 车道到车位中心横向 5.64 m，正好等于倒车圆弧半径（tanδ = 2.6/5.64 → 舵 0.70）

用法（先脚本、后场景，顺序不能反）：
    .venv\Scripts\python.exe -u templates\parking\zyyxnjydhcg\run.py
    → 客户端点「启动场景」→ 选「垂直车道泊车」

参数全部在 metacar_race/config.py 的 PARKING_* 里，这里只做入口与打点。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.parking import ParkingController  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="垂直车道泊车（倒车入库）")
    ap.add_argument("--max-seconds", type=float, default=200.0)
    args = ap.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    print(f"子场景: {[s.name for s in static.sub_scenes]}", flush=True)
    print("提示：车位坐标从 obstacles 里读（type=30），不写死。", flush=True)

    ctrl = ParkingController()
    last_t = None
    frames = 0
    last_phase = ""
    try:
        for msg, _cam in api.main_loop():
            frames += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            dt = 1 / 30 if last_t is None else max(st.used_time - last_t, 1e-3)
            last_t = st.used_time

            vc, info = ctrl.update(pose, float(car.speed), msg.obstacles, dt)
            api.set_vehicle_control(vc)

            if info.phase != last_phase:
                last_phase = info.phase
                print(
                    f"  → {info.phase}  pos=({float(pose.pos_x):.2f},{float(pose.pos_y):.2f}) "
                    f"yaw={float(pose.ori_z):.1f} v={float(car.speed):.2f} "
                    f"dist={info.dist_to_center:.2f} yaw_err={info.yaw_error:.1f}",
                    flush=True,
                )

            if frames % 30 == 1:
                print(
                    f"[{st.used_time:6.1f}s] {info.phase:9s} "
                    f"pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f}) "
                    f"yaw={float(pose.ori_z):6.1f} v={float(car.speed):5.2f} "
                    f"dist={info.dist_to_center:5.2f}",
                    flush=True,
                )

            if ctrl.done() or st.used_time >= args.max_seconds:
                print(f"结束：{info.phase} frames={frames}", flush=True)
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
