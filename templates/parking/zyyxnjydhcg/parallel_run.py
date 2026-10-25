# -*- coding: utf-8 -*-
r"""平行车道泊车 · 基线模板（实测连续 5 次 210 分）

2026-10-23 实测：连续 5 次 210.00，停位离散 < 3 cm（dx 0.31~0.34 / dy 0.34~0.35）。

题面（场景元素 2.22，总分 200）：驶进与车道平行的可用车位并停稳；
>200 s 判 0；压线 −20；碰对手车 −5×v；另加通用加速度分（|a|≤5 每题 +10）。

几何（同一场景实测，脚本会自己算，不写死）：
    自车起点 (-6.35, 30.00) yaw=90
    车位 (-9.78,-2.84) 朝向 90°，5.5 × 2.5
    两侧车 (-9.78, +3.79) / (-9.78, -8.97)
    → 横向 L=3.43 m，满舵半径 R=5.38 m，入位角 θ=47°

用法（先脚本、后场景，顺序不能反）：
    .venv\Scripts\python.exe -u templates\parking\zyyxnjydhcg\parallel_run.py
    → 客户端 第 2 页 →「平行车道泊车」→ 点「启动场景」

参数在 metacar_race/config.py 的 PARALLEL_* 里。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.parallel_parking import ParallelParkingController  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="平行车道泊车")
    ap.add_argument("--max-seconds", type=float, default=200.0)
    args = ap.parse_args()

    print("等待场景连接（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    names = [s.name for s in api.get_scene_static_data().sub_scenes]
    print(f"子场景: {names}", flush=True)
    if not any("平行" in n for n in names):
        print("!! 当前不是「平行车道泊车」，退出（确认客户端第 2 页第 9 个卡片）", flush=True)
        return 2

    ctrl = ParallelParkingController()
    last_t = None
    frame = 0
    last_phase = ""
    try:
        for msg, _cam in api.main_loop():
            frame += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            dt = 1 / 30 if last_t is None else max(st.used_time - last_t, 1e-3)
            last_t = st.used_time

            vc, info = ctrl.update(pose, float(car.speed), msg.obstacles, dt)
            api.set_vehicle_control(vc)

            if info.phase != last_phase:
                last_phase = info.phase
                print(f"  → {info.phase}  pos=({float(pose.pos_x):.2f},{float(pose.pos_y):.2f}) "
                      f"yaw={float(pose.ori_z):.1f} v={float(car.speed):.2f} "
                      f"dx={info.dx:+.2f} dy={info.dy:+.2f} θ={info.theta:.0f}°", flush=True)

            if frame % 30 == 1:
                print(f"[{st.used_time:6.1f}s] {info.phase:8s} "
                      f"pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f}) "
                      f"yaw={float(pose.ori_z):6.1f} v={float(car.speed):5.2f} "
                      f"dx={info.dx:+.2f} dy={info.dy:+.2f}", flush=True)

            if ctrl.done() or st.used_time >= args.max_seconds:
                print(f"结束：{info.phase} frames={frame}", flush=True)
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
