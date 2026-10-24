# -*- coding: utf-8 -*-
r"""连续场景 1 · 入口（几何循迹，无经验参数）

控制器：`metacar_race/control/geo_follower.py`
    - 规则常数：|a| ≤ 5 m/s²、限速 60 km/h
    - 车辆参数（轴距、舵角→曲率）在线标定
    - 路径曲率来自 static.route 三点外接圆
    - 横向 Stanley（增益 1）、纵向归一化误差

用法：
    .venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\lianxu1_run.py
    → 客户端 第 3 页 →「连续场景1」→ 点「启动场景」
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.geo_follower import GeoFollower  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-seconds", type=float, default=900.0)
    ap.add_argument("--jsonl", type=str, default="logs/lianxu1.jsonl")
    args = ap.parse_args()

    print("等待场景连接（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"已连接。子场景: {[s.name for s in static.sub_scenes]}", flush=True)

    pts = [(float(p.x), float(p.y)) for p in static.route]
    ctrl = GeoFollower(pts)
    print(f"路线 {len(pts)} 点，总长 {ctrl.path.total:.1f} m", flush=True)

    fh = open(args.jsonl, "w", encoding="utf-8")
    frame = 0
    t0 = None
    try:
        for msg, _cam in api.main_loop():
            frame += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            if t0 is None:
                t0 = st.used_time
            vc, info = ctrl.update(pose, float(car.speed), 1 / 30)
            api.set_vehicle_control(vc)
            fh.write(f'{{"t":{st.used_time:.2f},"x":{float(pose.pos_x):.2f},"y":{float(pose.pos_y):.2f},'
                     f'"yaw":{float(pose.ori_z):.1f},"v":{float(car.speed):.2f},'
                     f'"s":{info["s"]:.2f},"e":{info["e"]:.2f},"vcmd":{info["v_cmd"]:.2f},'
                     f'"k":{info["k"]:.4f},"L":{info["L"]:.2f}}}\n')
            if frame % 30 == 1:
                print(f"[{st.used_time:6.1f}s] s={info['s']:7.1f} e={info['e']:+5.2f} "
                      f"v={float(car.speed):5.2f}/{info['v_cmd']:5.2f} κ={info['k']:.3f} "
                      f"L={info['L']:.2f} pos=({float(pose.pos_x):7.1f},{float(pose.pos_y):7.1f})",
                      flush=True)
            if ctrl.done or st.used_time >= args.max_seconds:
                print(f"结束：done={ctrl.done} frames={frame} s={info['s']:.1f}/{ctrl.path.total:.1f}", flush=True)
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
