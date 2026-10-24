# -*- coding: utf-8 -*-
r"""斜坡泊车（场景元素 24）· 复用平行泊车控制器 + 坡度适配

几何（脚本运行时读，不写死）：
    自车起点 (-6.35, 35.00) yaw=90
    车位 (-10.09,-2.84) 朝向 90°，5.5 × 2.5
    两侧车 (-9.78, +3.79) / (-9.78, -8.97)
    → 与「平行车道泊车」同构，只是横向 3.74 m（而不是 3.43 m），并且坡道 15°

用法（先脚本、后场景）：
    .venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\xiepo_run.py
    → 客户端 第 2 页 →「斜坡泊车」→ 点「启动场景」
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.slot_reverse import SlotReverseParker  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="斜坡泊车")
    ap.add_argument("--max-seconds", type=float, default=60.0)
    ap.add_argument("--jsonl", type=str, default="logs/xiepo.jsonl")
    ap.add_argument("--slope-thr", type=float, default=0.08, help="重力前馈油门")
    ap.add_argument("--slope-brake", type=float, default=0.35, help="坡道保持刹车")
    args = ap.parse_args()

    print("等待场景连接（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    names = [s.name for s in api.get_scene_static_data().sub_scenes]
    print(f"子场景: {names}", flush=True)
    if not any("斜坡" in n for n in names):
        print("!! 当前不是「斜坡泊车」，退出（确认客户端第 2 页第 12 个卡片）", flush=True)
        return 2

    ctrl = SlotReverseParker(slope_thr=args.slope_thr, slope_brake=args.slope_brake)
    print(f"坡度适配: slope_thr={ctrl.slope_thr:.3f} slope_brake={ctrl.slope_brake:.2f}", flush=True)
    fh = open(args.jsonl, "w", encoding="utf-8")
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
            fh.write(f'{{"t":{st.used_time:.2f},"x":{float(pose.pos_x):.3f},'
                     f'"y":{float(pose.pos_y):.3f},"yaw":{float(pose.ori_z):.2f},'
                     f'"v":{float(car.speed):.3f},"phase":"{info.phase}",'
                     f'"dx":{info.dx:.3f},"dy":{info.dy:.3f},"steer":{float(vc.steering):.3f}}}\n')

            if info.phase != last_phase:
                last_phase = info.phase
                print(f"  → {info.phase:8s} pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f}) "
                      f"yaw={float(pose.ori_z):6.1f} v={float(car.speed):5.2f} "
                      f"dx={info.dx:+.2f} dy={info.dy:+.2f}", flush=True)

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
        fh.close()
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())



