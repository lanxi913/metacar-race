# -*- coding: utf-8 -*-
r"""车辆直行 · 基线模板（目标 110 分 = 任务 100 + 加速度 ≤5 m/s² 的 10）

为什么用定油门、不用速度闭环（2026-10-23 实测结论）：
    速度闭环最好只到 100 分：为了触发升挡必须把油门顶到 1.0，会在起飞段
    产生 >5 m/s² 的加速度尖峰，丢掉 10 分加速度分。
    定油门 0.5 匀速跑完全程，实测稳定 110 分。

为什么油门不能低于 0.45：
    低挡位下油门再大也上不去，车会卡在 17.2 km/h；0.5 是实测能稳定升挡的下限。

用法（顺序不能反）：
    1) .venv\Scripts\python.exe -u templates\straight\zyyxnjydhcg\run.py
    2) 客户端点「启动场景」
    3) 终端开始刷 pos/v，跑完自动刹车退出

参数：
    --throttle     定油门，默认 0.5；<0.45 会卡 17.2 km/h
    --lookahead    纯跟踪前视距离（m），默认 6.0；直行给大一点更稳
    --max-seconds  保护上限（s），默认 100，对应元素题时限
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]   # templates/<模块>/<名字>/run.py → 仓库根
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.tracking import PurePursuit  # noqa: E402

DEFAULT_THROTTLE = 0.5      # 实测能升挡且不产生加速度尖峰
DEFAULT_LOOKAHEAD = 6.0     # 直行场景前视给大，减少方向抖动
UPSHIFT_FLOOR = 0.45        # 低于这个油门会卡在 17.2 km/h（低挡位）


def main() -> int:
    parser = argparse.ArgumentParser(description="车辆直行基线模板")
    parser.add_argument("--throttle", type=float, default=DEFAULT_THROTTLE)
    parser.add_argument("--lookahead", type=float, default=DEFAULT_LOOKAHEAD)
    parser.add_argument("--max-seconds", type=float, default=100.0)
    args = parser.parse_args()

    if args.throttle < UPSHIFT_FLOOR:
        print(f"提醒：油门 {args.throttle} < {UPSHIFT_FLOOR}，大概率卡在 17.2 km/h。")
    args.throttle = max(0.0, min(1.0, args.throttle))

    print("连接仿真场景（先跑脚本，再点启动场景，这里会等）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    route = list(static.route)
    if len(route) < 2:
        print("route 点不足，无法循迹。", flush=True)
        return 1
    pursuit = PurePursuit(route, lookahead=args.lookahead)
    print(
        f"子场景: {[s.name for s in static.sub_scenes]}  "
        f"route={len(route)}  lookahead={args.lookahead:.1f}  "
        f"throttle={args.throttle:.2f}",
        flush=True,
    )

    start = time.time()
    frames = 0
    try:
        for msg, _camera_frames in api.main_loop():
            frames += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status

            # 场景 yaw 顺时针为正，PurePursuit 内部已按数学坐标处理
            steering = pursuit.steering(
                float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
            )
            api.set_vehicle_control(
                VehicleControl(
                    gear=GearMode.DRIVE,
                    throttle=args.throttle,
                    brake=0.0,
                    steering=float(steering),
                )
            )

            if frames % 30 == 1:
                print(
                    f"[{status.used_time:6.1f}s] "
                    f"pos=({float(pose.pos_x):7.2f},{float(pose.pos_y):7.2f}) "
                    f"yaw={float(pose.ori_z):7.2f} v={float(car.speed):5.2f} "
                    f"st={float(steering):+5.2f}",
                    flush=True,
                )

            if status.used_time >= args.max_seconds or time.time() - start > args.max_seconds + 5:
                print("到达运行时长，退出。", flush=True)
                break
    except KeyboardInterrupt:
        print("手动中断。", flush=True)
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass

    print(f"结束，共 {frames} 帧。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
