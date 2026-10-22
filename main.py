# -*- coding: utf-8 -*-
r"""最小闭环：连上仿真场景，只读状态、不发油门。

正确用法（顺序不能反）：
    1) 另开终端运行本脚本，它会卡在 api.connect() 等待场景：
           .venv\Scripts\python.exe main.py
    2) 再在客户端/网页点“启动场景”
    3) 场景起来后，本脚本开始打印车辆状态

参数：
    --drive   匀速前进（默认只刹车停住，防止第一次就撞墙）
    --seconds 运行多少秒后自动退出（默认 20）
"""
import argparse
import math
import time

from metacar import GearMode, SceneAPI, VehicleControl


def fmt(v: float) -> str:
    return f"{v:8.2f}"


def main() -> int:
    parser = argparse.ArgumentParser(description="metacar 最小闭环")
    parser.add_argument("--drive", action="store_true", help="给油前进（默认刹停）")
    parser.add_argument("--throttle", type=float, default=0.35, help="油门 0~1")
    parser.add_argument("--seconds", type=float, default=20.0, help="运行时长上限")
    args = parser.parse_args()

    print("连接仿真场景（要先开场景，这里会一直等）...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    print(f"路线点: {len(static.route)}  子场景: {[s.name for s in static.sub_scenes]}")
    print(f"VLA 场景: {'是' if static.vla_extension else '否'}")

    start = time.time()
    frames = 0

    for sim_car_msg, camera_frames in api.main_loop():
        frames += 1
        car = sim_car_msg.main_vehicle
        pose = sim_car_msg.pose_gnss
        status = sim_car_msg.scene_status

        if frames % 10 == 1:
            speed_kmh = car.speed * 3.6
            print(
                f"[{status.used_time:6.1f}s] "
                f"pos=({fmt(pose.pos_x)},{fmt(pose.pos_y)}) "
                f"yaw={fmt(pose.ori_z)} "
                f"v={fmt(car.speed)}m/s({speed_kmh:5.1f}km/h) "
                f"frames={frames} cams={len(camera_frames)}"
            )

        vc = VehicleControl()
        if args.drive:
            vc.gear = GearMode.DRIVE
            vc.throttle = max(0.0, min(1.0, args.throttle))
        else:
            vc.gear = GearMode.PARKING
            vc.brake = 1.0
        api.set_vehicle_control(vc)

        if time.time() - start > args.seconds:
            print("到达运行时长，退出。")
            break

    # 退出前刹停
    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

