# -*- coding: utf-8 -*-
r"""单车入口：连接场景 → 循迹 / 匀速 / 刹停 → 打印状态。

正确用法（顺序不能反）：
    1) 另开终端运行本脚本，它会卡在 api.connect() 等待场景：
           .venv\Scripts\python.exe -u main.py
    2) 再在客户端/网页点“启动场景”
    3) 场景起来后，本脚本开始打印车辆状态

参数：
    --track     纯跟踪 + 速度环（默认只刹停，防止第一次就撞墙）
    --drive     直给油（调试用）
    --throttle  油门开度，配合 --drive
    --seconds   运行时长上限
"""
import argparse
import time

from metacar import GearMode, SceneAPI, VehicleControl

from metacar_race.control.tracking import (
    PurePursuit,
    SpeedController,
    distance_left,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="metacar 单车入口")
    parser.add_argument("--track", action="store_true", help="循迹 + 速度环")
    parser.add_argument("--drive", action="store_true", help="直接给油（调试）")
    parser.add_argument("--throttle", type=float, default=0.35, help="油门 0~1")
    parser.add_argument("--seconds", type=float, default=60.0, help="运行时长上限")
    parser.add_argument("--speed", type=float, default=None, help="循迹目标速度 m/s（默认取 config）")
    parser.add_argument("--kick", action="store_true", help="强制允许顶满油门升挡（会牺牲加速度加分）")
    args = parser.parse_args()

    print("连接仿真场景（要先开场景，这里会一直等）...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    route = list(static.route)
    print(f"路线点: {len(route)}  子场景: {[s.name for s in static.sub_scenes]}")
    print(f"VLA 场景: {'是' if static.vla_extension else '否'}")

    pursuit = PurePursuit(route) if args.track and len(route) >= 2 else None
    speed = SpeedController() if args.speed is None else SpeedController(v_target=args.speed)
    speed.reset(0.0)

    start = time.time()
    last_t = start
    v_prev = 0.0
    accel = 0.0
    frames = 0

    for sim_car_msg, camera_frames in api.main_loop():
        now = time.time()
        dt = max(now - last_t, 1e-3)
        last_t = now
        frames += 1

        car = sim_car_msg.main_vehicle
        pose = sim_car_msg.pose_gnss
        status = sim_car_msg.scene_status
        v = car.speed
        accel_raw = (v - v_prev) / dt
        v_prev = v
        accel = 0.9 * accel + 0.1 * max(-10.0, min(10.0, accel_raw))  # 平滑，瞬时值抖动太大

        vc = VehicleControl()
        left = 0.0

        if pursuit is not None:
            steering = pursuit.steering(pose.pos_x, pose.pos_y, pose.ori_z)
            left = distance_left(route, pose.pos_x, pose.pos_y, pursuit.idx)
            # 终点前按剩余距离收敛，避免冲过头
            v_ceiling = max(0.0, 0.55 * left)
            # 只有按当前速度赶不上时限时才允许升挡 kick，否则宁可慢一点保住加速度加分
            remaining = max(1.0, status.time_limit - status.used_time)
            eta = left / max(v, 1.0)
            allow_kick = args.kick or (eta > 0.55 * remaining)
            throttle, brake = speed.update(
                v, dt, v_ceiling=v_ceiling, a_now=accel, allow_kick=allow_kick
            )
            vc.gear = GearMode.DRIVE
            vc.throttle = throttle
            vc.brake = brake
            vc.steering = steering
        elif args.drive:
            vc.gear = GearMode.DRIVE
            vc.throttle = max(0.0, min(1.0, args.throttle))
        else:
            vc.gear = GearMode.PARKING
            vc.brake = 1.0

        api.set_vehicle_control(vc)

        if frames % 10 == 1:
            print(
                f"[{status.used_time:6.1f}s] "
                f"pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                f"yaw={pose.ori_z:7.2f} "
                f"v={v:5.2f}m/s({v * 3.6:5.1f}km/h) "
                f"a={accel:+5.2f} "
                f"thr={vc.throttle:4.2f} brk={vc.brake:4.2f} str={vc.steering:+5.2f} "
                f"left={left:6.1f}m"
            )

        if now - start > args.seconds:
            print("到达运行时长，退出。")
            break

    print(f"结束：共 {frames} 帧")
    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

