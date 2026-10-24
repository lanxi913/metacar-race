# -*- coding: utf-8 -*-
r"""「车辆直行」全分跑分入口。

与根目录 main.py 相互独立：本脚本只做直行场景，目标是拿满 110 分
（到达终点 100 + 全程 |a| ≤ 5 m/s² 加 10）。

用法（顺序不能反）：
    1) 另开终端：.venv\Scripts\python.exe -u straight_full\main.py
    2) 客户端点「启动场景」
    3) 看终端刷 pos/v，跑完会打一行 SUMMARY

参数：
    --speed       巡航目标速度 m/s（默认 10，限速 60 km/h 留足余量）
    --accel       加速度上限 m/s²（默认 2.0，远低于 5 的红线）
    --seconds     运行时长上限（默认 120）
    --no-kick     禁用低挡位升挡 kick
    --steer-sign  转向符号，实测为 -1；若车往反方向偏就改这个
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

from metacar import GearMode, SceneAPI, VehicleControl

# 允许直接以脚本方式运行（python straight_full\main.py）
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from straight_full.controller import StraightDriver  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="车辆直行全分跑分")
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--accel", type=float, default=2.0)
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--no-kick", action="store_true")
    parser.add_argument("--cruise", type=float, default=None,
                        help="定油门巡航（例如 0.5）：不做速度闭环，油门单调斜坡，加速度最稳")
    parser.add_argument("--cruise-rate", type=float, default=0.6, help="巡航油门的每秒变化上限")
    parser.add_argument("--no-end-brake", action="store_true",
                        help="终点前不减速（元素题冲线即结束，刹车容易造成加速度毛刺）")
    parser.add_argument("--steer-sign", type=float, default=-1.0)
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    route = list(static.route)
    print(f"路线点: {len(route)}  子场景: {[s.name for s in static.sub_scenes]}", flush=True)
    if len(route) < 2:
        print("route 点不足，退出。", flush=True)
        return 1

    driver = StraightDriver(
        route,
        v_max=args.speed,
        accel_limit=args.accel,
        steer_sign=args.steer_sign,
        allow_kick=not args.no_kick,
        cruise=args.cruise,
        cruise_rate=args.cruise_rate,
    )
    if args.no_end_brake:
        # 把刹车曲线退化成「不减速」：规划器只受 v_max 约束
        driver.planner.a_down = 1e-6
    print(
        f"控制参数: v_max={args.speed} a_max={args.accel} "
        f"kick={'off' if args.no_kick else 'on'} steer_sign={args.steer_sign}",
        flush=True,
    )
    print(f"路径总长: {driver.path.total:.1f} m", flush=True)

    frames = 0
    v_prev = 0.0
    t_prev = None
    a_max_abs = 0.0
    v_peak = 0.0
    t_start = time.time()
    last_s = 0.0
    s_left = driver.path.total
    ending = "unknown"

    # 逐帧落盘，方便事后核对加速度到底在哪里超限
    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_f = (log_dir / f"frames_{stamp}.csv").open("w", newline="", encoding="utf-8")
    csv_w = csv.writer(csv_f)
    csv_w.writerow(["frame", "t", "x", "y", "yaw", "v", "a", "throttle", "brake", "steer", "s_left"])

    try:
        for msg, camera_frames in api.main_loop():
            frames += 1
            car = msg.main_vehicle
            pose = msg.pose_gnss
            status = msg.scene_status
            v = float(car.speed)

            # 用仿真时间算真实加速度（30fps 下比墙钟更准）
            if t_prev is None:
                dt = 1.0 / 30.0
            else:
                dt = max(status.used_time - t_prev, 1e-3)
            t_prev = status.used_time

            a_inst = 0.0
            # 仿真时间偶尔会跳变（换挡/掉帧），dt 太小的样本不可信，直接跳过
            if frames > 2 and dt >= 0.015:
                a_inst = (v - v_prev) / dt
                a_max_abs = max(a_max_abs, abs(a_inst))
            v_prev = v
            v_peak = max(v_peak, v)

            vc, info = driver.step(pose.pos_x, pose.pos_y, pose.ori_z, v, dt)
            s_left = info["s_left"]
            last_s = info["s"]

            api.set_vehicle_control(vc)
            csv_w.writerow([
                frames, round(status.used_time, 4), round(pose.pos_x, 4), round(pose.pos_y, 4),
                round(pose.ori_z, 3), round(v, 4), round(a_inst, 4),
                round(vc.throttle, 4), round(vc.brake, 4), round(vc.steering, 4),
                round(s_left, 3),
            ])

            if frames % 10 == 1:
                print(
                    f"[{status.used_time:6.1f}s] "
                    f"pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                    f"yaw={pose.ori_z:7.2f} "
                    f"v={v:5.2f}m/s({v * 3.6:5.1f}km/h) "
                    f"thr={vc.throttle:4.2f} brk={vc.brake:4.2f} str={vc.steering:+5.2f} "
                    f"lat={info['lat']:+5.2f} left={s_left:6.1f}m",
                    flush=True,
                )

            if time.time() - t_start > args.seconds:
                ending = "timeout"
                break
    except KeyboardInterrupt:
        ending = "interrupt"
    finally:
        try:
            csv_f.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass

    reached = s_left <= 2.0
    if ending == "unknown":
        ending = "scene_end"
    summary = {
        "ending": ending,
        "frames": frames,
        "s_total": round(driver.path.total, 2),
        "s_done": round(last_s, 2),
        "s_left": round(s_left, 2),
        "reached_end": reached,
        "v_peak_kmh": round(v_peak * 3.6, 2),
        # 注意：这是 car.speed 的差分，受仿真速度信号抖动影响，仅供参考；
        # |a| ≤ 5 的判定以平台「得分」为准（本脚本用 --cruise 实测 110 分）。
        "a_signal_max": round(a_max_abs, 3),
        "params": {
            "mode": "cruise" if args.cruise is not None else "track",
            "cruise": args.cruise,
            "v_max": args.speed,
            "a_max": args.accel,
            "kick": not args.no_kick,
            "steer_sign": args.steer_sign,
        },
    }
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)

    (log_dir / f"straight_{stamp}.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
