# -*- coding: utf-8 -*-
r"""垂直泊车 · 基线模板（目标 200 分）—— 目前是「骨架 + 倒车入库策略」，尚未标定

背景（2026-10-23 第一轮实测，lanxi913 组）：
    前进左转 90° 进车位失败：车道到车位中心只有 5.6 m，车头顶到车位边线，
    速度归零。转弯半径至少要 4 m 才安全，5.6 m 横向距离不够。
    改用倒车入库：先开过车位、停在前方 4.5 m，再挂 R 挡把车尾甩进去，
    后轮轨迹半径小，5.6 m 横向距离够用。

停稳判据（待实测标定）：
    中心误差 < 0.30 m，航向误差 < 10°，速度 < 0.3 m/s 保持 0.8 s

本文件提供：
    1) 连接场景、读静态数据（泊位/路线不写死，全部从场景读）
    2) 倒车入库的状态机骨架（STAGE_TO_STAGING → REVERSE → SETTLE）
    3) 打印每个阶段的位置/航向误差，方便照着调参

还没做的（需要真机标定，别照着跑满分局）：
    staging 距离、倒车油门、倒车打舵角、何时回正 —— 都还是占位值。

用法：
    .venv\Scripts\python.exe -u templates\parking\zyyxnjydhcg\run.py --dry-run
    --dry-run 只打印阶段与误差，不提交最终控制（安全调试用）
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

# ---- 待标定参数（改这里，注释写清每个数的来历）----
STAGING_AHEAD_M = 4.5        # 车位前方多远处停车准备倒车（lanxi913 给的初值）
REVERSE_THROTTLE = 0.01      # 倒车油门，先按停止题的安全值来
REVERSE_STEER = 0.35         # 倒车打舵（正负取决于车位在左还是右，运行时按偏差决定）
SETTLE_SPEED_MS = 0.30       # 判定静止的速度阈值
SETTLE_SECONDS = 0.80        # 低速保持多久算停稳
CENTER_TOL_M = 0.30          # 中心误差要求
YAW_TOL_DEG = 10.0           # 航向误差要求


class Stage(Enum):
    TO_STAGING = "开到车位前方"
    REVERSE = "倒车入库"
    SETTLE = "停稳保持"
    DONE = "完成"


@dataclass
class Berth:
    """车位目标点与朝向（从场景静态数据里取，不写死）。"""

    x: float
    y: float
    yaw_deg: float


def pick_berth(static) -> Berth | None:
    """从静态数据里找车位目标点。

    真实场景里泊位信息一般在 sub_scenes 的 end_point / 或 route 末端；
    这里先取最后一个子场景的 end_point 兜底，接场景后按实际字段改。
    """
    if not static.sub_scenes:
        return None
    end = getattr(static.sub_scenes[-1], "end_point", None)
    if end is None:
        return None
    return Berth(float(end.x), float(end.y), float(getattr(end, "ori_z", 0.0)))


def wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def main() -> int:
    parser = argparse.ArgumentParser(description="垂直泊车（倒车入库）骨架")
    parser.add_argument("--dry-run", action="store_true", help="只打印误差，不真的提交控制")
    parser.add_argument("--max-seconds", type=float, default=200.0)
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    berth = pick_berth(static)
    print(f"子场景: {[s.name for s in static.sub_scenes]}  车位: {berth}", flush=True)
    if berth is None:
        print("没读到车位目标点：接上真实场景后按 static 里的字段补 pick_berth()。", flush=True)
        return 1

    stage = Stage.TO_STAGING
    settle_seconds = 0.0
    frames = 0
    try:
        for msg, _camera_frames in api.main_loop():
            frames += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            x, y = float(pose.pos_x), float(pose.pos_y)
            yaw = float(pose.ori_z)
            speed = float(car.speed)

            # 场景 yaw 顺时针为正：误差用 wrap 后的角度
            yaw_err = wrap_deg(berth.yaw_deg - yaw) if berth.yaw_deg else 0.0
            dist = math.hypot(berth.x - x, berth.y - y)

            if stage is Stage.TO_STAGING and dist <= STAGING_AHEAD_M:
                stage = Stage.REVERSE
            elif stage is Stage.REVERSE and abs(speed) <= SETTLE_SPEED_MS and dist <= CENTER_TOL_M:
                stage = Stage.SETTLE
                settle_seconds = 0.0

            if stage is Stage.SETTLE:
                settle_seconds += 1.0 / 30.0
                if settle_seconds >= SETTLE_SECONDS:
                    stage = Stage.DONE

            if stage is Stage.DONE:
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
            elif stage is Stage.TO_STAGING:
                # 前进阶段：低速直行到车位前方（转向留给下一版标定）
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.01, steering=0.0)
            elif stage is Stage.REVERSE:
                # 倒车阶段：R 挡 + 小油门 + 打舵（方向随航向误差符号）
                steer = REVERSE_STEER if yaw_err >= 0 else -REVERSE_STEER
                vc = VehicleControl(
                    gear=GearMode.REVERSE if hasattr(GearMode, "REVERSE") else GearMode.DRIVE,
                    throttle=REVERSE_THROTTLE,
                    steering=steer,
                )
            else:
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)

            if frames % 15 == 1:
                print(
                    f"[{status.used_time:6.1f}s] {stage.value:10s} "
                    f"pos=({x:7.2f},{y:7.2f}) yaw_err={yaw_err:+6.1f}° "
                    f"dist={dist:5.2f} v={speed:5.2f}",
                    flush=True,
                )

            if not args.dry_run:
                api.set_vehicle_control(vc)

            if status.used_time >= args.max_seconds:
                break
    except KeyboardInterrupt:
        print("手动中断。", flush=True)
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass

    print(f"结束于 {stage.value}，共 {frames} 帧。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
