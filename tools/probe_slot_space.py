# -*- coding: utf-8 -*-
"""斜坡泊车·边界探测：先用泊车控制器开到"卡住点"，再向六个方向各推 2 s，
量出每个方向实际能走多远（= 该方向还剩多少可行驶空间）。

全部在同一个 main_loop 里用状态机推进，避免反复重建生成器。
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.slot_reverse import SlotReverseParker  # noqa: E402

PROBES = [
    ("前 steer=0",    GearMode.DRIVE, 0.0, 2.0),
    ("后 steer=0",    GearMode.REVERSE, 0.0, 2.0),
    ("前 steer=+0.6", GearMode.DRIVE, 0.6, 2.0),
    ("前 steer=-0.6", GearMode.DRIVE, -0.6, 2.0),
    ("后 steer=+0.6", GearMode.REVERSE, 0.6, 2.0),
    ("后 steer=-0.6", GearMode.REVERSE, -0.6, 2.0),
]


def main() -> int:
    print("等待场景连接（先跑本脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)
    ctrl = SlotReverseParker(slope_thr=0.08, slope_brake=0.35)

    phase = "APPROACH"
    t_phase = time.time()
    stuck_t = 0.0
    idx = 0
    start = (0.0, 0.0)
    vmax = 0.0
    try:
        for msg, _cam in api.main_loop():
            now = time.time()
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            x, y = float(pose.pos_x), float(pose.pos_y)

            if phase == "APPROACH":
                vc, info = ctrl.update(pose, float(car.speed), msg.obstacles, 1 / 30)
                api.set_vehicle_control(vc)
                stuck_t = stuck_t + 1 / 30 if abs(float(car.speed)) < 0.05 else 0.0
                if (stuck_t > 1.2 and info.phase == "REVERSE") or st.used_time > 26.0:
                    print(f"[卡住点] ({x:.2f},{y:.2f}) yaw={float(pose.ori_z):.1f} "
                          f"phase={info.phase} v={float(car.speed):.2f}", flush=True)
                    phase, t_phase = "PROBE", now
                    start, vmax = (x, y), 0.0
            elif phase == "PROBE":
                name, gear, steer, dur = PROBES[idx]
                api.set_vehicle_control(
                    VehicleControl(gear=gear, throttle=0.18, steering=steer))
                vmax = max(vmax, abs(float(car.speed)))
                if now - t_phase > dur:
                    dx, dy = x - start[0], y - start[1]
                    print(f"{name:14s} 起点({start[0]:6.2f},{start[1]:6.2f}) "
                          f"Δ=({dx:+.2f},{dy:+.2f}) |Δ|={math.hypot(dx, dy):4.2f} "
                          f"vmax={vmax:.2f}", flush=True)
                    idx += 1
                    if idx >= len(PROBES):
                        print("探测完成", flush=True)
                        break
                    phase, t_phase = "STOP", now
            elif phase == "STOP":
                api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, brake=0.45))
                if now - t_phase > 1.2:
                    phase, t_phase = "PROBE", now
                    start, vmax = (x, y), 0.0
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.40))
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
