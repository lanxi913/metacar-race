# -*- coding: utf-8 -*-
"""刹车标定：先加速到 ~4.8 m/s，再分段施加不同刹车量，测"刹车→减速度"曲线。

用途：复杂路况超车收尾要把车从 4.78 m/s 平稳刹到 0，且 |a| ≤ 5。
已知仿真刹车 <0.05 几乎无效果、0.12 却有 13 m/s²，映射非线性，必须实测。
"""
from __future__ import annotations

import time

from metacar import GearMode, SceneAPI, VehicleControl

STEPS = [0.10, 0.20, 0.35, 0.50]
DWELL = 2.0


def main() -> int:
    print("等待场景连接（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。先加速到 4.8 m/s", flush=True)

    t0 = None
    phase = "accel"
    idx = -1
    t_step = None
    last_print = 0.0
    v_enter = 0.0
    try:
        for msg, _cam in api.main_loop():
            v = float(msg.main_vehicle.speed)
            st = msg.scene_status
            now = time.time()
            if t0 is None:
                t0 = now
            el = now - t0
            if phase == "accel":
                api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=0.10))
                if v > 4.5:
                    phase, t_step = "brake", now
                    v_enter = v
                    print(f"到达 {v:.2f} m/s，开始刹车台阶", flush=True)
            else:
                since = now - (t_step or now)
                want = int(since // DWELL)
                if want != idx and want < len(STEPS):
                    idx = want
                    print(f"== 台阶 {idx + 1}/{len(STEPS)}：brake {STEPS[idx]:.2f} ==", flush=True)
                if idx >= len(STEPS):
                    print("刹车台阶完成", flush=True)
                    break
                api.set_vehicle_control(
                    VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=STEPS[idx]))
                if now - last_print > 0.4:
                    last_print = now
                    print(f"  thr=0 brk={STEPS[idx]:.2f} v={v:5.2f} m/s", flush=True)
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.0))
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
