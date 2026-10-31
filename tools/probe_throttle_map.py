# -*- coding: utf-8 -*-
"""油门台阶探针：固定油门 4 s 一档，测这台车模型的"油门→稳态车速"曲线。

用途：复杂路况超车限时 30 s、路线 146.9 m，必须平均 ≥4.9 m/s；
但历史日志显示 4.78 m/s 处像是有个平台，再加油就跳到 13+。
本脚本用台阶把 0.10~0.30 每档跑 4 s，看是否存在稳定的 5~10 m/s 区间。
"""
from __future__ import annotations

import time

from metacar import GearMode, SceneAPI, VehicleControl

STEPS = [0.10, 0.14, 0.18, 0.22, 0.26, 0.30]
DWELL = 4.0          # 每档保持秒数


def main() -> int:
    print("等待场景连接（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。开始油门台阶测试", flush=True)

    t0 = None
    idx = -1
    t_step = None
    last_print = 0.0
    try:
        for msg, _cam in api.main_loop():
            pose = msg.pose_gnss
            v = float(msg.main_vehicle.speed)
            st = msg.scene_status
            now = time.time()
            if t0 is None:
                t0 = now
            el = now - t0
            want = int(el // DWELL)
            if want != idx and want < len(STEPS):
                idx = want
                t_step = now
                print(f"== 台阶 {idx + 1}/{len(STEPS)}：油门 {STEPS[idx]:.2f} ==", flush=True)
            if idx >= len(STEPS):
                print("全部台阶完成", flush=True)
                break
            thr = STEPS[idx]
            # 每档开头给 0.6 s 让油门斜坡跟上（避免阶跃本身造成尖峰）
            since = now - (t_step or now)
            thr_eff = thr * min(1.0, since / 0.6)
            api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=thr_eff))
            if now - last_print > 0.5:
                last_print = now
                print(f"  t={since:4.1f}s thr={thr_eff:.3f} v={v:5.2f} m/s "
                      f"pos=({float(pose.pos_x):6.1f},{float(pose.pos_y):7.1f})", flush=True)
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
