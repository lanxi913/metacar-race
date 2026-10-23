# -*- coding: utf-8 -*-
"""倒车运动学标定：静止起步，倒车满舵各 1.5 秒，测倒车半径与方向。

用法：
    .venv\\Scripts\\python.exe -u tools/probe_reverse.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

THROTTLE = 0.13


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...")
    api.connect()
    print("已连接。")
    it = api.main_loop()
    for _ in range(5):
        next(it)

    # 先倒车直行 0.8 秒，让车动起来
    t0 = time.time()
    for msg, _f in it:
        api.set_vehicle_control(VehicleControl(gear=GearMode.REVERSE, throttle=THROTTLE, steering=0.0))
        if time.time() - t0 > 0.8:
            break

    for steer in (+1.0, -1.0):
        t0 = time.time()
        yaw0 = None
        vs, n = 0.0, 0
        for msg, _f in it:
            pose = msg.pose_gnss
            v = msg.main_vehicle.speed
            if yaw0 is None:
                yaw0 = pose.ori_z
            vs += abs(v)
            n += 1
            api.set_vehicle_control(VehicleControl(gear=GearMode.REVERSE,
                                                   throttle=THROTTLE, steering=steer))
            dt = time.time() - t0
            if dt >= 1.5:
                dyaw = -(pose.ori_z - yaw0)          # 数学角变化
                dyaw = (dyaw + 180) % 360 - 180
                vavg = vs / max(n, 1)
                k = math.radians(abs(dyaw)) / max(vavg * dt, 1e-3)
                r = 1 / max(k, 1e-6)
                print(f"倒车 steer={steer:+.1f}  dyaw={dyaw:+7.1f}°  v={vavg:.2f} m/s  "
                      f"曲率 {k:.4f}  R={r:.2f} m")
                break

    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
