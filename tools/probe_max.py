# -*- coding: utf-8 -*-
"""测满舵转弯半径：steer=±1.0，前进 1.5 秒。

用法：
    .venv\\Scripts\\python.exe -u tools/probe_max.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

THROTTLE = 0.20


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...")
    api.connect()
    print("已连接。")

    it = api.main_loop()
    for _ in range(5):
        next(it)

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
            api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=THROTTLE, steering=steer))
            dt = time.time() - t0
            if dt >= 1.5:
                dyaw = -(pose.ori_z - yaw0)
                # 统一到 [-180,180]
                dyaw = (dyaw + 180) % 360 - 180
                vavg = vs / max(n, 1)
                k = math.radians(abs(dyaw)) / max(vavg * dt, 1e-3)
                print(f"steer={steer:+.1f}  dyaw={dyaw:+7.1f}°  v={vavg:.2f} m/s  "
                      f"曲率 {k:.4f} 1/m  → R={1 / max(k, 1e-6):.2f} m")
                break

    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
