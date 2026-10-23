# -*- coding: utf-8 -*-
"""车辆模型标定：测「转角 → 曲率」的对应关系，以及倒车时的转向符号。

分四段，每段 1.5 秒，全程低速：
  1) 直行            → 检查是否有跑偏
  2) 前进 steer +0.6 → 测曲率 k+
  3) 前进 steer -0.6 → 测曲率 k-
  4) 倒车 steer +0.6 → 测倒车曲率符号

输出直接给出：k_per_steer（每单位 steering 对应多少曲率），
泊车时用 k = k_per_steer * steering 反解需要的转角。

用法：
    .venv\\Scripts\\python.exe -u tools/probe_steer.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

THROTTLE = 0.22
STEER_TEST = 0.6
SEG_SECONDS = 1.5


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def run_segment(api, msg_iter, gear, steer, seconds, label):
    """跑一段定转角，返回 (dyaw_deg, avg_speed, duration)。"""
    t0 = time.time()
    yaw0 = None
    v_sum, n = 0.0, 0
    for msg, _f in msg_iter:
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        if yaw0 is None:
            yaw0 = pose.ori_z
        v_sum += abs(v)
        n += 1
        vc = VehicleControl(gear=gear, throttle=THROTTLE, steering=steer)
        api.set_vehicle_control(vc)
        dt = time.time() - t0
        if dt >= seconds:
            dyaw = math.degrees(wrap(math.radians(-(pose.ori_z - yaw0))))
            print(f"  {label}: dyaw={dyaw:+7.1f}°  用时 {dt:.2f}s  "
                  f"平均速度 {v_sum / max(n, 1):.2f} m/s")
            return dyaw, v_sum / max(n, 1), dt
    return 0.0, 0.0, 0.0


def main() -> int:
    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    it = api.main_loop()
    # 跳过前几帧等场景稳定
    for _ in range(5):
        next(it)

    print("开始标定（每段 1.5 秒）")
    d0, v0, t0 = run_segment(api, it, GearMode.DRIVE, 0.0, SEG_SECONDS, "直行 steer=0")

    dp, vp, tp = run_segment(api, it, GearMode.DRIVE, +STEER_TEST, SEG_SECONDS, "前进 steer=+0.6")
    dn, vn, tn = run_segment(api, it, GearMode.DRIVE, -STEER_TEST, SEG_SECONDS, "前进 steer=-0.6")
    dr, vr, tr = run_segment(api, it, GearMode.REVERSE, +STEER_TEST, SEG_SECONDS, "倒车 steer=+0.6")

    print("\n=== 结论 ===")
    for name, dyaw, v, t in (("steer +0.6 前进", dp, vp, tp), ("steer -0.6 前进", dn, vn, tn)):
        if v > 0.2 and t > 0:
            k = math.radians(abs(dyaw)) / max(v * t, 1e-3)
            print(f"{name}: 曲率 {k:.4f} 1/m  → 最小转弯半径 {1 / max(k, 1e-6):.2f} m")
    if vr > 0.2 and tr > 0:
        k = math.radians(abs(dr)) / max(vr * tr, 1e-3)
        print(f"倒车 steer +0.6: dyaw={dr:+.1f}°  曲率 {k:.4f} 1/m  "
              f"（dyaw>0 表示与前进同向）")

    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
