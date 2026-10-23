# -*- coding: utf-8 -*-
"""车辆停止（场景元素 02）：巡航到终点线前精确刹停。

评分要点：
  - 停稳在指定位置：误差 e 米扣 10e 分
  - 压线每次 −10
  - 碰撞 5×v
  - 100 秒超时判 0

策略：沿 route 纯跟踪巡航，接近 end_point 时按剩余距离反推目标速度并刹车，
停稳后挂 P 挡保持 5 秒。

用法：
    .venv\\Scripts\\python.exe -u tools/stop_run.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

CRUISE_THROTTLE = 0.45
SLOW_DIST = 14.0        # 进入减速区
STOP_TOL = 0.15         # 认为到位的距离
HOLD_SECONDS = 5.0
MAX_SECONDS = 90


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    return math.radians(-yaw_deg)


def main() -> int:
    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    route = list(static.route)
    print(f"路线点 {len(route)}  子场景 {[s.name for s in static.sub_scenes]}")

    if not route:
        print("没有路线点，退出")
        return 1

    idx = 0
    t0 = time.time()
    hold_t = None
    last = 0.0
    end_point = None

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        status = msg.scene_status
        now = time.time()

        if status.end_point is not None:
            end_point = status.end_point
        if end_point is None:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            continue

        d_end = math.hypot(end_point.x - pose.pos_x, end_point.y - pose.pos_y)

        # 推进路点
        while idx < len(route) - 1:
            p = route[idx]
            if math.hypot(p.x - pose.pos_x, p.y - pose.pos_y) < 4.0:
                idx += 1
            else:
                break

        # 转向：朝前视点
        tx, ty = route[min(idx, len(route) - 1)].x, route[min(idx, len(route) - 1)].y
        for j in range(idx, len(route)):
            if math.hypot(route[j].x - pose.pos_x, route[j].y - pose.pos_y) >= 5.0:
                tx, ty = route[j].x, route[j].y
                break
        err = wrap(math.atan2(ty - pose.pos_y, tx - pose.pos_x) - heading_rad(pose.ori_z))
        steer = max(-1.0, min(1.0, -1.6 * err))

        vc = VehicleControl()
        vc.steering = steer

        if d_end > SLOW_DIST:
            vc.gear = GearMode.DRIVE
            vc.throttle = CRUISE_THROTTLE
        elif v - 0.05 > max(0.0, 0.9 * d_end) and d_end > STOP_TOL:
            # 需要减速：按剩余距离反推目标速度
            vc.gear = GearMode.DRIVE
            vc.throttle = 0.0
            vc.brake = min(1.0, 0.35 + 0.35 * (v - 0.9 * d_end))
        elif d_end > STOP_TOL:
            # 低速爬行对准
            vc.gear = GearMode.DRIVE
            vc.throttle = 0.06
            vc.brake = 0.0
        else:
            if hold_t is None:
                hold_t = now
                print(f"到位：距目标 {d_end:.2f} m，速度 {v:.2f} m/s，用时 {now - t0:.1f}s")
            vc.gear = GearMode.PARKING
            vc.throttle = 0.0
            vc.brake = 1.0

        api.set_vehicle_control(vc)

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={v:5.2f} d_end={d_end:6.2f} "
                  f"thr={vc.throttle:4.2f} brk={vc.brake:4.2f} steer={steer:+5.2f}")

        if hold_t is not None and now - hold_t > HOLD_SECONDS:
            print(f"保持 {HOLD_SECONDS:.0f} 秒完成，最终距目标 {d_end:.3f} m，"
                  f"总用时 {now - t0:.1f}s")
            break

        if now - t0 > MAX_SECONDS:
            print("超时退出")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
