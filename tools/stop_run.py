# -*- coding: utf-8 -*-
"""车辆停止（场景元素 02）：平滑巡航 + 提前线性减速，目标 110 分。

评分：停在指定位置基础 100 分，|a| <= 5 m/s² 再加 10 分 = 110。
关键是**减速要早、刹车要轻**，一脚踩死必定丢分。

策略：
    d_end 远        → 巡航（油门斜坡升到 0.35）
    d_end 变近      → 目标速度 v_t = sqrt(2*1.2*(d_end-0.15))，连续速度环跟
    d_end <= 0.15   → 轻刹停稳，P 挡保持 5 秒

用法：
    .venv\\Scripts\\python.exe -u tools/stop_run.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

CRUISE_THROTTLE = 0.35
CRUISE_SPEED = 4.0
DECEL = 0.7
STOP_TOL = 0.03
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


def speed_cmd(v_meas, v_target, thr_max=0.35, brk_max=0.12):
    err = v_target - abs(v_meas)
    if abs(err) < 0.05:
        return 0.0, 0.0
    if err > 0:
        return min(thr_max, 0.02 + 0.10 * err), 0.0
    return 0.0, min(brk_max, -0.08 * err)


def main() -> int:
    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    route = list(static.route)
    print(f"路线点 {len(route)}  子场景 {[s.name for s in static.sub_scenes]}")

    idx = 0
    t0 = time.time()
    hold_t = None
    last = 0.0
    v_prev = 0.0
    t_prev = t0
    peak1s = 0.0
    peak100 = 0.0
    peak_raw = 0.0
    v_prev_ = 0.0
    hist = []
    lim = {"thr": 0.0, "brk": 0.0}

    def send(vc):
        lim["thr"] += max(-0.0015, min(0.0015, vc.throttle - lim["thr"]))
        lim["brk"] += max(-0.004, min(0.004, vc.brake - lim["brk"]))
        lim["thr"] = max(0.0, min(1.0, lim["thr"]))
        lim["brk"] = max(0.0, min(1.0, lim["brk"]))
        vc.throttle, vc.brake = lim["thr"], lim["brk"]
        api.set_vehicle_control(vc)

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()
        dt = min(0.10, max(now - t_prev, 0.02))
        t_prev = now
        a_raw = (v - v_prev) / dt
        v_prev_ = v_prev
        v_prev = v
        peak_raw = max(peak_raw, abs(a_raw))
        if abs(a_raw) > 5.0:
            print(f"!! 尖峰 |a|={abs(a_raw):5.1f} dt={dt:.3f} v_prev={v_prev_:+.2f} v={v:+.2f} d_end={d_end if 'd_end' in dir() else -1:.2f}")
        hist.append((now, v))
        while len(hist) > 1 and now - hist[0][0] > 1.2:
            hist.pop(0)
        if hist and now - hist[0][0] >= 0.9:
            peak1s = max(peak1s, abs((v - hist[0][1]) / (now - hist[0][0])))
        if hist and now - hist[0][0] >= 0.08:
            peak100 = max(peak100, abs((v - hist[0][1]) / (now - hist[0][0])))

        end_point = msg.scene_status.end_point
        if end_point is None:
            send(VehicleControl(gear=GearMode.PARKING, brake=0.2))
            continue

        d_end = math.hypot(end_point.x - pose.pos_x, end_point.y - pose.pos_y)

        while idx < len(route) - 1:
            p = route[idx]
            if math.hypot(p.x - pose.pos_x, p.y - pose.pos_y) < 4.0:
                idx += 1
            else:
                break

        tx, ty = route[min(idx, len(route) - 1)].x, route[min(idx, len(route) - 1)].y
        for j in range(idx, len(route)):
            if math.hypot(route[j].x - pose.pos_x, route[j].y - pose.pos_y) >= 5.0:
                tx, ty = route[j].x, route[j].y
                break
        err_h = wrap(math.atan2(ty - pose.pos_y, tx - pose.pos_x) - heading_rad(pose.ori_z))
        steer = max(-1.0, min(1.0, -1.6 * err_h))

        if d_end > STOP_TOL:
            v_t = min(CRUISE_SPEED, math.sqrt(max(2 * DECEL * max(d_end - STOP_TOL, 0.005), 0.0)))
            thr, brk = speed_cmd(v, v_t, thr_max=CRUISE_THROTTLE)
            send(VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer))
        else:
            if hold_t is None:
                hold_t = now
                print(f"到位：距目标 {d_end:.2f} m，速度 {v:.2f} m/s，用时 {now - t0:.1f}s")
            ramp = min(1.0, 0.20 + 0.50 * (now - hold_t))
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=ramp))

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"v={v:5.2f} d_end={d_end:6.2f} thr={lim['thr']:4.2f} brk={lim['brk']:4.2f}")

        if hold_t is not None and now - hold_t > HOLD_SECONDS:
            print(f"加速度峰值：逐帧 {peak_raw:.2f}，0.1s {peak100:.2f}，1s {peak1s:.2f} m/s2")
            print(f"完成：距目标 {d_end:.2f} m，用时 {now - t0:.1f}s")
            break

        if now - t0 > MAX_SECONDS:
            print("超时")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.3))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
