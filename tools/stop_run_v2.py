# -*- coding: utf-8 -*-
"""车辆停止 · 过冲保护版（不改 tools/stop_run.py，另开入口）。

stop_run.py 的问题（2026-10-25 实测）：判据 `d_end > 0.03` 就给油、否则刹车。
这车油门是开关量，最小起步就 0.3 m/s，最后一帧落在 5 cm 时它会继续往前顶；
一旦越过目标 d_end 反而变大，控制器以为"还没到"，于是一路开到撞墙：
判「车辆前撞」+「停车位置错误」，110 → 76.76。

本版三处改动：
  1. 接近段速度控制器改成"低速度-高刹车主"：v_t 压到 0.15~0.45 m/s，超速优先刹车；
  2. **过冲保护**：d_end 由减转增（连续两帧增加 >2 cm）立即认定已到位，刹车停住；
  3. 到位判据放宽到 0.12 m 并立即挂 P 保持 5 s，不再追求 3 cm 的刀尖平衡。

用法：先跑脚本，再点「车辆停止」。
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

CRUISE_THROTTLE = 0.35
CRUISE_SPEED = 4.0
ARRIVE_TOL = 0.12
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
    print("waiting for scene ...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    route = list(static.route)
    print(f"route {len(route)} sub {[s.name for s in static.sub_scenes]}", flush=True)

    idx = 0
    t0 = time.time()
    hold_t = None
    arrived_by = None
    last = 0.0
    v_prev = 0.0
    t_prev = t0
    d_hist = []
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
        v = float(msg.main_vehicle.speed)
        now = time.time()
        dt = min(0.10, max(now - t_prev, 0.02))
        t_prev = now
        v_prev = v

        end_point = msg.scene_status.end_point
        if end_point is None:
            send(VehicleControl(gear=GearMode.PARKING, brake=0.2))
            continue

        d_end = math.hypot(end_point.x - pose.pos_x, end_point.y - pose.pos_y)
        d_hist.append(d_end)
        if len(d_hist) > 4:
            d_hist.pop(0)

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

        # 过冲检测：距离由减转增（连续两帧都变大），说明已经越过目标
        overshoot = (len(d_hist) >= 3 and d_hist[-1] > d_hist[-2] + 0.02
                     and d_hist[-2] > d_hist[-3] + 0.005)

        if hold_t is None:
            if d_end <= ARRIVE_TOL:
                hold_t, arrived_by = now, f"到位 {d_end:.2f} m"
            elif overshoot:
                hold_t, arrived_by = now, f"过冲保护 {d_end:.2f} m"
            else:
                if d_end > 2.5:
                    v_t = CRUISE_SPEED
                else:
                    # 低速接近：0.15~0.45 m/s，避免掉进 0.5~1.6 m/s 的速度台阶带
                    v_t = max(0.15, min(0.45, 0.55 * d_end))
                err = v_t - abs(v)
                if err > 0.05:
                    thr, brk = min(CRUISE_THROTTLE, 0.01 + 0.08 * err), 0.0
                elif err < -0.05:
                    thr, brk = 0.0, min(0.25, -0.10 * err + 0.03)
                else:
                    thr, brk = 0.0, 0.0
                # 距离很近时刹车优先，宁停勿顶
                if d_end < 0.45 and v > 0.30:
                    thr, brk = 0.0, 0.12
                send(VehicleControl(gear=GearMode.DRIVE, throttle=float(thr),
                                    brake=float(brk), steering=float(steer)))
        else:
            ramp = min(1.0, 0.25 + 0.55 * (now - hold_t))
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=ramp))

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) v={v:5.2f} "
                  f"d_end={d_end:6.2f} thr={lim['thr']:4.2f} brk={lim['brk']:4.2f}", flush=True)

        if hold_t is not None and now - hold_t > HOLD_SECONDS:
            print(f"完成：{arrived_by}，最终 d_end={d_end:.3f} m，用时 {now - t0:.1f}s", flush=True)
            break
        if now - t0 > MAX_SECONDS:
            print("timeout", flush=True)
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.3))
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
