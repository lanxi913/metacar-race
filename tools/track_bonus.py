# -*- coding: utf-8 -*-
"""拿到 110 分的那版循迹控制器（直角转弯 21:01、21:03 两次验证）。

特点：**没有任何额外的平滑/限幅逻辑**，只有：
  1. 纯跟踪（前视 5 m）
  2. 弯道预判限速（跨 5 点看转角：>45° 1.6、>25° 2.2、>10° 3.0 m/s）
  3. 油门/刹车斜率限制（0.0015 / 0.004 每帧）
  4. 终点前 3 m 线性减速

后来加的 vt 变化率限制、方向盘限速、起步软起、加速度反馈，反而把奖励分弄丢了，
所以单独保留这一份「已验证 110」的版本。

用法：
    .venv\\Scripts\\python.exe -u tools/track_bonus.py
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

LOOKAHEAD = 5.0
SLOW_LOOKAHEAD = 12.0
MAX_SECONDS = 200


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    return math.radians(-yaw_deg)


def speed_cmd(v, vt, thr_max=0.45, brk_max=0.30):
    err = vt - abs(v)
    if abs(err) < 0.05:
        return 0.0, 0.0
    if err > 0:
        return min(thr_max, 0.02 + 0.12 * err), 0.0
    return 0.0, min(brk_max, -0.15 * err)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cruise", type=float, default=4.0)
    args = ap.parse_args()

    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    route = [(p.x, p.y) for p in static.route]
    print(f"路线点 {len(route)}  子场景 {[s.name for s in static.sub_scenes]}")

    curv = [0.0] * len(route)
    for i in range(1, len(route) - 1):
        a = math.atan2(route[i][1] - route[i - 1][1], route[i][0] - route[i - 1][0])
        b = math.atan2(route[i + 1][1] - route[i][1], route[i + 1][0] - route[i][0])
        curv[i] = abs(wrap(b - a))

    idx = 0
    t0 = time.time()
    last = 0.0
    lim = {"thr": 0.0, "brk": 0.0}
    done = False

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
        dt = min(0.10, max(now - t_prev, 0.02)) if 't_prev' in dir() else 0.05
        t_prev = now

        while idx < len(route) - 1:
            if math.hypot(route[idx][0] - pose.pos_x, route[idx][1] - pose.pos_y) < 4.0:
                idx += 1
            else:
                break

        tx, ty = route[min(idx, len(route) - 1)]
        for j in range(idx, len(route)):
            if math.hypot(route[j][0] - pose.pos_x, route[j][1] - pose.pos_y) >= LOOKAHEAD:
                tx, ty = route[j]
                break

        dx, dy = tx - pose.pos_x, ty - pose.pos_y
        d = max(math.hypot(dx, dy), 1e-3)
        yaw = heading_rad(pose.ori_z)
        lx = dx * math.cos(yaw) + dy * math.sin(yaw)
        ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
        if lx < 0:
            delta = 0.6 if ly > 0 else -0.6
        else:
            delta = math.atan(2.0 * ly / d)
        steer = max(-1.0, min(1.0, -delta / 0.6))

        max_turn = 0.0
        acc_len = 0.0
        for j in range(idx, len(route)):
            if j > 0:
                acc_len += math.hypot(route[j][0] - route[j - 1][0], route[j][1] - route[j - 1][1])
            max_turn = max(max_turn, curv[j])
            if acc_len > SLOW_LOOKAHEAD:
                break

        vt = args.cruise
        if max_turn > math.radians(45):
            vt = min(vt, 1.6)
        elif max_turn > math.radians(25):
            vt = min(vt, 2.2)
        elif max_turn > math.radians(10):
            vt = min(vt, 3.0)
        if abs(steer) > 0.6:
            vt = min(vt, 1.8)

        dist_left = math.hypot(route[-1][0] - pose.pos_x, route[-1][1] - pose.pos_y)
        if dist_left < 3.0:
            if not done:
                done = True
                print(f"接近终点 3 m，开始减速（剩余 {dist_left:.1f} m）")
            vt = min(vt, max(0.4, 0.6 * dist_left))

        thr, brk = speed_cmd(v, vt)

        # 起步：油门上限随时间线性放开，避免起步加速度超标
        if now - t0 < 3.0:
            thr = min(thr, 0.002 + 0.006 * (now - t0))
            brk = 0.0
        send(VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer))

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"v={v:5.2f} vt={vt:4.1f} steer={steer:+5.2f} idx={idx} left={dist_left:5.1f}")

        if dist_left < 0.8 and abs(v) < 0.3:
            print(f"到达终点，用时 {now - t0:.1f}s")
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
