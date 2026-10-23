# -*- coding: utf-8 -*-
"""通用循迹：带弯道预判限速 + 加速度限制，适用于直角转弯/U 型弯/环形/连续转弯。

要点：
  1. 纯跟踪横向控制（前视 5 m，steering 取负号——场景转向符号与纯跟踪相反）
  2. **预判限速**：看前方 12 m 路线点的最大转角，提前把目标速度降下来
  3. 速度环连续控制 + 油门/刹车斜率限制（0.0015 / 0.004 每帧），保证 |a| 小

用法：
    .venv\\Scripts\\python.exe -u tools/track_run.py [--cruise 4.0]
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

LOOKAHEAD = 5.0
SLOW_LOOKAHEAD = 12.0     # 预判距离
MAX_SECONDS = 180


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
    if len(route) < 3:
        print("路线点太少")
        return 1

    # 预计算每个点的局部转角（前后各 2 个点）
    # 用跨 5 个点的方向变化估计「这一段有多弯」，避免采样过密导致角度被摊薄
    SPAN = 5
    curv = [0.0] * len(route)
    for i in range(len(route)):
        j0 = max(0, i - SPAN)
        j1 = min(len(route) - 1, i + SPAN)
        if j1 - i < 1 or i - j0 < 1:
            continue
        a = math.atan2(route[i][1] - route[j0][1], route[i][0] - route[j0][0])
        b = math.atan2(route[j1][1] - route[i][1], route[j1][0] - route[i][0])
        curv[i] = abs(wrap(b - a))
    print(f"最大局部转角 {math.degrees(max(curv)):.1f}°  点 {curv.index(max(curv))}")

    idx = 0
    t0 = time.time()
    last = 0.0
    v_prev = 0.0
    t_prev = t0
    peak1s = 0.0
    hist = []
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
        dt = min(0.10, max(now - t_prev, 0.02))
        t_prev = now
        v_prev_c = v_prev
        v_prev = v
        hist.append((now, v))
        while len(hist) > 1 and now - hist[0][0] > 1.2:
            hist.pop(0)
        if hist and now - hist[0][0] >= 0.9:
            peak1s = max(peak1s, abs((v - hist[0][1]) / (now - hist[0][0])))

        # 推进最近点
        while idx < len(route) - 1:
            if math.hypot(route[idx][0] - pose.pos_x, route[idx][1] - pose.pos_y) < 4.0:
                idx += 1
            else:
                break

        # 纯跟踪前视点
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

        # 预判限速：看前方 12 m 内最大的局部转角
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
        # 当前转角大时也压速
        if abs(steer) > 0.6:
            vt = min(vt, 1.8)

        dist_left = math.hypot(route[-1][0] - pose.pos_x, route[-1][1] - pose.pos_y)

        if dist_left < 3.0:
            if not done:
                done = True
                print(f"接近终点 3 m，开始减速（剩余 {dist_left:.1f} m）")
            vt = min(vt, max(0.4, 0.6 * dist_left))

        thr, brk = speed_cmd(v, vt)
        send(VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer))

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"v={v:5.2f} vt={vt:4.1f} steer={steer:+5.2f} idx={idx} left={dist_left:5.1f}")

        if dist_left < 0.8 and abs(v) < 0.3:
            print(f"到达终点，用时 {now - t0:.1f}s，加速度峰值(1s) {peak1s:.2f}")
            break

        if now - t0 > MAX_SECONDS or idx >= len(route) - 1 and dist_left < 1.0:
            print(f"结束：用时 {now - t0:.1f}s，剩余 {dist_left:.1f} m，加速度峰值(1s) {peak1s:.2f}")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.3))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
