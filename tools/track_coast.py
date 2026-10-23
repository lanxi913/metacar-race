# -*- coding: utf-8 -*-
"""v2 保守循迹：**全程不主动刹车**，只靠油门开/关控速；入弯前滑行减速。

依据（今晚实测）：
  - 仿真车任何非零油门都会蠕行到 4.76 m/s
  - 刹车会触发「加速度低于允许下限」
  - 压线扣 5 分/次，多发生在切弯

策略：
  1. 起步：油门 0.02 + 刹车斜坡（0.28 -> 0），3 秒内放开
  2. 巡航：目标速度 = 直道 4.6、弯道按曲率 2.2~3.0；**只用油门 0 / 0.02 切换**，
     低于目标就点油门，高于目标就松油门滑行（不刹车）
  3. 终点：距终点 6 m 起松油门滑行，2 m 内用 <=0.05 的刹车轻踩停住
  4. 方向盘一阶滤波（每帧 ±0.02），前视 2.5 m 抑制切弯

用法：
    .venv\\Scripts\\python.exe -u tools/track_coast.py
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

LOOKAHEAD = 2.5
SPAN = 5


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    return math.radians(-yaw_deg)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vmax", type=float, default=4.6)
    ap.add_argument("--vcorn", type=float, default=2.2)
    ap.add_argument("--seconds", type=float, default=240)
    args = ap.parse_args()

    api = SceneAPI()
    print("等待场景连接...")
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    route = [(p.x, p.y) for p in static.route]
    print(f"路线点 {len(route)}  子场景 {[s.name for s in static.sub_scenes]}")

    curv = [0.0] * len(route)
    for i in range(len(route)):
        j0, j1 = max(0, i - SPAN), min(len(route) - 1, i + SPAN)
        if j1 - i < 1 or i - j0 < 1:
            continue
        a = math.atan2(route[i][1] - route[j0][1], route[i][0] - route[j0][0])
        b = math.atan2(route[j1][1] - route[i][1], route[j1][0] - route[i][0])
        curv[i] = abs(wrap(b - a))

    idx = 0
    t0 = time.time()
    last = 0.0
    lim = {"thr": 0.0, "brk": 0.0, "steer": 0.0}

    def send(vc):
        lim["thr"] += max(-0.0008, min(0.0008, vc.throttle - lim["thr"]))
        lim["brk"] += max(-0.002, min(0.002, vc.brake - lim["brk"]))
        lim["steer"] += max(-0.02, min(0.02, vc.steering - lim["steer"]))
        lim["thr"] = max(0.0, min(1.0, lim["thr"]))
        lim["brk"] = max(0.0, min(1.0, lim["brk"]))
        lim["steer"] = max(-1.0, min(1.0, lim["steer"]))
        vc.throttle, vc.brake, vc.steering = lim["thr"], lim["brk"], lim["steer"]
        api.set_vehicle_control(vc)

    for msg, _f in api.main_loop():
        pose = msg.pose_gnss
        v = msg.main_vehicle.speed
        now = time.time()

        while idx < len(route) - 1:
            if math.hypot(route[idx][0] - pose.pos_x, route[idx][1] - pose.pos_y) < 2.0:
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
            delta = 0.5 if ly > 0 else -0.5
        else:
            delta = math.atan(2.0 * ly / d)
        steer = max(-1.0, min(1.0, -delta / 0.6))

        # 前方曲率 -> 目标速度
        max_turn = 0.0
        acc = 0.0
        for j in range(idx, len(route)):
            if j > 0:
                acc += math.hypot(route[j][0] - route[j - 1][0], route[j][1] - route[j - 1][1])
            max_turn = max(max_turn, curv[j])
            if acc > 10.0:
                break
        vt = args.vmax
        if max_turn > math.radians(30):
            vt = min(vt, args.vcorn)
        elif max_turn > math.radians(15):
            vt = min(vt, max(args.vcorn, 3.0))
        if abs(steer) > 0.6:
            vt = min(vt, args.vcorn)

        dist_left = math.hypot(route[-1][0] - pose.pos_x, route[-1][1] - pose.pos_y)
        if dist_left < 6.0:
            vt = min(vt, max(0.8, 0.5 * dist_left))

        # 控速：不刹车，只开关油门
        thr = 0.02 if v < vt else 0.0
        brk = 0.0
        if now - t0 < 3.0:
            thr = min(thr, 0.02)
            brk = max(0.0, 0.28 - 0.09 * (now - t0))
        elif dist_left < 2.0 and v < 1.2:
            brk = 0.05

        send(VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer))

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) v={v:5.2f} "
                  f"vt={vt:4.1f} thr={lim['thr']:5.3f} brk={lim['brk']:5.3f} left={dist_left:5.1f}")

        if dist_left < 0.8 and abs(v) < 0.25:
            print(f"到达终点，用时 {now - t0:.1f}s")
            break
        if now - t0 > args.seconds:
            print(f"超时退出（剩余 {dist_left:.1f} m）")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.2))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
