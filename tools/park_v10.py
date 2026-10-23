# -*- coding: utf-8 -*-
"""通用泊车 v10：两段圆弧（Reeds-Shepp 风格），支持斜列/平行/垂直车位。

为什么需要两段
--------------
斜列车位从车道倒进去要转 111°，而单圆弧的弦长 2R·sin(φ/2)=8.7 m，
车道到车位只有 2~3 m，单段放不下。拆成两段后每段弦长 ~5 m，就够用了。

规划
----
    段1：倒车圆弧 A，曲率 κ1，转角 φ1
    段2：倒车圆弧 B，曲率 κ2 = −κ1，转角 φ2
    段3：直线倒车 s

终点已知（车位中心，车头朝开口），反推起点：
    θ0 = θ_goal − (φ1 + φ2)
    Δ  = 正向积分(θ0, κ1, φ1, κ2, φ2, s)
    start = goal − Δ
约束：起点航向 = 接近方向、起点落在车道侧、全程不碰两侧停车。

用法：
    .venv\\Scripts\\python.exe -u tools/park_v10.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

SLOT_TYPE = 30
R = 5.26                 # 满舵倒车半径
CAR_L, CAR_W = 4.00, 1.69
STOP_LONG_TOL = 0.08
HOLD_SECONDS = 3.0


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    return math.radians(-yaw_deg)


def find_slot(obs):
    for o in obs:
        v = getattr(getattr(o, "type", None), "value", getattr(o, "type", None))
        if v == SLOT_TYPE:
            return o
    return None


def integrate(theta0, k1, phi1, k2, phi2, s):
    """正向积分（倒车：弧长取负），返回相对位移 (dx, dy) 和终点航向。"""
    x = y = 0.0
    th = theta0
    # 倒车：ds = -R*|φ|，航向变化 = κ*ds；用符号统一
    for kappa, phi in ((k1, phi1), (k2, phi2)):
        if abs(kappa) < 1e-9 or abs(phi) < 1e-9:
            continue
        ds = phi / kappa          # 带符号弧长
        th2 = th + phi
        thm = th + 0.5 * phi
        x += math.cos(thm) * ds
        y += math.sin(thm) * ds
        th = th2
    if s:
        th2 = th
        x += math.cos(th2) * s
        y += math.sin(th2) * s
    return x, y, th


def car_poly(x, y, th, L=CAR_L, W=CAR_W):
    c, s = math.cos(th), math.sin(th)
    pts = []
    for dx, dy in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2)):
        pts.append((x + dx * c - dy * s, y + dx * s + dy * c))
    return pts


def overlap(p1, p2):
    for poly in (p1, p2):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            nx, ny = y2 - y1, -(x2 - x1)
            ln = math.hypot(nx, ny)
            if ln < 1e-9:
                continue
            nx, ny = nx / ln, ny / ln
            a = [nx * px + ny * py for px, py in p1]
            b = [nx * px + ny * py for px, py in p2]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


def obstacles_polys(obs):
    polys = []
    for o in obs:
        t = getattr(getattr(o, "type", None), "value", getattr(o, "type", None))
        if t != SLOT_TYPE:
            polys.append((o.pos_x, o.pos_y, heading_rad(o.ori_z), 4.0, 1.8))
    return polys


def plan(goal, theta_goal, approach, entry, obstacles, car_pose):
    """枚举两段圆弧，返回可行方案（段列表 + 起点）。"""
    phi_total = wrap(theta_goal - approach)
    best = None
    for k1 in (1 / R, -1 / R):
        k2 = -k1
        for deg1 in range(-150, 151, 5):
            phi1 = math.radians(deg1)
            phi2 = phi_total - phi1
            if abs(phi2) > math.radians(150):
                continue
            for s in (0.0, 1.0, 2.0, 3.0):
                theta0 = theta_goal - (phi1 + phi2)
                dx, dy, _ = integrate(theta0, k1, phi1, k2, phi2, s)
                sx, sy = goal[0] - dx, goal[1] - dy
                proj = (sx - goal[0]) * entry[0] + (sy - goal[1]) * entry[1]
                if proj < 1.0:
                    continue
                # 起点要在车附近（可达）
                d_car = math.hypot(sx - car_pose[0], sy - car_pose[1])
                # 碰撞检查：沿路径采样
                ok = True
                x, y, th = sx, sy, theta0
                for kappa, phi in ((k1, phi1), (k2, phi2)):
                    steps = max(4, int(abs(math.degrees(phi)) / 10))
                    for _ in range(steps):
                        ds = (phi / kappa) / steps if abs(kappa) > 1e-9 else 0.0
                        thm = th + 0.5 * (phi / steps)
                        x += math.cos(thm) * ds
                        y += math.sin(thm) * ds
                        th += phi / steps
                        body = car_poly(x, y, th)
                        for (ox, oy, oth, ol, ow) in obstacles:
                            if overlap(body, car_poly(ox, oy, oth, ol, ow)):
                                ok = False
                                break
                        if not ok:
                            break
                    if not ok:
                        break
                if not ok:
                    continue
                # 起点必须还在车道走廊里（横向偏移 < 3 m），否则会开进停车场
                if abs(sx - car_pose[0]) > 3.0:
                    continue
                # 起点必须能从当前位置直线开过去（否则会撞已停车）
                ok2 = True
                steps_a = max(6, int(d_car / 1.0))
                for i in range(steps_a + 1):
                    t = i / steps_a
                    px = car_pose[0] + (sx - car_pose[0]) * t
                    py = car_pose[1] + (sy - car_pose[1]) * t
                    body = car_poly(px, py, theta0)
                    for (ox, oy, oth, ol, ow) in obstacles:
                        if overlap(body, car_poly(ox, oy, oth, ol, ow)):
                            ok2 = False
                            break
                    if not ok2:
                        break
                if not ok2:
                    continue
                score = abs(math.degrees(phi1)) + abs(math.degrees(phi2)) + s * 20 + d_car * 0.5
                if best is None or score < best[0]:
                    best = (score, (sx, sy, theta0), phi1, k1, phi2, k2, s)
    return best


def speed_cmd(v, vt):
    err = vt - abs(v)
    if abs(err) < 0.06:
        return 0.0, 0.0
    if err > 0:
        return min(0.10, 0.01 + 0.04 * err), 0.0
    return 0.0, min(0.10, -0.06 * err)


def main() -> int:
    api = SceneAPI()
    print("等待场景连接...")
    api.connect()
    print("已连接。")

    slot = None
    plan_ = None
    stage = 0
    phase = "approach"
    t0 = time.time()
    hold_t = 0.0
    last = 0.0
    v_prev = 0.0
    t_prev = t0
    peak1s = 0.0
    hist = []
    theta_acc = 0.0
    theta_last = None
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
        vp = v_prev
        v_prev = v
        hist.append((now, v))
        while len(hist) > 1 and now - hist[0][0] > 1.2:
            hist.pop(0)
        if hist and now - hist[0][0] >= 0.9:
            peak1s = max(peak1s, abs((v - hist[0][1]) / (now - hist[0][0])))

        if slot is None:
            slot = find_slot(msg.obstacles)
            if slot is None:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.2))
                continue
            th_raw = heading_rad(slot.ori_z)
            a0 = (pose.pos_x - slot.pos_x) * math.cos(th_raw) + \
                 (pose.pos_y - slot.pos_y) * math.sin(th_raw)
            sign = 1.0 if a0 >= 0 else -1.0
            theta_goal = th_raw if sign > 0 else wrap(th_raw + math.pi)
            entry = (math.cos(theta_goal), math.sin(theta_goal))
            obstacles = obstacles_polys(msg.obstacles)
            print(f"车位({slot.pos_x:.2f},{slot.pos_y:.2f}) 轴线 {slot.ori_z:.1f}° "
                  f"开口方向 {math.degrees(theta_goal):.1f}°")

        th_slot = theta_goal
        rx, ry = pose.pos_x - slot.pos_x, pose.pos_y - slot.pos_y
        a_long = rx * math.cos(th_slot) + ry * math.sin(th_slot)
        heading_err = wrap(th_slot - heading_rad(pose.ori_z))

        if plan_ is None and math.hypot(rx, ry) < 22.0:
            approach = heading_rad(pose.ori_z)
            best = plan((slot.pos_x, slot.pos_y), theta_goal, approach, entry,
                        obstacles, (pose.pos_x, pose.pos_y))
            if best:
                _, start, phi1, k1, phi2, k2, s = best
                plan_ = dict(start=start, phi1=phi1, k1=k1, phi2=phi2, k2=k2, s=s)
                print(f"规划成功：起点({start[0]:.2f},{start[1]:.2f}) "
                      f"φ1={math.degrees(phi1):+.0f}° φ2={math.degrees(phi2):+.0f}° 直线{s}m")
            else:
                print("!! 无可行两段圆弧")

        if phase == "approach":
            if plan_:
                d = math.hypot(plan_["start"][0] - pose.pos_x, plan_["start"][1] - pose.pos_y)
                if d < 0.40:
                    phase = "stop"
                    print(f"到起点(d={d:.2f})，停稳换倒挡")
                    continue
                thr = 0.14 if d > 20 else (0.0 if d < 6 else 0.05)
                brk = 0.0
                if d < 20:
                    a_req = (v * v) / (2.0 * max(d, 0.3))
                    brk = max(0.0, min(0.10, (a_req - 0.8) / 25.0))
                err = wrap(math.atan2(plan_["start"][1] - pose.pos_y,
                                      plan_["start"][0] - pose.pos_x) - heading_rad(pose.ori_z))
                steer = max(-1.0, min(1.0, -1.5 * err))
                send(VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer))
            else:
                send(VehicleControl(gear=GearMode.DRIVE, throttle=0.05, brake=0.0, steering=0.0))

        elif phase == "stop":
            if abs(v) < 0.25:
                phase = "arc1"
                lim["thr"] = lim["brk"] = 0.0
                theta_acc = 0.0
                theta_last = heading_rad(pose.ori_z)
                print(f"停稳(v={v:.2f})，开始第一段圆弧")
            else:
                send(VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.08))

        elif phase in ("arc1", "arc2"):
            if theta_last is not None:
                theta_acc += wrap(heading_rad(pose.ori_z) - theta_last)
            theta_last = heading_rad(pose.ori_z)
            target = plan_["phi1"] if phase == "arc1" else plan_["phi2"]
            if abs(theta_acc) >= abs(target) - math.radians(3):
                if phase == "arc1":
                    phase = "arc2"
                    theta_acc = 0.0
                    print("第一段完成，转入第二段圆弧")
                else:
                    phase = "straight"
                    print(f"两段圆弧完成（累计 {math.degrees(theta_acc):.0f}°），回正倒直线")
            k = plan_["k1"] if phase == "arc1" else plan_["k2"]
            steer = max(-1.0, min(1.0, k * R))
            thr, brk = speed_cmd(v, 0.50)
            send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=steer))

        elif phase == "straight":
            if abs(a_long) < STOP_LONG_TOL or a_long < -0.45:
                phase = "hold"
                hold_t = now
                print(f"入库：a_long={a_long:+.2f} 航向 {math.degrees(heading_err):+.1f}°")
            else:
                vt = max(0.15, min(0.70, 0.9 * max(abs(a_long) - 0.10, 0.0) + 0.20))
                thr, brk = speed_cmd(v, vt)
                steer = max(-0.45, min(0.45, 1.4 * heading_err))
                send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=steer))

        else:
            if abs(v) > 0.08:
                send(VehicleControl(gear=GearMode.REVERSE, throttle=0.0, brake=0.08))
            else:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.8))
            if now - hold_t > HOLD_SECONDS:
                print(f"加速度峰值（1s）{peak1s:.2f}")
                print(f"完成：a_long={a_long:+.2f} 航向 {math.degrees(heading_err):+.1f}° "
                      f"用时 {now - t0:.1f}s")
                break

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] {phase:<8} pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"a_long={a_long:+6.2f} v={v:5.2f} head={math.degrees(heading_err):+5.1f}°")

        if now - t0 > 100:
            print("超时")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.3))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
