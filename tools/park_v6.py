# -*- coding: utf-8 -*-
"""垂直泊车 v6：全部由「车位坐标系」推导，不含场景魔法数。

核心思想
--------
拿到车位位姿 (center, ori) 后建一个局部坐标系：

    u = 车位轴线方向（指向车位开口外）
    v = 垂直方向（横向）

自车相对车位：
    a_long = (pos - center) · u      # 沿轴线，>0 表示在车位外面
    a_lat  = (pos - center) · v      # 横向，0 表示正好在轴线上

于是：

    对位点 = center + u*当前a_long + v*(-side*(R + 0.10))
    side   = sign(a_lat)             # 车在车位哪一侧
    倒车圆弧转向符号 = +side          # 由几何决定，不是试出来的

倒车 90° 后车头朝向 +u，正好是车位轴线方向；再直线倒到 a_long < 0.06。

全程只用三个物理量：车位位姿、车辆满舵半径 R、容差。
换任何位置的垂直车位，只要给到车位位姿，这套逻辑都成立。
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = 30
ARC_RADIUS = 5.26          # 车辆满舵倒车半径（实测，车辆属性）
STAGING_OFFSET = 0.10      # 对位点相对圆弧半径再多让 0.1 m
STAGE_SPEED = 0.14
STOP_LONG_TOL = 0.06
STOP_LAT_TOL = 0.15
HOLD_SECONDS = 3.0


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def heading_rad(yaw_deg):
    """场景 yaw 顺时针为正 → 数学角。"""
    return math.radians(-yaw_deg)


def find_slot(obs):
    for o in obs:
        v = getattr(getattr(o, "type", None), "value", getattr(o, "type", None))
        if v == PARKING_SLOT_TYPE:
            return o
    return None


def speed_cmd(v_meas, v_target):
    err = v_target - abs(v_meas)
    if abs(err) < 0.06:
        return 0.0, 0.0
    if err > 0:
        return min(0.10, 0.01 + 0.04 * err), 0.0
    return 0.0, min(0.10, -0.06 * err)


def main() -> int:
    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    slot = None
    frame = None
    phase = "approach"
    t0 = time.time()
    hold_t = 0.0
    last = 0.0
    v_prev = 0.0
    t_prev = t0
    accel = 0.0
    peak1s = 0.0
    hist = []
    lim = {"thr": 0.0, "brk": 0.0}
    lat_sign = 1.0
    lat_prev = None
    lat_check_t = 0.0

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
        v_prev_calc = v_prev
        v_prev = v
        a_raw = (v - v_prev_calc) / dt
        accel = 0.9 * accel + 0.1 * max(-12.0, min(12.0, a_raw))
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
            th = heading_rad(slot.ori_z)                 # 车位轴线数学角
            ux, uy = math.cos(th), math.sin(th)
            vx, vy = -math.sin(th), math.cos(th)
            rel_lat = (pose.pos_x - slot.pos_x) * vx + (pose.pos_y - slot.pos_y) * vy
            side = 1.0 if rel_lat >= 0 else -1.0
            frame = dict(ux=ux, uy=uy, vx=vx, vy=vy, side=side, th=th)
            print(f"车位({slot.pos_x:.2f},{slot.pos_y:.2f}) 轴线角 {slot.ori_z:.1f}°  "
                  f"车在 {('+' if side > 0 else '-')}v 侧  R={ARC_RADIUS}")

        ux, uy, vx, vy = frame["ux"], frame["uy"], frame["vx"], frame["vy"]
        side, th_slot = frame["side"], frame["th"]

        rx, ry = pose.pos_x - slot.pos_x, pose.pos_y - slot.pos_y
        a_long = rx * ux + ry * uy
        a_lat = rx * vx + ry * vy
        heading_err = wrap(th_slot - heading_rad(pose.ori_z))

        # 对位点：保持当前沿轴线位置，横向退到 -side*(R+偏移)
        stage_lat = -side * (ARC_RADIUS + STAGING_OFFSET)
        stage_x = slot.pos_x + ux * a_long + vx * stage_lat
        stage_y = slot.pos_y + uy * a_long + vy * stage_lat
        d_stage = math.hypot(stage_x - pose.pos_x, stage_y - pose.pos_y)

        # ---------- 1) 接近对位点 ----------
        if phase == "approach":
            if d_stage < 0.45:
                phase = "stop"
                print(f"到对位点(d={d_stage:.2f})，准备停稳")
                continue
            if d_stage < 20.0:
                a_req = (v * v) / (2.0 * max(d_stage, 0.3))
                brk = max(0.0, min(0.10, (a_req - 0.8) / 25.0))
                thr = 0.0
            else:
                thr, brk = STAGE_SPEED, 0.0
            err_h = wrap(math.atan2(stage_y - pose.pos_y, stage_x - pose.pos_x)
                         - heading_rad(pose.ori_z))
            steer = max(-1.0, min(1.0, -1.5 * err_h))
            if now - t0 < 2.5:
                thr = min(thr, 0.02)
            send(VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer))

        # ---------- 2) 停稳换倒挡（不刹死） ----------
        elif phase == "stop":
            if abs(v) < 0.30:
                phase = "arc"
                lim["thr"] = lim["brk"] = 0.0
                print(f"停稳(v={v:.2f})，挂倒挡，转向符号 {side:+.0f}")
            else:
                send(VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.10))

        # ---------- 3) 满舵倒车，车头摆到车位轴线 ----------
        elif phase == "arc":
            if abs(heading_err) < math.radians(1.5):
                phase = "straight"
                print(f"车头摆正(误差 {math.degrees(heading_err):.1f}°)，回正倒直线")
            thr, brk = speed_cmd(v, 0.55)
            send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=side))

        # ---------- 4) 直线倒进车位 ----------
        elif phase == "straight":
            # 横向自适应：每 0.6 秒看一次 |a_lat| 有没有变小，没变小就翻符号
            if lat_prev is None:
                lat_prev, lat_check_t = a_lat, now
            elif now - lat_check_t > 0.6:
                if abs(a_lat) > abs(lat_prev) + 0.01:
                    lat_sign = -lat_sign
                    print(f"横向修正符号翻转 -> {lat_sign:+.0f}")
                lat_prev, lat_check_t = a_lat, now

            if abs(a_long) < STOP_LONG_TOL or a_long < -0.45:
                phase = "hold"
                hold_t = now
                print(f"入库：a_long={a_long:+.2f} a_lat={a_lat:+.2f} "
                      f"航向 {math.degrees(heading_err):+.1f}°")
            else:
                vt = max(0.15, min(0.80, 1.1 * max(abs(a_long) - 0.10, 0.0) + 0.22))
                thr, brk = speed_cmd(v, vt)
                steer = max(-0.45, min(0.45, 1.6 * heading_err + lat_sign * 0.5 * a_lat))
                send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=steer))

        # ---------- 5) 驻车 ----------
        else:
            ramp = min(1.0, 0.22 + 0.55 * (now - hold_t))
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=ramp))
            if now - hold_t > HOLD_SECONDS:
                print(f"加速度峰值（1s 窗口）{peak1s:.2f} m/s2")
                print(f"完成：a_long={a_long:+.2f} a_lat={a_lat:+.2f} "
                      f"航向 {math.degrees(heading_err):+.1f}°  用时 {now - t0:.1f}s")
                break

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] {phase:<9} pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"a_long={a_long:+6.2f} a_lat={a_lat:+6.2f} v={v:5.2f} "
                  f"head={math.degrees(heading_err):+5.1f}°")

        if now - t0 > 90:
            print("超时")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.3))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
