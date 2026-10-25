# -*- coding: utf-8 -*-
"""垂直泊车 v4：满舵单圆弧倒车 + 直线补齐。

★★★ 垂直泊车**效果最好**的版本（2026-10-25 复核确认，优先复用这一份）★★★
    复核实测：单场景「垂直车道泊车」210.00（16:25:57–16:26:28，用时 29.5 s），
    入库中心误差 0.14 m（dx +0.05 / dy +0.13），航向误差 1.5°。
    同场对比 tools/park_v8.py 同为 210.00，但中心误差 0.24 m、航向 −2.3°。
    `tools/park_cs_single_arc.py` 与 `metacar_race/control/vertical_park.py`
    都是本文件的移植/封装，同属这一系，改动作请先改这里再同步过去。

实测（probe_max.py）：steer=+1.0 → R=4.20 m，比车位横向距离 5.64 m 小，几何可行。

路径：
    1) 车道内开到 (x=lane_x, y=slot_y+4.2)  ← 圆弧起点，正好 R 个车身
    2) 倒车、满舵 steer=+1.0，直到车头朝 +x（与车位轴线平行）
    3) 继续保持倒车、回正方向，直线退到车位中心
    4) 挂 P 保持 3 秒

用法：
    .venv\\Scripts\\python.exe -u tools/park_v4.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = 30
ARC_RADIUS = 5.26         # 倒车满舵实测半径（probe_reverse）
STAGE_SPEED = 0.14
REVERSE_SPEED = 0.13
TARGET_TOL = 0.30
PARK_DEPTH_TOL = 0.15      # 车心与车位中心的 x 偏差（入得够深）
PARK_LATERAL_TOL = 0.25    # y 偏差（横向不偏）
PARK_HEADING_TOL = 6.0     # 航向误差
ARC_END_Y_FIX = 0.30       # 圆弧结束时的 y 欠量补偿（实测 dy≈-0.19，抬 0.2 补回）
HEADING_TOL_DEG = 8.0
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
        if v == PARKING_SLOT_TYPE:
            return o
    return None


def speed_cmd(v_meas, v_target):
    """连续速度闭环：返回 (throttle, brake)，不同时非零。"""
    err = v_target - abs(v_meas)
    if abs(err) < 0.06:
        return 0.0, 0.0            # 死区：误差小就滑行，避免油门刹车来回切
    if err > 0:
        return min(0.10, 0.01 + 0.04 * err), 0.0
    return 0.0, min(0.10, -0.06 * err)


def main() -> int:
    print("等待场景连接...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    slot = None
    stage = None
    phase = "stage"
    t0 = time.time()
    hold_t = 0.0
    last = 0.0
    prev_d = 99.0
    v_prev = 0.0
    t_prev = t0
    accel = 0.0
    peak_raw = 0.0
    hist = []
    peak_100ms = 0.0
    peak_1s = 0.0
    lim = {"thr": 0.0, "brk": 0.0}

    def send(vc, dt):
        """油门/刹车限斜率，并用加速度反馈保护 |a| <= 5。"""
        thr, brk = vc.throttle, vc.brake
        lim["thr"] += max(-0.0015, min(0.0015, thr - lim["thr"]))
        lim["brk"] += max(-0.004, min(0.004, brk - lim["brk"]))
        lim["thr"] = max(0.0, min(1.0, lim["thr"]))
        lim["brk"] = max(0.0, min(1.0, lim["brk"]))
        if accel > 3.6:
            lim["thr"] = max(0.0, lim["thr"] - 0.02)
        elif accel < -3.6:
            lim["brk"] = max(0.0, lim["brk"] - 0.04)
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
        accel = 0.9 * accel + 0.1 * max(-10.0, min(10.0, a_raw))
        peak_raw = max(peak_raw, abs(a_raw))
        hist.append((now, v))
        while len(hist) > 1 and now - hist[0][0] > 1.2:
            hist.pop(0)
        if hist and now - hist[0][0] >= 0.08:
            peak_100ms = max(peak_100ms, abs((v - hist[0][1]) / max(now - hist[0][0], 1e-3)))
        if hist and now - hist[0][0] >= 0.9:
            peak_1s = max(peak_1s, abs((v - hist[0][1]) / max(now - hist[0][0], 1e-3)))
        if abs(a_raw) > 5.0:
            print(f"!! 尖峰 |a|={abs(a_raw):5.1f}  phase={phase} dt={dt:.3f} "
                  f"v_prev={v_prev_:+.2f} v={v:+.2f}")

        if slot is None:
            slot = find_slot(msg.obstacles)
            if slot is None:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                continue
            # 倒车左转会往 +y 方向退，所以起点要放在车位「下方」R 处
            stage = (pose.pos_x, slot.pos_y - ARC_RADIUS + ARC_END_Y_FIX)
            print(f"车位({slot.pos_x:.2f},{slot.pos_y:.2f})  R={ARC_RADIUS}  "
                  f"圆弧起点({stage[0]:.2f},{stage[1]:.2f})")

        d_center = math.hypot(slot.pos_x - pose.pos_x, slot.pos_y - pose.pos_y)
        dyaw = abs(math.degrees(wrap(heading_rad(pose.ori_z) - heading_rad(slot.ori_z))))

        if phase == "stage":
            d_stage = math.hypot(stage[0] - pose.pos_x, stage[1] - pose.pos_y)
            if d_stage < 20.0:
                # 提前线性减速，避免最后一脚急刹把 |a| 顶过 5
                if abs(v) < 0.30 and d_stage < 0.45:
                    phase = "arc"
                    print(f"到位并停稳(d={d_stage:.2f}, v={v:.2f})，切倒挡满舵")
                    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.20))
                    time.sleep(0.4)
                else:
                    a_req = (v * v) / (2.0 * max(d_stage, 0.3))
                    brk = max(0.0, min(0.10, (a_req - 0.8) / 25.0))
                    vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=brk)
                    send(vc, dt)
            else:
                dx, dy = stage[0] - pose.pos_x, stage[1] - pose.pos_y
                err = wrap(math.atan2(dy, dx) - heading_rad(pose.ori_z))
                steer = max(-1.0, min(1.0, -1.5 * err))
                thr = STAGE_SPEED if d_stage > 4 else 0.12
                # 起步 1.2 秒内轻踩，压住场景启动时的加速度尖峰
                if now - t0 < 2.5:
                    thr = 0.02
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, steering=steer)
                send(vc, dt)

        elif phase == "arc":
            # 倒车满舵，直到车头与车位轴线平行
            if dyaw < HEADING_TOL_DEG:
                phase = "straight"
                print(f"车头已摆正(dyaw={dyaw:.1f}°)，回正倒直线")
            thr, brk = speed_cmd(v, 0.55)
            send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=1.0), dt)

        elif phase == "straight":
            # 分轴判据：深度（x）要进得够深，横向（y）不能偏，航向要正
            dx = pose.pos_x - slot.pos_x
            dy = pose.pos_y - slot.pos_y
            deep_enough = abs(dx) < PARK_DEPTH_TOL and abs(dy) < PARK_LATERAL_TOL
            if (abs(dx) < 0.06) or (dx < -0.35) or (prev_d < 0.9 and d_center > prev_d + 0.10):
                phase = "hold"
                hold_t = now
                print(f"入库：dx={dx:+.2f} dy={dy:+.2f} 航向 {dyaw:.1f}°")
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.20))
            else:
                # 直退的同时用陀螺仪修正朝向
                err = wrap(heading_rad(slot.ori_z) - heading_rad(pose.ori_z))
                steer = max(-0.4, min(0.4, 1.2 * err))
                # 目标速度随剩余深度平滑衰减，最后 0.6 m 放慢到 0.15 m/s
                vt = max(0.15, min(0.80, 1.1 * max(abs(dx) - 0.10, 0.0) + 0.22))
                thr, brk = speed_cmd(v, vt)
                send(VehicleControl(gear=GearMode.REVERSE, throttle=thr, brake=brk, steering=steer), dt)

        else:
            # 停稳过程也做斜坡：先轻踩，1.5 秒后再压满，避免最后一下急刹
            ramp = min(1.0, 0.22 + 0.55 * (now - hold_t))
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=ramp))
            if now - hold_t > HOLD_SECONDS:
                print(f"加速度峰值：逐帧 {peak_raw:.2f}，0.1s {peak_100ms:.2f}，1s {peak_1s:.2f} m/s2")
                print(f"保持完成，用时 {now - t0:.1f}s，中心误差 {d_center:.2f} m，"
                      f"航向误差 {dyaw:.1f}°")
                break

        if now - last > 0.5:
            last = now
            print(f"[{now - t0:5.1f}s] {phase:<8} pos=({pose.pos_x:7.2f},{pose.pos_y:7.2f}) "
                  f"yaw={pose.ori_z:6.1f} v={v:5.2f} a_raw={a_raw:+6.2f} a={accel:+5.2f} "
                  f"d={d_center:5.2f} dyaw={dyaw:5.1f}")

        if now - t0 > 90:
            print("超时")
            break

        prev_d = d_center

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
