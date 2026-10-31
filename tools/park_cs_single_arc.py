# -*- coding: utf-8 -*-
r"""连续场景 · 垂直泊车 · 单圆弧移植版。

策略直接照搬已在单场景拿到 200 分的 chang-kou 基线（提交 5f72206）：

    1. 沿车道行驶，**开过**车位；
    2. 在车位中心往前（行车方向）约 5.6 m 处停稳；
    3. 挂 R，持续反向转向，让车尾扫进车位；
    4. 车身与车位轴线平行、中心误差 <= 0.35 m 时驻车保持。

与基线的唯一区别是"怎么到车位"：连续场景里车位在 180 m 外、且前面还有四个元素，
所以用 --skip-to-index 自动跳关直达垂直泊车元素，再用曲率限速沿路线开到车位前。

运行顺序：
    python -u tools\park_cs_single_arc.py --skip-to-index 5
    → 客户端启动"连续场景1"

CSV 每帧记录车身四角相对车位框的余量（margin<0 表示压出库边线），
这是"不能压库边线"这条判据的直接证据。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, ObstacleType, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.config import LOOKAHEAD_M  # noqa: E402
from metacar_race.control.profile import SpeedProfile  # noqa: E402
from metacar_race.control.tracking import PurePursuit, SpeedController  # noqa: E402

PARKING_SLOT_TYPE = 30

APPROACH_THROTTLE = 0.01        # 靠近车位时的爬行油门（基线值）
STAGE_DECEL_MS2 = 2.70
STAGE_LEAD_M = 0.10
STAGE_STILL_SPEED_MS = 0.30
STAGE_SETTLE_SECONDS = 0.80
STAGE_PAST_M = 5.57             # 基线实测：停稳点 (-2.16,-5.69) 距车位中心 5.57 m

REVERSE_THROTTLE = 0.16
REVERSE_BRAKE = 0.20
REVERSE_MAX_SPEED_MS = 1.20
REVERSE_STOP_START_M = 1.50
REVERSE_CREEP_THROTTLE = 0.04
REVERSE_STEER_SIGN = -1.0

CENTER_TOL_M = 0.35
HEADING_TOL_DEG = 8.0
STAGE_LATERAL_M = 5.64          # 车位中心到车道中心的横向距离（基线与连续场景实测一致）
STOP_AT_STAGE_M = 0.35          # 距准备点这么近就刹停换倒挡
AIM_NEAR_M = 28.0               # 距准备点这么近就不再跟赛道，直接对准准备点


class _Point:
    __slots__ = ("x", "y")

    def __init__(self, x: float, y: float) -> None:
        self.x = float(x)
        self.y = float(y)


def aim_steering(x, y, yaw, target, lookahead=3.0):
    """直接对准准备点的转向（复用 PurePursuit 的方向盘约定）。"""
    path = [_Point(x, y), _Point(target[0], target[1])]
    return PurePursuit(path, lookahead=lookahead).steering(x, y, yaw)


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def find_slot(obstacles, x=None, y=None):
    """取停车位；视野里有多个时取离自车最近的（原来直接取第一个，双车位场景会选错）。

    type 判据兼容 枚举/整数 两种写法（SDK 现在是 ObstacleType.PARKING_SLOT = 30）。
    """
    found = []
    for obstacle in obstacles or ():
        t = getattr(obstacle, "type", None)
        v = getattr(t, "value", t)
        try:
            ok = int(v) == PARKING_SLOT_TYPE
        except (TypeError, ValueError):
            ok = "PARKING_SLOT" in str(v).upper()
        if ok:
            found.append(obstacle)
    if not found:
        return None
    if x is not None and y is not None and len(found) > 1:
        print(f"[slot] 视野里 {len(found)} 个车位，按最近距离选", flush=True)
        found.sort(key=lambda o: (float(o.pos_x) - x) ** 2 + (float(o.pos_y) - y) ** 2)
    return found[0]


def slot_axes(slot):
    th = math.radians(-float(slot.ori_z))
    ux, uy = math.cos(th), math.sin(th)
    return ux, uy, -uy, ux


def ego_corners(x, y, yaw_deg, length, width):
    yaw = math.radians(-yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0, width / 2.0
    return [
        (x + dl * c - dw * s, y + dl * s + dw * c)
        for dl, dw in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))
    ]


def slot_metrics(pose, slot, length, width):
    """du/dv：车中心在车位系里的纵向/横向偏移；margin：车身相对车位框的最小余量（正=框内）。"""
    ux, uy, vx, vy = slot_axes(slot)
    sx, sy = float(slot.pos_x), float(slot.pos_y)
    hl = float(slot.length) / 2.0
    hw = float(slot.width) / 2.0
    du = (pose.pos_x - sx) * ux + (pose.pos_y - sy) * uy
    dv = (pose.pos_x - sx) * vx + (pose.pos_y - sy) * vy
    corners = ego_corners(pose.pos_x, pose.pos_y, pose.ori_z, length, width)
    cu = [(cx - sx) * ux + (cy - sy) * uy for cx, cy in corners]
    cv = [(cx - sx) * vx + (cy - sy) * vy for cx, cy in corners]
    overflow = max(max(cu) - hl, -hl - min(cu), max(cv) - hw, -hw - min(cv))
    return {
        "du": du,
        "dv": dv,
        "dist": math.hypot(du, dv),
        "ori_err": wrap_deg(float(pose.ori_z) - float(slot.ori_z)),
        "margin": -overflow,
        "inside": overflow <= 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="连续场景垂直泊车单圆弧移植")
    parser.add_argument("--skip-to-index", type=int, default=0,
                        help="只跑第 N 个元素（垂直泊车=5），其余自动 skip_level")
    parser.add_argument("--vmax", type=float, default=8.0, help="沿路线行驶的最高速度")
    parser.add_argument("--stage-past", type=float, default=STAGE_PAST_M)
    parser.add_argument("--stage-trim", type=float, default=0.26,
                        help="对位点横向修正，补满舵半径差（同 park_v8 的 LAT_TRIM）")
    parser.add_argument("--hold", type=float, default=8.0, help="入库后保持驻车秒数")
    parser.add_argument("--max-seconds", type=float, default=260.0)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()

    # 驻车刹车加斜坡：从 0 一步到 1.0 会把残余车速一下刹死（实测 +7 m/s^2 尖峰），
    # 这里把 PARKING 挡的刹车改成每帧 +0.05 的斜坡，约 0.4 s 到位，不影响驻车。
    _raw_send = api.set_vehicle_control
    _hold_brk = [0.0]

    def _send_ramp(vc):
        if vc.gear == GearMode.PARKING:
            _hold_brk[0] = min(1.0, _hold_brk[0] + 0.05)
            vc = VehicleControl(gear=vc.gear, throttle=0.0,
                                brake=_hold_brk[0], steering=vc.steering)
        else:
            _hold_brk[0] = 0.0
        _raw_send(vc)

    api.set_vehicle_control = _send_ramp
    static = api.get_scene_static_data()
    _subs = len(getattr(static, "sub_scenes", None) or [])
    _route = list(getattr(static, "route", None) or [])
    print(f"已连接。元素 {_subs} 个，赛道 {len(_route)} 点", flush=True)
    if len(_route) < 2:
        # 没有参考路径（VLA / 空场景）：驻车并退出，别在第一帧就 IndexError 崩掉
        print("!! static.route 不足 2 点，无法循迹：驻车退出", flush=True)
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        return 1

    target = ""
    elem_start = None
    if 1 <= args.skip_to_index <= _subs:
        _elem = static.sub_scenes[args.skip_to_index - 1]
        target = _elem.name
        if _elem.start_point is not None:
            elem_start = (float(_elem.start_point.x), float(_elem.start_point.y))
        print(f"目标元素：{target}（其余自动跳关）", flush=True)

    pursuit = PurePursuit(static.route, lookahead=LOOKAHEAD_M) if len(static.route) >= 2 else None
    profile = SpeedProfile(static.route, v_max=args.vmax) if len(static.route) >= 3 else None
    lim_thr = 0.0
    rev_thr = 0.0     # reverse throttle slew state
    speed_ctl = SpeedController(v_target=args.vmax, accel_limit=1.5, throttle_max=0.85)
    speed_ctl.reset(0.0)

    out_dir = args.out or (REPO / "logs" / "park_cs" / time.strftime("%Y%m%d-%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "park_cs.csv"
    csv_fh = csv_path.open("w", newline="", encoding="utf-8-sig")
    writer = csv.writer(csv_fh)
    writer.writerow([
        "frame", "sim_t", "sub_scene", "phase", "x", "y", "yaw", "speed",
        "gear", "throttle", "brake", "steering",
        "slot_dist", "slot_du", "slot_dv", "slot_ori_err", "slot_margin", "slot_inside",
    ])

    phase = "skip"
    skipped: set[str] = set()
    slot = None
    min_dist = float("inf")
    closest = None
    stage_ready = False
    stationary = 0.0
    approach_t0 = 0.0
    hold = 0.0
    frame = 0
    sim_t = 0.0
    dt = 1.0 / 30.0
    t_end = time.time() + args.max_seconds
    ending = "ended"
    final = (0.0, 0.0, 0.0)

    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            sim_t += dt
            sub = status.sub_scene_name or ""
            x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
            speed = float(car.speed)
            final = (x, y, yaw)

            if phase == "skip" and target and sub != target:
                if sub not in skipped and len(skipped) < 8:
                    skipped.add(sub)
                    api.skip_level()
                    print(f"跳过元素 {sub}", flush=True)
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                continue
            if phase == "skip":
                # 元素名会先于位姿跳变（车还在上一个元素终点），必须等车真的落位再开始跑，
                # 否则会带着上一段注入的 8 m/s 初速、按旧位置的路线索引乱打方向。
                arrived = elem_start is None
                if elem_start is not None:
                    arrived = math.hypot(elem_start[0] - x, elem_start[1] - y) <= 20.0
                if not arrived:
                    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                    continue
                phase = "approach"
                approach_t0 = sim_t
                if pursuit is not None:
                    pursuit.sync(x, y, max_jump_m=1.0)
                print(f"进入 {sub}，开始沿路线接近车位", flush=True)

            slot_now = find_slot(msg.obstacles, x, y)
            if slot is None and slot_now is not None:
                slot = slot_now
            if slot is None:
                # 还没看到车位（起跑到车位有 200 m）：照常沿赛道跑，不能在这里爬行
                if pursuit is not None and static.route:
                    i_near = min(range(len(static.route)),
                                 key=lambda k: (static.route[k].x - x) ** 2
                                 + (static.route[k].y - y) ** 2)
                    pursuit.sync(x, y)
                    steer = pursuit.steering(x, y, yaw)
                    v_tgt = profile.target(i_near, speed) if profile is not None else args.vmax
                else:
                    steer, v_tgt = 0.0, args.vmax
                speed_ctl.v_target = v_tgt
                thr, brk = speed_ctl.update(speed, dt, v_ceiling=args.vmax)
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                    brake=brk, steering=steer)
                api.set_vehicle_control(vc)
                writer.writerow([
                    frame, round(sim_t, 3), sub, "cruise", round(x, 3), round(y, 3),
                    round(yaw, 2), round(speed, 3), car.gear.name,
                    round(vc.throttle, 3), round(vc.brake, 2), round(vc.steering, 3),
                    "", "", "", "", "", "",
                ])
                if frame % 60 == 0:
                    csv_fh.flush()
                    print(f"[{sim_t:6.1f}s] cruise   pos=({x:8.2f},{y:8.2f}) "
                          f"yaw={yaw:6.1f} v={speed:5.2f} 未发现车位", flush=True)
                if time.time() > t_end:
                    ending = "timeout"
                    break
                continue

            car_len = float(getattr(car, "length", 0) or 0) or 4.814
            car_wid = float(getattr(car, "width", 0) or 0) or 2.180
            met = slot_metrics(pose, slot, car_len, car_wid)
            # 进库朝向按 ±180 取近的：原来只认 slot.ori_z，若从车位另一头进库，
            # 航向误差恒 ≈180°，倒车段的 8° 判据永远不满足，会一直倒到超时。
            _he_a = wrap_deg(float(slot.ori_z) - yaw)
            _he_b = wrap_deg(float(slot.ori_z) + 180.0 - yaw)
            heading_error = _he_a if abs(_he_a) <= abs(_he_b) else _he_b

            # 倒车准备点：车位中心沿"出库方向"5.64 m、沿行进方向再退 5.57 m（与基线同构）
            ux, uy, vx, vy = slot_axes(slot)
            stage = (float(slot.pos_x) + ux * STAGE_LATERAL_M
                     - vx * (args.stage_past - args.stage_trim),
                     float(slot.pos_y) + uy * STAGE_LATERAL_M
                     - vy * (args.stage_past - args.stage_trim))
            dsx, dsy = stage[0] - x, stage[1] - y
            d_stage = math.hypot(dsx, dsy)
            if pursuit is not None and static.route:
                i_near = min(range(len(static.route)),
                             key=lambda k: (static.route[k].x - x) ** 2
                             + (static.route[k].y - y) ** 2)
                # 跳关会把车瞬移到元素起点，纯跟踪的内部索引还停在路线开头，
                # 不移位就会朝 700 m 外的起点打满方向（把车甩出赛道）。
                pursuit.sync(x, y)
                j_near = min(len(static.route) - 1, i_near + 3)
                tx = static.route[j_near].x - static.route[i_near].x
                ty = static.route[j_near].y - static.route[i_near].y
                tn = math.hypot(tx, ty) or 1.0
                tx, ty = tx / tn, ty / tn
            else:
                rad = math.radians(-yaw)
                tx, ty = math.cos(rad), math.sin(rad)
            remaining = dsx * tx + dsy * ty

            # 到位判据只在准备点附近才成立：连续转弯段是蛇形，route 切线会让
            # stage 的投影提前变负（实测在 642 m 外就误判"到达准备点"并开始倒车）。
            near_stage = d_stage <= AIM_NEAR_M

            if phase == "approach":
                if near_stage and (remaining <= STOP_AT_STAGE_M
                                   or d_stage <= STOP_AT_STAGE_M):
                    if speed <= STAGE_STILL_SPEED_MS:
                        vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                        stationary += dt
                    else:
                        vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                            brake=max(0.05, min(0.10, speed * 0.10)), steering=0.0)
                        stationary = 0.0
                    if stationary >= STAGE_SETTLE_SECONDS:
                        stage_ready = True
                        phase = "reverse"
                        stationary = 0.0
                        print(f"到达倒车准备点 ({x:.2f},{y:.2f}) yaw={yaw:.1f} "
                              f"剩余={remaining:+.2f} 距准备点={d_stage:.2f} "
                              f"du={met['du']:+.2f} dv={met['dv']:+.2f}", flush=True)
                elif near_stage:
                    # 车位附近：不再跟赛道（赛道末段会拐进车位），直接对准准备点并减速
                    v_des = min(2.20, max(0.35, 0.90 * remaining))
                    steer = aim_steering(x, y, yaw, stage)
                    thr = min(0.20, max(0.0, 0.02 + 0.05 * (v_des - speed)))
                    brk = 0.0
                    if speed > v_des + 0.30:
                        brk = min(0.25, max(0.03, 0.05 * (speed - v_des)))
                        thr = 0.0
                    vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                        brake=brk, steering=steer)
                else:
                    if speed > 3.0 and sim_t - approach_t0 < 2.5:
                        # 平台瞬移时会注入初速（实测 8.2 m/s），前 2.5 s 先刹下去；
                        # 之后交回闭环，否则 3 m/s 的门槛会和加速段形成来回打刹的极限环。
                        vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                            brake=0.15, steering=0.0)
                    else:
                        steer = pursuit.steering(x, y, yaw) if pursuit is not None else 0.0
                        v_tgt = args.vmax
                        if profile is not None and pursuit is not None:
                            v_tgt = profile.target(i_near, speed)
                        speed_ctl.v_target = v_tgt
                        thr, brk = speed_ctl.update(speed, dt, v_ceiling=args.vmax)
                        vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                            brake=brk, steering=steer)

            elif phase == "reverse":
                if d_stage > 15.0:
                    # 兜底：误判进倒车段时退回接近段，不许在路中间倒车
                    phase = "approach"
                    approach_t0 = sim_t
                    print(f"倒车位置异常（距准备点 {d_stage:.1f} m，剩 {remaining:+.1f}），"
                          f"退回接近段", flush=True)
                    vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                        brake=0.12, steering=0.0)
                elif (met["dist"] <= CENTER_TOL_M and abs(heading_error) <= HEADING_TOL_DEG
                        and abs(speed) <= 0.06):
                    phase = "hold"
                    hold = 0.0
                    vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                    print(f"入库条件满足 d={met['dist']:.3f} dv={met['dv']:+.3f} "
                          f"yaw_error={heading_error:+.2f} margin={met['margin']:+.3f}", flush=True)
                elif (met["dist"] <= CENTER_TOL_M and abs(heading_error) <= HEADING_TOL_DEG
                      and abs(speed) > 0.06):
                    # 位置和航向都已到位，但还没停稳：只踩刹车把它刹住，
                    # 否则 P 律在 ±0.1 m/s 死区内不给油也不给刹，车会匀速滑过车位。
                    steering = max(-1.0, min(1.0, REVERSE_STEER_SIGN * heading_error / 30.0))
                    vc = VehicleControl(gear=GearMode.REVERSE, throttle=0.0,
                                        brake=min(0.08, max(0.03, abs(speed) * 0.30)),
                                        steering=steering)
                else:
                    steering = max(-1.0, min(1.0, REVERSE_STEER_SIGN * heading_error / 30.0))
                    # 连续 P 律（照 park_v8 的 speed_cmd）：油门连续给，只在明显超速时轻刹，
                    # 避免 0.16 <-> 0.20 的开关切换造成 ±6~8 m/s^2 的加速度台阶
                    vt_rev = 0.55 if met["dist"] > REVERSE_STOP_START_M else 0.20
                    err_v = vt_rev - abs(speed)
                    if err_v > 0.05:
                        cmd_thr = min(0.12, 0.01 + 0.05 * err_v)
                        cmd_brk = 0.0
                    elif err_v < -0.10:
                        cmd_thr = 0.0
                        cmd_brk = min(0.10, -0.06 * err_v)
                    else:
                        cmd_thr = 0.0
                        cmd_brk = 0.0

                    # 倒挡起步油门斜率：静止时只给 0.006（离合器门槛内），每帧最多 +0.002；
                    # 且**不得超过 P 律指令**、**刹车时绝不给油**——否则油刹同踩会冲过车位。
                    if rev_thr < 0.006:
                        rev_thr = 0.006
                    if cmd_thr > 0.0 and cmd_brk == 0.0:
                        rev_thr = min(cmd_thr, rev_thr + 0.002)
                    else:
                        rev_thr = max(0.0, rev_thr - 0.010)
                    throttle, brake = (rev_thr if cmd_brk == 0.0 else 0.0), cmd_brk
                    vc = VehicleControl(gear=GearMode.REVERSE, throttle=throttle,
                                        brake=brake, steering=steering)

            elif phase == "hold":
                hold += dt
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                if hold >= args.hold:
                    ending = "hold_complete"
                    api.set_vehicle_control(vc)
                    break
            else:
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)

            # 前段油门保护：按速度给油门上限。
            # 这车低挡上限 4.76 m/s(17 km/h)，升挡需 0.80 以上油门且必带 ~10 m/s^2 冲量，
            # 所以这里压住油门，用 17 km/h 跑完整关，换取第一个元素的加速度奖励。
            if vc.gear == GearMode.DRIVE and not (sub and "泊车" in sub and d_stage is not None and d_stage <= AIM_NEAR_M):
                lim_thr += max(-0.0015, min(0.0015, vc.throttle - lim_thr))
                lim_thr = max(0.0, min(1.0, lim_thr))
                if speed < 6.0:
                    cap = 0.02
                else:
                    cap = 0.02 + 0.06 * (speed - 6.0)
                if speed < 1.5:
                    cap = 0.006   # 离合器门槛以下，压住起步
                lim_thr = min(lim_thr, min(0.85, cap))
                vc = VehicleControl(gear=vc.gear, throttle=lim_thr, brake=vc.brake,
                                    steering=vc.steering)
            else:
                lim_thr = 0.0

            api.set_vehicle_control(vc)
            writer.writerow([
                frame, round(sim_t, 3), sub, phase, round(x, 3), round(y, 3), round(yaw, 2),
                round(speed, 3), car.gear.name, round(vc.throttle, 3), round(vc.brake, 2),
                round(vc.steering, 3),
                round(met["dist"], 3), round(met["du"], 3), round(met["dv"], 3),
                round(met["ori_err"], 2), round(met["margin"], 3), int(met["inside"]),
            ])
            if frame % 20 == 0:
                csv_fh.flush()
                print(f"[{sim_t:6.1f}s] {phase:8s} pos=({x:8.2f},{y:8.2f}) yaw={yaw:6.1f} "
                      f"v={speed:5.2f} d={met['dist']:5.2f} du={met['du']:+6.2f} "
                      f"dv={met['dv']:+6.2f} margin={met['margin']:+6.3f}", flush=True)

            if time.time() > t_end:
                ending = "timeout"
                break
    except KeyboardInterrupt:
        ending = "interrupt"
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
        csv_fh.close()

    summary = {
        "ending": ending,
        "phase": phase,
        "frames": frame,
        "stage_ready": stage_ready,
        "slot_center": None if slot is None else [round(float(slot.pos_x), 3),
                                                  round(float(slot.pos_y), 3)],
        "slot_size": None if slot is None else [round(float(slot.length), 3),
                                                round(float(slot.width), 3)],
        "final": [round(final[0], 3), round(final[1], 3), round(final[2], 2)],
        "csv": str(csv_path),
    }
    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
