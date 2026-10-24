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


def find_slot(obstacles):
    for obstacle in obstacles:
        if int(obstacle.type.value) == PARKING_SLOT_TYPE:
            return obstacle
    return None


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
    parser.add_argument("--skip-to-index", type=int, default=5,
                        help="只跑第 N 个元素（垂直泊车=5），其余自动 skip_level")
    parser.add_argument("--vmax", type=float, default=8.0, help="沿路线行驶的最高速度")
    parser.add_argument("--stage-past", type=float, default=STAGE_PAST_M)
    parser.add_argument("--hold", type=float, default=8.0, help="入库后保持驻车秒数")
    parser.add_argument("--max-seconds", type=float, default=260.0)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"已连接。元素 {len(static.sub_scenes)} 个，赛道 {len(static.route)} 点", flush=True)

    target = ""
    elem_start = None
    if 1 <= args.skip_to_index <= len(static.sub_scenes):
        _elem = static.sub_scenes[args.skip_to_index - 1]
        target = _elem.name
        if _elem.start_point is not None:
            elem_start = (float(_elem.start_point.x), float(_elem.start_point.y))
        print(f"目标元素：{target}（其余自动跳关）", flush=True)

    pursuit = PurePursuit(static.route, lookahead=LOOKAHEAD_M) if len(static.route) >= 2 else None
    profile = SpeedProfile(static.route, v_max=args.vmax) if len(static.route) >= 3 else None
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

            slot_now = find_slot(msg.obstacles)
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
            heading_error = wrap_deg(float(slot.ori_z) - yaw)

            # 倒车准备点：车位中心沿"出库方向"5.64 m、沿行进方向再退 5.57 m（与基线同构）
            ux, uy, vx, vy = slot_axes(slot)
            stage = (float(slot.pos_x) + ux * STAGE_LATERAL_M - vx * args.stage_past,
                     float(slot.pos_y) + uy * STAGE_LATERAL_M - vy * args.stage_past)
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

            if phase == "approach":
                if remaining <= STOP_AT_STAGE_M or d_stage <= STOP_AT_STAGE_M:
                    if speed <= STAGE_STILL_SPEED_MS:
                        vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                        stationary += dt
                    else:
                        vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                            brake=0.12, steering=0.0)
                        stationary = 0.0
                    if stationary >= STAGE_SETTLE_SECONDS:
                        stage_ready = True
                        phase = "reverse"
                        stationary = 0.0
                        print(f"到达倒车准备点 ({x:.2f},{y:.2f}) yaw={yaw:.1f} "
                              f"剩余={remaining:+.2f} 距准备点={d_stage:.2f} "
                              f"du={met['du']:+.2f} dv={met['dv']:+.2f}", flush=True)
                elif d_stage < AIM_NEAR_M:
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
                if (met["dist"] <= CENTER_TOL_M and abs(heading_error) <= HEADING_TOL_DEG
                        and abs(speed) <= STAGE_STILL_SPEED_MS):
                    phase = "hold"
                    hold = 0.0
                    vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                    print(f"入库条件满足 d={met['dist']:.3f} dv={met['dv']:+.3f} "
                          f"yaw_error={heading_error:+.2f} margin={met['margin']:+.3f}", flush=True)
                else:
                    steering = max(-1.0, min(1.0, REVERSE_STEER_SIGN * heading_error / 30.0))
                    if met["dist"] <= REVERSE_STOP_START_M:
                        throttle, brake = 0.0, REVERSE_BRAKE
                        if abs(speed) <= 0.10:
                            throttle, brake = REVERSE_CREEP_THROTTLE, 0.0
                    else:
                        throttle, brake = REVERSE_THROTTLE, 0.0
                        if abs(speed) > REVERSE_MAX_SPEED_MS:
                            throttle, brake = 0.0, REVERSE_BRAKE
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
