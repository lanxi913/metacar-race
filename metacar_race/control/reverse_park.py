# -*- coding: utf-8 -*-
"""倒车入库状态机（移植自 tools/park_v8.py，那里拿过垂直泊车 200 分）。

几何原理：满舵倒车的转弯半径 R 是车辆属性（实测 5.26 m）。只要把车摆到
「车位中心 + 沿车轴前 R + 横向 -side*R」这个对位点，倒车打满舵转 90°，
车就会精确落在车位中心，**不需要任何经验补偿**。代价是这个点通常离车位
轴线 5.26 m —— 比一条车道宽，所以进车位前往往要先往路的另一侧借位。

用法：
    park = ReversePark()
    park.begin(pose, slot, now)      # 拿到车位后调用一次
    gear, thr, brk, steer = park.update(pose, v, now)   # 每帧调用，gear 是字符串
    park.done                        # 保持够 HOLD_SECONDS 后为 True

油门/刹车的斜坡与软起步**不在这里做**，交给 OutputLimiter，保持和跟车同一个出口。
"""
from __future__ import annotations

import math

PARKING_SLOT_TYPE = 30

DEFAULT_RADIUS = 5.26          # 倒车弧半径（staging 点的几何基准）
APPROACH_SPEED = 2.30          # 对位段的目标车速（m/s）
APPROACH_DECEL = 0.80          # 对位段规划减速度（m/s²）
STAGE_SPEED = 0.14             # 兼容旧脚本：对位段的油门量（≈2.3 m/s）
ARC_SPEED = 0.55               # 倒车弧段的目标速度
STOPPED_SPEED = 0.30           # 认为「停稳」的车速
STOP_LONG_TOL = 0.06           # 沿轴线距车位中心多近算入库
STOP_LAT_TOL = 0.15            # 横向容差（只用于日志判定）
HOLD_SECONDS = 5.0             # 连续场景要求停稳保持 5 s
ARC_TOL_RAD = math.radians(2.0)
ARC_STEER = 0.74               # 倒车弧的转向量：steering=1.0 实测半径只有 3.61 m
TIMEOUT_SECONDS = 120.0
GOAL_TOL = 0.08                # 距「停车检查点」多近算到位（平台 end_point）
GOAL_MAX_OFFSET = 1.50         # 检查点离车位中心超过这个距离就不当停车点用
GOAL_MAX_LATERAL = 0.60
GOAL_STOP_TOL = 0.05           # 沿车位轴看，离检查点多远就停车（二维距离永远到不了 0）
GOAL_STOP_LOOKAHEAD = 0.10      # 提前量：rem <= tol + k*|v| 就判到位，抵消平台响应延迟


def wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def heading_rad(yaw_deg: float) -> float:
    """场景 yaw 顺时针为正 → 数学角。"""
    return math.radians(-float(yaw_deg))


def find_slot(obstacles):
    """在障碍物里找泊车位（类型 30）。"""
    for obs in obstacles or ():
        kind = getattr(obs, "type", None)
        value = getattr(kind, "value", kind)
        try:
            if int(value) == PARKING_SLOT_TYPE:
                return obs
        except (TypeError, ValueError):
            continue
    return None


def speed_cmd(v_meas: float, v_target: float):
    """简单的速度→油门/刹车映射（和 park_v8 一致，上限 0.10）。"""
    err = v_target - abs(v_meas)
    if abs(err) < 0.06:
        return 0.0, 0.0
    if err > 0:
        return min(0.10, 0.01 + 0.04 * err), 0.0
    return 0.0, min(0.10, -0.06 * err)


class ReversePark:
    """倒车入库：approach → stop → arc → straight → hold。"""

    def __init__(self, radius=DEFAULT_RADIUS, hold_seconds=HOLD_SECONDS,
                 timeout=TIMEOUT_SECONDS, goal=None, goal_tol=GOAL_TOL,
                 arc_steer=ARC_STEER):
        self.radius = float(radius)
        self.hold_seconds = float(hold_seconds)
        self.timeout = float(timeout)
        # 倒车弧的转向量。20261024-142117 实测：steering=1.0 倒车画出来的半径只有
        # 3.61 m（最小二乘拟合残差 4 mm，转角 80.3°、弧长 5.06 m），而 staging 点是
        # 按 5.26 m 算的，两者差 1.65 m —— 弧末端横向误差 0.73 m 全从这里来。
        # 反推满舵前轮角 35.7°（≈0.623 rad），要凑出 5.26 m 需 steering≈0.74。
        self.arc_steer = float(arc_steer)
        # 平台给的「停车检查点」（static.sub_scenes[i].end_point）。车必须停到这
        # 个点上，元素才算完成——20261024-133827 那次停在车位中心，离检查点
        # 0.40 m，垂直泊车直接 0 分，430 分里少了 200。
        self.goal = None if goal is None else (float(goal[0]), float(goal[1]))
        self.goal_tol = float(goal_tol)
        self.goal_dist = float("inf")
        self.goal_a_long = 0.0
        self.goal_a_lat = 0.0
        self.phase = "approach"
        self.done = False
        self.slot = None
        self.frame = {}
        self.stage = (0.0, 0.0)
        self.t_begin = 0.0
        self.t_hold = 0.0
        self.d_stage = float("inf")
        self.a_long = 0.0
        self.a_lat = 0.0
        self.heading_err = 0.0
        self.lat_sign = 1.0
        self._lat_prev = None
        self._lat_check_t = 0.0
        self._gear = "drive"

    # ------------------------------------------------------------------ #
    def begin(self, x, y, yaw_deg, slot, now) -> dict:
        """锁定车位坐标系，算对位点。返回对位点信息（写日志用）。"""
        self.slot = slot
        th = heading_rad(getattr(slot, "ori_z", 0.0))
        ux, uy = math.cos(th), math.sin(th)
        vx, vy = -math.sin(th), math.cos(th)
        sx, sy = float(slot.pos_x), float(slot.pos_y)

        rel_lat = (x - sx) * vx + (y - sy) * vy
        side = 1.0 if rel_lat >= 0 else -1.0

        stage_lat = -side * self.radius
        self.stage = (sx + ux * self.radius + vx * stage_lat,
                      sy + uy * self.radius + vy * stage_lat)
        self.frame = dict(ux=ux, uy=uy, vx=vx, vy=vy, side=side, th=th)
        # 检查点必须落在车位轴线附近（平台给的就是车位里那个点，实测离中心 0.40 m）。
        # 离得太远的「检查点」不是停车点（比如元素终点在本段路口），按车位中心停。
        if self.goal is not None:
            ga = (self.goal[0] - sx) * ux + (self.goal[1] - sy) * uy
            gb = (self.goal[0] - sx) * vx + (self.goal[1] - sy) * vy
            self.goal_a_long = ga
            self.goal_a_lat = gb
            if abs(ga) > GOAL_MAX_OFFSET or abs(gb) > GOAL_MAX_LATERAL:
                self.goal = None
        self.t_begin = float(now)
        self.phase = "approach"
        self.done = False
        self._gear = "drive"
        return self.info(x, y, yaw_deg)

    def info(self, x=0.0, y=0.0, yaw_deg=0.0) -> dict:
        """当前几何量（不推进状态），日志和调试都用它。"""
        if not self.frame:
            return {"phase": self.phase, "d_stage": self.d_stage,
                    "a_long": 0.0, "a_lat": 0.0, "heading_err": 0.0,
                    "stage_x": 0.0, "stage_y": 0.0}
        f = self.frame
        sx, sy = float(self.slot.pos_x), float(self.slot.pos_y)
        rx, ry = x - sx, y - sy
        a_long = rx * f["ux"] + ry * f["uy"]
        a_lat = rx * f["vx"] + ry * f["vy"]
        return {
            "phase": self.phase,
            "d_stage": math.hypot(self.stage[0] - x, self.stage[1] - y),
            "a_long": a_long,
            "a_lat": a_lat,
            "heading_err": wrap(f["th"] - heading_rad(yaw_deg)),
            "stage_x": self.stage[0],
            "stage_y": self.stage[1],
        }

    # ------------------------------------------------------------------ #
    def update(self, x, y, yaw_deg, v, now):
        """推进一帧。返回 (gear, throttle, brake, steering)，gear ∈ drive/reverse/parking。"""
        if self.done or not self.frame:
            return "parking", 0.0, 1.0, 0.0

        f = self.frame
        ux, uy, vx, vy = f["ux"], f["uy"], f["vx"], f["vy"]
        side, th_slot = f["side"], f["th"]

        sx, sy = float(self.slot.pos_x), float(self.slot.pos_y)
        rx, ry = x - sx, y - sy
        a_long = rx * ux + ry * uy
        a_lat = rx * vx + ry * vy
        heading_err = wrap(th_slot - heading_rad(yaw_deg))
        self.a_long, self.a_lat, self.heading_err = a_long, a_lat, heading_err

        stage_x, stage_y = self.stage
        d_stage = math.hypot(stage_x - x, stage_y - y)
        self.d_stage = d_stage

        # ---------- 1) 开到对位点 ----------
        # 原版（park_v8）在 20 m 内直接收油靠滑行，滑多远取决于车辆阻力，换个车位
        # 距离就变味。这里改成按距离给速度指令：v = sqrt(2*a*d)，最后一米爬行。
        if self.phase == "approach":
            # 对位点误差会原封不动传进车位：20261024-141009 那次在 d_stage=0.45 就
            # 换挡，倒车弧的起点横向差 0.36 m，最后停在离检查点 0.27 m 的地方。
            # 现在要求贴到 0.12 m 再停，并且最后几米把方向打得更狠。
            if d_stage < 0.12:
                self.phase = "stop"
                return "drive", 0.0, 0.0, 0.0
            v_des = min(APPROACH_SPEED,
                        max(0.14, math.sqrt(2.0 * APPROACH_DECEL
                                           * max(d_stage - 0.10, 0.02))))
            thr, brk = speed_cmd(v, v_des)
            # speed_cmd 的刹车上限 0.10 是 park_v8 里「20 m 外收油靠滑行」的遗留，
            # 现在换成按距离给速度，刹车必须真的能把速度压下去，否则对位点会冲过头。
            if v > v_des + 0.20:
                need = (v * v - v_des * v_des) / (2.0 * max(d_stage - 0.20, 0.10))
                brk = max(brk, min(0.45, need / 8.0))
                thr = 0.0
            err_h = wrap(math.atan2(stage_y - y, stage_x - x) - heading_rad(yaw_deg))
            gain = 1.5 if d_stage > 4.0 else 2.5
            steer = max(-1.0, min(1.0, -gain * err_h))
            return "drive", thr, brk, steer

        # ---------- 2) 停稳换倒挡 ----------
        if self.phase == "stop":
            if abs(v) < STOPPED_SPEED:
                self.phase = "arc"
                return "reverse", 0.0, 0.0, side * self.arc_steer
            return "drive", 0.0, 0.10, 0.0

        # ---------- 3) 定半径倒车，车头摆到车位轴线 ----------
        if self.phase == "arc":
            if abs(heading_err) < ARC_TOL_RAD:
                self.phase = "straight"
                return "reverse", 0.0, 0.0, 0.0
            thr, brk = speed_cmd(v, ARC_SPEED)
            return "reverse", thr, brk, side * self.arc_steer

        # ---------- 4) 直线倒进车位 ----------
        if self.phase == "straight":
            if self._lat_prev is None:
                self._lat_prev, self._lat_check_t = a_lat, now
            elif now - self._lat_check_t > 0.6:
                if abs(a_lat) > abs(self._lat_prev) + 0.01:
                    self.lat_sign = -self.lat_sign
                self._lat_prev, self._lat_check_t = a_lat, now

            rem = a_long - (self.goal_a_long if self.goal is not None else 0.0)
            if self.goal is not None:
                # 检查点不在车位轴线的正投影上（实测偏 0.13~0.16 m），二维距离永远
                # 收不到 GOAL_TOL，只能看「沿轴还剩多远」，再按车速留一点提前量。
                gd = math.hypot(self.goal[0] - x, self.goal[1] - y)
                self.goal_dist = gd
                if rem <= GOAL_STOP_TOL + GOAL_STOP_LOOKAHEAD * abs(v):
                    self.phase = "hold"
                    self.t_hold = float(now)
                    return "parking", 0.0, 0.35, 0.0
            elif abs(a_long) < STOP_LONG_TOL or a_long < -0.45:
                self.phase = "hold"
                self.t_hold = float(now)
                return "parking", 0.0, 0.35, 0.0

            if self.goal is not None:
                # 越接近检查点越慢：0.45 m/s per metre，最后 0.2 m 只剩 0.13 m/s
                vt = max(0.05, min(0.80, 0.45 * max(rem, 0.0) + 0.04))
            else:
                vt = max(0.15, min(0.80, 1.1 * max(abs(a_long) - 0.10, 0.0) + 0.22))
            thr, brk = speed_cmd(v, vt)
            if abs(v) > vt + 0.06:
                # 倒挡不给油也会一直滑（实测 0.5 m/s 不衰减），必须主动刹车
                thr = 0.0
                brk = max(brk, min(0.30, 0.45 * (abs(v) - vt)))
            lat_err = a_lat - self.goal_a_lat if self.goal is not None else a_lat
            # 靠近检查点时横向纠偏要更硬：实测 6.5 s 只纠回 0.07 m，收得太慢
            lat_gain = 0.6 if rem > 3.0 else 1.3
            steer = max(-0.55, min(0.55,
                                   1.8 * heading_err + self.lat_sign * lat_gain * lat_err))
            return "reverse", thr, brk, steer

        # ---------- 5) 驻车保持 ----------
        ramp = min(1.0, 0.22 + 0.55 * (now - self.t_hold))
        if now - self.t_hold > self.hold_seconds:
            self.done = True
        return "parking", 0.0, ramp, 0.0
