# -*- coding: utf-8 -*-
r"""垂直车道泊车（场景元素 21）控制逻辑。

评分（附件一）：
  - 驶进可用车位并在车位内停稳 → 满分 200
  - 压单实线 / 双黄线 / 道路边线：每次 −20
  - 碰对手车：按碰撞瞬时速度 v 扣 5×v
  - 200 秒没完成 → 重开

坐标约定（和 tracking.py 保持一致，都是实测出来的）：
  - 场景 yaw 顺时针为正：yaw=90° 时车头朝 -y，所以数学角 = -radians(yaw)
  - steering 为正 = 往 -x 侧偏，纯跟踪算出的转角要取负号
  - 倒车时转向效果反向，所以 REVERSE 状态下再乘 -1

用法见 main.py 的 --park 分支；先跑 tools/probe_parking.py 把车位几何摸清楚再调参。
"""
from __future__ import annotations

import math

from metacar_race.config import (
    PARK_APPROACH_SPEED_MS,
    PARK_HEADING_TOL_DEG,
    PARK_REVERSE_SPEED_MS,
    PARK_STAGING_BACK_M,
    PARK_TARGET_TOL_M,
)

# 障碍物类型里车位是 30（ObstacleType.PARKING_SLOT）
PARKING_SLOT_TYPE = 30


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _heading_rad(yaw_deg: float) -> float:
    """场景 yaw（顺时针）转成数学角。"""
    return math.radians(-yaw_deg)


def find_slots(obstacles):
    """从 ObstacleInfo 列表里挑出车位。"""
    slots = []
    for ob in obstacles:
        t = getattr(ob, "type", None)
        value = getattr(t, "value", t)
        if value == PARKING_SLOT_TYPE:
            slots.append(ob)
    return slots


def pick_target(slots, pose, prefer_right: bool = True):
    """挑一个目标车位：距离最近，且大致在车头前方或侧方。"""
    if not slots:
        return None

    best, best_score = None, float("inf")
    for s in slots:
        dx, dy = s.pos_x - pose.pos_x, s.pos_y - pose.pos_y
        dist = math.hypot(dx, dy)
        if dist < 0.5:
            continue
        # 车体坐标下的相对位置
        yaw = _heading_rad(pose.ori_z)
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        # 侧方满分，后方次之，前方扣分
        side_penalty = 0.0
        if local_x > 3.0:
            side_penalty = 8.0
        if prefer_right and local_y > 0:
            side_penalty += 2.0
        score = dist + side_penalty
        if score < best_score:
            best, best_score = s, score
    return best


def slot_frame(slot):
    """返回车位中心与车位轴线朝向（弧度，数学角）。"""
    return (
        (slot.pos_x, slot.pos_y),
        _heading_rad(getattr(slot, "ori_z", 0.0)),
    )


def staging_pose(slot, back=PARK_STAGING_BACK_M):
    """车位外侧的对位点：沿车位轴线往车道方向退 back 米。"""
    (cx, cy), yaw = slot_frame(slot)
    # 车位朝向指向"车位纵深"，退出来就是车道
    return (cx - math.cos(yaw) * back, cy - math.sin(yaw) * back), yaw


class ParkingManeuver:
    """三段式：走到对位点 → 对齐车位朝向 → 倒车入库 → 驻车保持。"""

    APPROACH = "approach"
    ALIGN = "align"
    REVERSE = "reverse"
    HOLD = "hold"

    def __init__(self):
        self.phase = self.APPROACH
        self.target = None
        self.elapsed = 0.0

    def reset(self):
        self.phase = self.APPROACH
        self.target = None
        self.elapsed = 0.0

    # ---------- 内部工具 ----------
    @staticmethod
    def _steer_to(target_xy, pose, reverse=False):
        dx = target_xy[0] - pose.pos_x
        dy = target_xy[1] - pose.pos_y
        # 倒车时目标在车后，用反向的朝向误差
        desired = math.atan2(-dy, -dx) if reverse else math.atan2(dy, dx)
        err = _wrap(desired - _heading_rad(pose.ori_z))
        # 正的 steering 往 -x 侧偏，所以取负；倒车再反一次
        k = -1.6 if not reverse else 1.6
        return max(-1.0, min(1.0, k * err))

    @staticmethod
    def _dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _heading_err(slot_yaw, pose):
        return abs(math.degrees(_wrap(slot_yaw - _heading_rad(pose.ori_z))))

    # ---------- 主循环 ----------
    def update(self, obstacles, pose, v_now, dt):
        """返回 (gear, throttle, brake, steering, phase)。

        gear 用字符串 'D' / 'R' / 'P'，由调用方映射到 GearMode。
        """
        self.elapsed += dt

        if self.target is None:
            self.target = pick_target(find_slots(obstacles), pose)
            if self.target is None:
                # 还没看到车位：慢慢往前挪，边挪边找
                return "D", 0.25, 0.0, 0.0, "search"

        (cx, cy), slot_yaw = slot_frame(self.target)
        center = (cx, cy)
        dist_to_center = self._dist(center, (pose.pos_x, pose.pos_y))
        heading_err = self._heading_err(slot_yaw, pose)

        # ---- 入库完成判定 ----
        if dist_to_center <= PARK_TARGET_TOL_M and heading_err <= PARK_HEADING_TOL_DEG:
            self.phase = self.HOLD
            return "P", 0.0, 1.0, 0.0, self.phase

        # ---- 阶段 1：开到对位点 ----
        if self.phase == self.APPROACH:
            staging, _ = staging_pose(self.target)
            dist_staging = self._dist(staging, (pose.pos_x, pose.pos_y))
            if dist_staging < 1.2:
                self.phase = self.ALIGN
            else:
                steer = self._steer_to(staging, pose)
                return "D", PARK_APPROACH_SPEED_MS, 0.0, steer, self.phase

        # ---- 阶段 2：原地摆正，指向车位 ----
        if self.phase == self.ALIGN:
            if heading_err <= 8.0:
                self.phase = self.REVERSE
            else:
                # 低速前后挪一点配合转向，比原地硬打好
                steer = self._steer_to(center, pose, reverse=True)
                return "D", 0.12, 0.0, steer, self.phase

        # ---- 阶段 3：倒车入库 ----
        if self.phase == self.REVERSE:
            # 倒车时目标就是车位中心
            steer = self._steer_to(center, pose, reverse=True)
            # 快到位就松油
            throttle = PARK_REVERSE_SPEED_MS if dist_to_center > 1.5 else 0.08
            return "R", throttle, 0.0, steer, self.phase

        return "P", 0.0, 1.0, 0.0, self.phase
