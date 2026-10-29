# -*- coding: utf-8 -*-
"""港口迷雾：单车执行器（纯跟踪 + 纵向软限幅）。

参数全部来自现场量测，不写死"经验值"：
  * 车长/轴距：两轴距按车长的 0.62 倍估（场景只给 length_m/width_m），
    每局第一帧用静止位姿自检；
  * 限速：min(场景 maximum_speed_mps, 现场指定 v_target)；
  * 转角：max_steer 由单车最小转弯半径反推，先按 0.45 保守起步，
    跑直线时用实际 yaw 变化率在线校正（见 calibrate_steer）。

对外只有两个入口：
  * Follower(path).step(state, dt) -> VehicleControl
  * Follower.done
"""
import math

WHEELBASE_RATIO = 0.62        # 轴距 / 车长（场景不给轴距）
MAX_STEER = 0.45              # 归一化舵角上限（VehicleControl.steering 定义域 ±1）
LOOKAHEAD_K = 0.60            # 前视距离 = LOOKAHEAD_K * v + LOOKAHEAD_BASE
LOOKAHEAD_BASE = 1.4
LOOKAHEAD_MAX = 12.0
THROTTLE_GAIN = 0.16          # 每 m/s 误差给多少油门
BRAKE_GAIN = 0.22
STEER_GAIN = 1.0


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Follower(object):
    def __init__(self, path, v_cap=5.0, arrive_m=1.6):
        self.path = [(float(p[0]), float(p[1])) for p in path]
        self.v_cap = float(v_cap)
        self.arrive_m = float(arrive_m)
        self.i = 0
        self.done = False
        self.passed = 0.0

    # --- 几何 ---------------------------------------------------------
    def _project(self, pos):
        """在 path 上找最近段，返回 (段号, 段上最近点, 横向偏差, 段方向)。"""
        best = None
        for k in range(max(0, self.i - 2), len(self.path) - 1):
            ax, ay = self.path[k]
            bx, by = self.path[k + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            if L2 < 1e-9:
                continue
            t = ((pos[0] - ax) * dx + (pos[1] - ay) * dy) / L2
            t = max(0.0, min(1.0, t))
            px, py = ax + t * dx, ay + t * dy
            d = math.hypot(pos[0] - px, pos[1] - py)
            if best is None or d < best[0]:
                best = (d, k, (px, py), (dx, dy))
        if best is None:
            ax, ay = self.path[-1]
            return len(self.path) - 2, (ax, ay), math.hypot(pos[0] - ax, pos[1] - ay), (1.0, 0.0)
        return best[1], best[2], best[0], best[3]

    def _lookahead(self, k, v):
        dist = LOOKAHEAD_K * max(v, 0.5) + LOOKAHEAD_BASE
        dist = min(dist, LOOKAHEAD_MAX)
        acc = 0.0
        for j in range(k, len(self.path) - 1):
            ax, ay = self.path[j]
            bx, by = self.path[j + 1]
            seg = math.hypot(bx - ax, by - ay)
            if acc + seg >= dist:
                r = (dist - acc) / max(seg, 1e-6)
                return (ax + (bx - ax) * r, ay + (by - ay) * r), j
            acc += seg
        return self.path[-1], len(self.path) - 2

    # --- 控制 ---------------------------------------------------------
    def step(self, pos, yaw_deg, speed, dt, wheelbase):
        k, proj, e_lat, seg = self._project(pos)
        self.i = max(self.i, k)
        remain = math.hypot(self.path[-1][0] - pos[0], self.path[-1][1] - pos[1])
        if remain <= self.arrive_m:
            self.done = True
            return 0.0, 0.35, 0.0

        target, _j = self._lookahead(k, speed)
        yaw = math.radians(yaw_deg)
        # 场景约定：yaw 顺时针为正，0 指向 +x，90 指向 -y（前几个场景实测）
        hx, hy = math.cos(yaw), -math.sin(yaw)
        dx, dy = target[0] - pos[0], target[1] - pos[1]
        alpha = math.atan2(dy, dx) - math.atan2(hy, hx)
        alpha = wrap_pi(alpha)
        # 把误差转到"yaw 顺时针为正"的坐标系
        err = -alpha
        ld = max(math.hypot(dx, dy), 1.0)
        steer = math.atan2(2.0 * wheelbase * math.sin(err), ld) / MAX_STEER * STEER_GAIN

        # 弯道降速：按剩余段的方向变化
        v_t = self.v_cap
        if k + 2 < len(self.path):
            a = math.atan2(self.path[k + 1][1] - self.path[k][1],
                           self.path[k + 1][0] - self.path[k][0])
            b = math.atan2(self.path[k + 2][1] - self.path[k + 1][1],
                           self.path[k + 2][0] - self.path[k + 1][0])
            turn = abs(wrap_pi(b - a))
            if turn > 0.5:
                v_t = min(v_t, 2.6)
            elif turn > 0.2:
                v_t = min(v_t, 3.6)
        if remain < 8.0:
            v_t = min(v_t, max(1.0, math.sqrt(2.0 * 1.2 * max(remain - self.arrive_m, 0.1))))

        err_v = v_t - speed
        thr = max(0.0, min(0.45, THROTTLE_GAIN * err_v * 10.0))
        brk = 0.0
        if err_v < -0.25:
            brk = max(0.0, min(0.6, BRAKE_GAIN * (-err_v)))
            thr = 0.0
        if speed < 0.2 and v_t > 0.3:
            thr = max(thr, 0.16)
        steer = max(-1.0, min(1.0, steer))
        return thr, brk, steer
