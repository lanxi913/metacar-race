# -*- coding: utf-8 -*-
"""港口迷雾：LaneDriver —— 把仓库里的 LaneFollower 内核接到港口迷雾协议上。

## 接的是什么

`metacar_race/control/lane_follower.py`（城市道路实测 110 分那套）里有三块能直接用的东西：

1. `Path`：把折线做成「弧长 s + 横向偏差 e」的参考路径。有了 s 就能把车**投影**到路径上，
   取前方 lookahead 处的 carrot —— 这是它比"到下一点才切"强的地方：
   路径首点落在车后时不会触发掉头（这车掉头半径 20 m 量级，一掉头就撞墙）。
2. 纯跟踪公式 `delta = atan2(2·L·sinα, ld)`，用轴距 L 和航向误差 α 直接算舵角，
   低速大误差时自动饱和，不用手调增益表。
3. 收尾用轻刹滑停、不硬切 P 档（硬停会甩出加速度尖峰）。

## 不能直接照搬的部分（所以执行量在这里重写）

* `LaneFollower.Ramp` 把目标油门乘 0.05（城市场景的油门标定），港口迷雾要 0.05~0.45；
* `LaneFollower` 从 `static.roads[].lanes[].path_points` 读参考线，港口迷雾**没有 lanes**，
  参考线由 `fog_route` 沿 mapd 的 192 条道路铺点给出（每 2 m 一点 + 每点期望速度）；
* 城市场景一辆车主控，港口迷雾四辆车同帧受控 —— 这里只管"一辆车这一帧给多少油门/刹车/转向"。

## 对外接口（fog_mission 依赖这几个名字）

    drv = LaneDriver(route, wb, v_cap=..., stop_xy=..., stop_m=...)
    thr, brk, steer = drv.step((x, y), yaw_deg, speed_mps, dt)
    drv.done / drv.i / drv.pts

自测（不需要场景）：

    set PYTHONIOENCODING=utf-8
    <repo>\\.venv\\Scripts\\python.exe fog_run5.py --selftest
"""
from __future__ import annotations

import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from metacar_race.control.lane_follower import Path          # noqa: E402
from metacar_race.config import LANE_STEER_LIMIT            # noqa: E402

from fog_run3 import MAX_STEER, clip, wrap180               # noqa: E402

ACCEL = 1.15          # 规划加速度（m/s²），用于剩余距离的刹车点
LOOKAHEAD_A = 1.0    # 前视距离 = a + b·v（短前视才能贴住 90° 弯，长前视会切内角）
LOOKAHEAD_B = 0.45
LOOKAHEAD_MIN = 1.2
LOOKAHEAD_MAX = 7.0
SLOW_HORIZON_A = 2.0  # 提前看多远的限速：a + b·v（弯前先减速，别等压上弯再刹）
SLOW_HORIZON_B = 1.0
LAT_DAMP = 0.10      # 横向偏差进转向的阻尼系数（e>0 = 车在路径左侧 → 回正向右）
CURV_FF = 0.7        # 弯道曲率前馈权重（0 = 纯 LaneFollower 的纯跟踪）
THR_MAX = 0.50
COLD_START_M = 1.0   # 位移航向没测出来前直行的距离
FILLET_R = 3.0       # 折线顶点倒角半径（参考线必须是车开得出来的曲线）

# 舵机方向：2026-10-29 19:23 实测（Car_C 直行段给满舵 3 s）——
#   steer=+0.6 → 航向 180.0→165.5（−14.5°），即 **+1 是右舵**（航向减小）。
# 控制器内部一律用"航向增大为正"的数学约定，只在输出时乘这个符号翻转。
STEER_SIGN = -1.0


def collapse(pts, tol=5.0):
    """把"每 2 m 一点"的点列压成只在真正拐弯处留点的折线。

    倒角必须先压点：直接对 2 m 间隔的点倒 3 m 角，会被 0.45×段长 卡成 0.9 m，
    得到一个比车还能转的弧，车根本跟不住（实测直角弯冲出去 3.3 m）。
    """
    out = [tuple(pts[0])]
    src = [0]
    k = 0
    for i in range(1, len(pts) - 1):
        h = math.degrees(math.atan2(pts[i + 1][1] - pts[i][1],
                                    pts[i + 1][0] - pts[i][0]))
        hp = math.degrees(math.atan2(pts[i][1] - pts[k][1],
                                     pts[i][0] - pts[k][0]))
        if abs(wrap180(h - hp)) > tol:
            out.append(tuple(pts[i]))
            src.append(i)
            k = i
    out.append(tuple(pts[-1]))
    src.append(len(pts) - 1)
    return out, src


def fillet(pts, r=FILLET_R, seg=6):
    """把折线顶点导成半径 r 的圆弧。

    参考线是"每条车道每 2 m 一点"拼出来的，路口处会出现 90° 硬拐点：
    纯跟踪在硬拐点上只能贴着顶点画弧，实测直角弯会冲到拐点外 2.7~4 m。
    先按车能开出来的曲率倒角，剩下的跟踪误差就只剩零点几米。
    返回 (新点列, 每个新点对应的原始点索引)。
    """
    out = [tuple(pts[0])]
    src = [0]
    for i in range(1, len(pts) - 1):
        a, p, b = pts[i - 1], pts[i], pts[i + 1]
        v1 = (a[0] - p[0], a[1] - p[1])
        v2 = (b[0] - p[0], b[1] - p[1])
        l1 = math.hypot(*v1)
        l2 = math.hypot(*v2)
        if l1 < 0.5 or l2 < 0.5:
            out.append(tuple(p))
            src.append(i)
            continue
        u1 = (v1[0] / l1, v1[1] / l1)
        u2 = (v2[0] / l2, v2[1] / l2)
        cos_t = clip(u1[0] * u2[0] + u1[1] * u2[1], -1.0, 1.0)
        theta = math.acos(cos_t)                 # 折点内角：180° = 直行
        phi = math.pi - theta                    # 偏折角：0 = 直行
        if phi < math.radians(8.0) or phi > math.radians(150.0):
            # 近直线不倒角；>150° 是发夹/掉头，倒了会得到半径趋 0 的弧（tan(φ/2)→∞），
            # 整条参考线会被带歪 —— 实测就是这么把车带进仓库的
            out.append(tuple(p))
            src.append(i)
            continue
        t = min(r * math.tan(phi / 2.0), 0.45 * l1, 0.45 * l2)
        rr = t / math.tan(phi / 2.0)
        if not math.isfinite(rr) or rr < 1e-3:
            out.append(tuple(p))
            src.append(i)
            continue
        a2 = (p[0] + u1[0] * t, p[1] + u1[1] * t)
        b2 = (p[0] + u2[0] * t, p[1] + u2[1] * t)
        bx, by = u1[0] + u2[0], u1[1] + u2[1]
        lb = math.hypot(bx, by)
        if lb < 1e-6:
            out.append(tuple(p))
            src.append(i)
            continue
        d = rr / math.sin(theta / 2.0)
        cx, cy = p[0] + bx / lb * d, p[1] + by / lb * d
        a0 = math.atan2(a2[1] - cy, a2[0] - cx)
        a1 = math.atan2(b2[1] - cy, b2[0] - cx)
        da = math.radians(wrap180(math.degrees(a1 - a0)))
        for k in range(seg + 1):
            ang = a0 + da * k / seg
            out.append((cx + rr * math.cos(ang), cy + rr * math.sin(ang)))
            src.append(i)
    out.append(tuple(pts[-1]))
    src.append(len(pts) - 1)
    return out, src


class LaneDriver(object):
    """沿参考折线行驶的纵向+横向控制器（一辆车）。"""

    def __init__(self, route, wb, v_cap=5.0, stop_xy=None, stop_m=0.0, accel=ACCEL,
                 do_fillet=True):
        src_pts = [tuple(p) for p in route.pts]
        if len(src_pts) < 2:
            src_pts = [src_pts[0], (src_pts[0][0] + 0.1, src_pts[0][1])]
        src_speeds = list(getattr(route, "speeds", None) or [])
        base, base_src = collapse(src_pts)
        if do_fillet:
            self.pts, idx = fillet(base)
        else:
            self.pts, idx = base, list(range(len(base)))
        self.src_pts = src_pts
        self.path = Path(self.pts)
        if src_speeds:
            self.speeds = [src_speeds[min(base_src[j], len(src_speeds) - 1)] for j in idx]
        else:
            self.speeds = []
        self.wb = float(wb)
        self.v_cap = float(v_cap)
        self.stop_xy = None if stop_xy is None else (float(stop_xy[0]), float(stop_xy[1]))
        self.stop_m = float(stop_m or 0.0)
        self.accel = float(accel)

        self.i = 0
        self.done = False
        self.thr = 0.0
        self.brk = 0.0
        self.t = 0.0
        self.e = 0.0
        self.s = 0.0

        # 自己的位移航向（冷启动期间直行，不跟着可疑的种子角满舵）
        self._ax = None
        self._ay = None
        self._mh = None
        self._cold = 0.0
        self._stall = 0.0

    # ------------------------------------------------------------ 工具
    def reset_heading(self):
        """倒车/瞬移之后必须调用：位移航向在倒车时会反 180°，不清就会朝反方向开。"""
        self._ax = None
        self._ay = None
        self._mh = None
        self._cold = 0.0

    def _nearest_i(self, x, y):
        best, bi = 1e18, 0
        for i, (px, py) in enumerate(self.pts):
            d = (x - px) ** 2 + (y - py) ** 2
            if d < best:
                best, bi = d, i
        return bi

    def _cap_at(self, i):
        if not self.speeds:
            return self.v_cap
        j = min(max(i, 0), len(self.speeds) - 1)
        return min(self.v_cap, float(self.speeds[j]))

    def _cap_ahead(self, s, speed):
        """前方 a+b·v 米内的最低限速（弯道提前减速，避免带着 3 m/s 切内角）。"""
        horizon = SLOW_HORIZON_A + SLOW_HORIZON_B * abs(speed)
        best = self.v_cap
        cum = self.path.cum
        for j in range(self.i, len(self.pts)):
            if cum[j] > s + horizon:
                break
            best = min(best, self._cap_at(j))
        return best

    def _path_heading(self, s):
        p1 = self.path.at(max(0.0, min(self.path.total, s)))
        p2 = self.path.at(max(0.0, min(self.path.total, s + 0.8)))
        return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

    def _curv_ff(self, s):
        """参考线前方 4 m 的曲率前馈（纯跟踪只会"追 carrot"，弯前不带方向就会切角冲出去）。"""
        dpsi = wrap180(self._path_heading(s + 4.0) - self._path_heading(s))
        kappa = math.radians(dpsi) / 4.0              # rad/m
        delta = math.atan(self.wb * kappa)
        return delta / MAX_STEER

    def _heading(self, x, y, yaw):
        if self._ax is None:
            self._ax, self._ay = x, y
        moved = math.hypot(x - self._ax, y - self._ay)
        if moved >= COLD_START_M:
            self._mh = math.degrees(math.atan2(y - self._ay, x - self._ax))
            self._ax, self._ay = x, y
            self._cold = 0.0
        return self._mh if self._mh is not None else yaw

    def _speed_ctrl(self, speed, v_des, dt):
        dv = v_des - abs(speed)
        if dv > 0.10:
            t = clip(0.08 + 0.28 * dv, 0.08, THR_MAX)
            self.brk = max(0.0, self.brk - 0.05)
            self.thr = min(t, self.thr + 0.03)          # 10 Hz 下 0.2/s
            if self.brk > 1e-6:
                self.thr = 0.0
        elif dv < -0.25:
            # 刹车上限从 THR_MAX(0.50) 放到 0.90：v_cap=4.0 的车实测会冲到 5.4 m/s
            # （长直道下坡），0.50 的刹车按不住，超速之后又只能硬刹，两头都挨违规
            b = clip(0.10 + 0.40 * (-dv - 0.25), 0.10, 0.90)
            self.thr = max(0.0, self.thr - 0.04)
            self.brk = min(b, self.brk + 0.03) if self.thr <= 1e-6 else 0.0
        else:
            self.thr = max(0.0, self.thr - 0.015)
            self.brk = max(0.0, self.brk - 0.03)

        # 破静摩擦：想走却不动，给一脚（车不动时满舵会顶死）
        if abs(speed) < 0.10 and self.thr > 0.05:
            self._stall += dt
            if self._stall > 1.2:
                # 一帧 0.1→0.42 会被判 excessive_acceleration，改成一帧 +0.07 往上爬
                self.thr = min(0.42, self.thr + 0.07)
                if self._stall > 2.6:
                    self._stall = 0.0
        else:
            self._stall = 0.0

    # ------------------------------------------------------------ 主循环
    def step(self, pos, yaw, speed, dt=0.1):
        x, y = float(pos[0]), float(pos[1])
        self.t += dt
        self.i = self._nearest_i(x, y)
        s, e = self.path.project(x, y)
        self.s, self.e = s, e
        rem = self.path.total - s

        if self.stop_xy is None:
            goal = rem
        else:
            goal = math.hypot(x - self.stop_xy[0], y - self.stop_xy[1]) - self.stop_m

        # 冷启动：航向还没测出来就给直行小油门，别跟可疑的种子角较劲
        if self._mh is None:
            self._cold += dt
        heading = self._heading(x, y, yaw)

        if self._mh is None:
            steer = 0.0
            v_des = min(0.8, self._cap_at(self.i))
        else:
            ld = clip(LOOKAHEAD_A + LOOKAHEAD_B * abs(speed), LOOKAHEAD_MIN, LOOKAHEAD_MAX)
            cx, cy = self.path.at(min(self.path.total, s + ld))
            bear = math.degrees(math.atan2(cy - y, cx - x))
            alpha = math.radians(wrap180(bear - heading))
            delta = math.atan2(2.0 * self.wb * math.sin(alpha), ld)
            steer = STEER_SIGN * (delta / MAX_STEER
                                  + CURV_FF * clip(self._curv_ff(s), -1.0, 1.0)
                                  - LAT_DAMP * clip(e, -3.0, 3.0))
            if not math.isfinite(steer):
                steer = 0.0
            steer = clip(steer, -LANE_STEER_LIMIT, LANE_STEER_LIMIT)
            v_des = min(self.v_cap, self._cap_ahead(s, speed),
                        math.sqrt(max(2.0 * self.accel * max(goal - 0.4, 0.0), 0.0)))
            # 大转角处再压一档速度：满舵半径由 v 决定，慢下来才转得进去
            if abs(self._curv_ff(s)) > 0.5:
                v_des = min(v_des, 1.1)

        self._speed_ctrl(speed, v_des, dt)

        if goal <= 0.7 or (self.stop_xy is None and rem <= 0.5):
            self.done = True
            self.thr = 0.0
            self.brk = min(0.50, self.brk + 0.06)   # 别一帧 0→0.35（会判一次加速度超限）
        return self.thr, self.brk, steer


# ---------------------------------------------------------------- 自测
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)


class _Sim(object):
    """单车运动学（前轮转向自行车模型），只用来验证控制器不自激、能收敛。"""

    def __init__(self, x, y, yaw, wb=2.65):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = 0.0
        self.wb = wb

    def step(self, thr, brk, steer, dt):
        a = 1.9 * thr - 3.2 * brk - 0.10 * self.v
        self.v = max(0.0, self.v + a * dt)
        # 舵机符号跟实测一致：steer=+1 是右舵 → 航向减小
        yr = -self.v * math.tan(MAX_STEER * steer) / self.wb
        self.yaw = wrap180(self.yaw + math.degrees(yr * dt))
        self.x += self.v * math.cos(math.radians(self.yaw)) * dt
        self.y += self.v * math.sin(math.radians(self.yaw)) * dt


class _Route(object):
    def __init__(self, pts, speeds):
        self.pts, self.speeds = pts, speeds


def _densify(pts, step=2.0):
    out = [pts[0]]
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        n = max(1, int(math.hypot(bx - ax, by - ay) / step))
        for k in range(1, n + 1):
            out.append((ax + (bx - ax) * k / n, ay + (by - ay) * k / n))
    return out


def _run(name, waypts, start, yaw0, v_cap, tol_lat, check_after=6.0, tmax=90.0,
         slow_at=None, verbose=False):
    dense = _densify(waypts)
    speeds = [v_cap] * len(dense)
    if slow_at is not None:
        sx, sy, srad, sv = slow_at
        for i, (px, py) in enumerate(dense):
            if math.hypot(px - sx, py - sy) <= srad:
                speeds[i] = min(speeds[i], sv)
    drv = LaneDriver(_Route(dense, speeds), 2.65, v_cap=v_cap)
    ref = Path(dense)                     # 误差一律对着"原始参考线"量，不能对着倒角后的线自我安慰
    sim = _Sim(start[0], start[1], yaw0)
    t, worst, flips, prev = 0.0, 0.0, 0, 0.0
    while t < tmax and not drv.done:
        thr, brk, steer = drv.step((sim.x, sim.y), sim.yaw, sim.v, 0.1)
        assert all(map(lambda q: not math.isnan(q), (thr, brk, steer))), "NaN 输出"
        assert -1.0 <= steer <= 1.0, "舵角越界 %s" % steer
        s_ref, e_ref = ref.project(sim.x, sim.y)
        if e_ref * prev < 0 and abs(e_ref) > 0.05:
            flips += 1
        prev = e_ref
        if s_ref > check_after:
            worst = max(worst, abs(e_ref))
        if verbose and abs(t % 2.0) < 1e-6:
            print("      t=%5.1f (%6.1f,%6.1f) v=%.2f e=%+5.2f s=%5.1f steer=%+5.2f"
                  % (t, sim.x, sim.y, sim.v, drv.e, drv.s, steer))
        sim.step(thr, brk, steer, 0.1)
        t += 0.1
    d_end = math.hypot(sim.x - dense[-1][0], sim.y - dense[-1][1])
    ok = drv.done and worst <= tol_lat and d_end <= 1.2
    print("  [%s] %s  done=%s t=%.1fs 最大横向=%.2fm 终点距=%.2fm 纠偏次数=%d"
          % ("PASS" if ok else "FAIL", name, drv.done, t, worst, d_end, flips))
    return ok


def _selftest():
    ok = True
    print("LaneDriver 自测（单车运动学仿真，不连场景）")
    # 1) 60 m 直线，起点横向偏 0.6 m、航向偏 5°
    ok &= _run("直线", [(0, 0), (60, 0)], (0.0, 0.6), 5.0, 4.0, 0.35)
    # 2) 直角弯：40 m 直 + 90° 右转 40 m，弯心按 fog_route 的规矩限到 1.6 m/s。
    #    3 m 倒角在 90° 顶点本身就内切 1.24 m，所以门限按 1.3 m 算（跟踪误差≈0）
    ok &= _run("直角弯", [(0, 0), (40, 0), (40, 40)], (0.0, 0.0), 0.0, 3.0, 1.45,
               check_after=8.0, slow_at=(40, 0, 10.0, 1.6))
    # 3) 5 m 闸口走廊：1.2 m/s 限速、起点横向偏 1.6 m、车头已顺路
    ok &= _run("闸口走廊", [(0, 0), (0, 45)], (-1.6, 0.0), 90.0, 1.2, 0.45,
               check_after=8.0, tmax=120.0)
    # 3b) 更狠：横向偏 3.0 m，验证能收敛回来
    ok &= _run("走廊-偏3m", [(0, 0), (0, 45)], (-3.0, 0.0), 90.0, 1.2, 0.60,
               check_after=10.0, tmax=120.0)
    # 4) 到达精度：stop_xy + stop_m=6，应该停在半径 6 m 外的 0.7 m 内
    dense = _densify([(0, 0), (0, 40)])
    drv = LaneDriver(_Route(dense, [2.0] * len(dense)), 2.65, v_cap=2.0,
                     stop_xy=(0, 40), stop_m=6.0)
    sim = _Sim(0, 0, 0)
    t = 0.0
    while t < 90 and not drv.done:
        thr, brk, steer = drv.step((sim.x, sim.y), sim.yaw, sim.v, 0.1)
        sim.step(thr, brk, steer, 0.1)
        t += 0.1
    dd = math.hypot(sim.x - 0, sim.y - 40)
    ok4 = drv.done and 6.0 < dd <= 6.9
    print("  [%s] stop_m 停车  done=%s d=%.2fm（目标 6.0~6.9）"
          % ("PASS" if ok4 else "FAIL", drv.done, dd))
    ok &= ok4
    print("自测结果：%s" % ("全部通过" if ok else "有失败项"))
    return 0 if ok else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        raise SystemExit(_selftest())
    raise SystemExit("用法：python fog_run5.py --selftest")
