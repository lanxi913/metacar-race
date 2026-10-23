# -*- coding: utf-8 -*-
"""泊车路径离线规划：两段圆弧倒车（S 形），先在纸上验证可行性。

已知（实测）：
    车位中心 S=(-7.80, -0.12)，轴线 +x，尺寸 5.50(深) × 2.50(宽)
    两侧停车：(-8.37, +3.03) 和 (-8.37, -3.05)，各 4.00 × 1.80
    车道：x ≈ -2.16
    曲率：前进 k≈0.18/单位转角，倒车 k≈0.16/单位转角 → 满舵 R≈5.5~6.3 m

模型：车辆 4.00 × 1.69，后轴参考点近似车心（先按车心算，留 0.3 m 余量）。

用法：
    .venv\\Scripts\\python.exe -u tools/plan_parking.py
"""
import itertools
import math

# ---- 场景常量 ----
SLOT_C = (-7.80, -0.12)
SLOT_DEPTH = 5.50      # 沿 x
SLOT_WIDTH = 2.50      # 沿 y
CAR_L, CAR_W = 4.00, 1.69
PARKED = [
    (-8.37, 3.03, 4.00, 1.80),
    (-8.37, -3.05, 4.00, 1.80),
]
LANE_X = -2.16
CLEAR_MARGIN = 0.25


def step(pose, k, ds):
    """圆弧积分：pose=(x,y,theta)，k=曲率(1/m)，ds=弧长(可负)。"""
    x, y, th = pose
    th2 = th + k * ds
    # 中点法，减小离散误差
    thm = th + 0.5 * k * ds
    x2 = x + math.cos(thm) * ds
    y2 = y + math.sin(thm) * ds
    return (x2, y2, th2)


def sweep(start, k, ds, n=40):
    """把一段圆弧采样成位姿列 [(x,y,th), ...]。"""
    poses = [start]
    pose = start
    for _ in range(n):
        pose = step(pose, k, ds / n)
        poses.append(pose)
    return pose, poses


def car_corners(x, y, th_deg):
    """车体四角（场景 yaw 顺时针，转成数学角后旋转）。"""
    th = math.radians(-th_deg)
    c, s = math.cos(th), math.sin(th)
    out = []
    for dx, dy in ((CAR_L / 2, CAR_W / 2), (CAR_L / 2, -CAR_W / 2),
                   (-CAR_L / 2, -CAR_W / 2), (-CAR_L / 2, CAR_W / 2)):
        out.append((x + dx * c - dy * s, y + dx * s + dy * c))
    return out


def rect_of(x, y, w, h):
    return [(x - w / 2, y - h / 2), (x + w / 2, y - h / 2),
            (x + w / 2, y + h / 2), (x - w / 2, y + h / 2)]


def sat_overlap(poly1, poly2):
    """分离轴定理，判断两个凸多边形是否相交。"""
    for poly in (poly1, poly2):
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            nx, ny = y2 - y1, -(x2 - x1)
            ln = math.hypot(nx, ny)
            if ln < 1e-9:
                continue
            nx, ny = nx / ln, ny / ln
            p1 = [nx * px + ny * py for px, py in poly1]
            p2 = [nx * px + ny * py for px, py in poly2]
            if max(p1) < min(p2) or max(p2) < min(p1):
                return False
    return True


def path_clear(poses):
    """路径上每个采样点检查是否撞到两辆已停车。"""
    for (x, y, th) in poses:
        corners = car_corners(x, y, math.degrees(th))
        for px, py, pw, ph in PARKED:
            if sat_overlap(corners, rect_of(px, py, pw + 2 * CLEAR_MARGIN, ph + 2 * CLEAR_MARGIN)):
                return False
    return True


def simulate(R, theta1_deg, theta2_deg, d0):
    """从车道 (LANE_X, sy+d0) 朝 -y 开始倒车，两段圆弧。

    倒车时曲率符号与转角相反：k = -1/R * sign。
    第一段把车头从 -90° 转到 -90+θ1，第二段反向转回目标 0°。
    """
    sy = SLOT_C[1]
    start = (LANE_X, sy + d0, math.radians(-90.0))

    k1 = +1.0 / R
    a1 = math.radians(theta1_deg)
    pose1, pts1 = sweep(start, k1, -R * a1)

    k2 = -1.0 / R
    a2 = math.radians(theta2_deg)
    pose2, pts2 = sweep(pose1, k2, -R * a2)

    end = pose2
    total_turn = theta1_deg - theta2_deg
    return end, pts1 + pts2, total_turn


def main() -> int:
    sy = SLOT_C[1]
    best = None

    for R, t1, t2, d0 in itertools.product(
            [5.6, 6.0, 6.5, 7.0, 7.5, 8.0],
            range(40, 95, 5),
            range(0, 55, 5),
            [x * 0.5 for x in range(6, 40)]):
        if abs((t1 - t2) - 90) > 6:      # 总转角要接近 90°
            continue
        end, pts, _ = simulate(R, t1, t2, d0)
        ex, ey, eth = end
        # 终点要求：贴到车位中心，车头朝 0°
        err = math.hypot(ex - SLOT_C[0], ey - SLOT_C[1])
        dth = abs(math.degrees(math.atan2(math.sin(eth), math.cos(eth))))
        if err > 0.8 or dth > 10:
            continue

        if not path_clear(pts):
            continue

        score = err + abs(dth) * 0.02
        if best is None or score < best[0]:
            best = (score, R, t1, t2, d0, end)

    if not best:
        print("没找到可行解：两段等半径圆弧在 5.64 m 横向距离内进不去")
        print("→ 需要换策略：斜向切入 / 三段式（前进-倒车-前进）")
        return 1

    score, R, t1, t2, d0, end = best
    print("找到可行路径：")
    print(f"  半径 R={R:.1f} m（曲率 {1/R:.3f}）")
    print(f"  第一段倒车转 {t1}°，第二段回 {t2}°")
    print(f"  起点：车道 x={LANE_X}，y={sy + d0:.2f}（车位中心 +{d0:.2f} m）")
    print(f"  终点：({end[0]:.2f}, {end[1]:.2f})  航向 {math.degrees(end[2]):.1f}°")
    print(f"  误差 {score:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
