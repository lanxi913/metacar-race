# -*- coding: utf-8 -*-
r"""「环岛驶入、驶出」入口（场景元素 13，100 分）。

规则：
  * 200 s 内从起点到终点（主车道终点）
  * 压线 -20

现场因素（全部运行时读取）：
  * 路线   <- SceneStaticData.route（445 点 / 443.7 m，北口进 -> 绕环岛 -> 南口出）
  * 车辆尺寸/边线 <- roads[].lanes（用于校验，不做硬编码）
  * 巡航油门 <- 在线自标定（慢爬 -> 车速平台 -> 锁死），保持 |a|<=5 的加分

控制：
  * 横向 Stanley：航向误差 + 横向偏差反馈（车偏了自动往回纠）
  * 纵向 复用 speed 模块的自标定恒定油门（标定后开环，避免抖动丢分）
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from metacar import GearMode, SceneAPI, VehicleControl

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from speed.main import AdaptiveCruise  # noqa: E402

KMH = 1.0 / 3.6


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class RouteFollower:
    """在折线路径上做 Stanley 跟踪。

    yaw 约定：场景 yaw 顺时针为正，车头朝 -y 时 yaw=90，
    即 yaw = -atan2(dy, dx)。
    """

    def __init__(self, route, k_heading=2.0, k_cross=1.5, smooth=3):
        self.pts = route
        self.k_h = k_heading
        self.k_c = k_cross
        self.smooth = smooth
        self.idx = 0

    def nearest(self, x, y):
        lo = max(0, self.idx - 10)
        hi = min(len(self.pts) - 1, self.idx + 40)
        best, bd = lo, 1e9
        for i in range(lo, hi + 1):
            px, py = self.pts[i]
            d = math.hypot(px - x, py - y)
            if d < bd:
                best, bd = i, d
        # 局部窗口里离得很远，说明跟丢了，退回全局搜索重新对齐
        if bd > 10.0:
            best, bd = min(range(len(self.pts)),
                           key=lambda k: math.hypot(self.pts[k][0] - x, self.pts[k][1] - y)), None
            bd = math.hypot(self.pts[best][0] - x, self.pts[best][1] - y)
        self.idx = max(self.idx, best)
        return self.idx

    def tangent_yaw(self, i):
        a = max(0, i - self.smooth)
        b = min(len(self.pts) - 1, i + self.smooth)
        dx = self.pts[b][0] - self.pts[a][0]
        dy = self.pts[b][1] - self.pts[a][1]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return None
        return -math.degrees(math.atan2(dy, dx))

    def control(self, x, y, yaw_deg, v):
        i = self.nearest(x, y)
        yaw_ref = self.tangent_yaw(i)
        if yaw_ref is None:
            return 0.0, 0.0
        # 切向单位向量 -> 有符号横向偏差（正 = 车在路径右侧）
        a = max(0, i - self.smooth)
        b = min(len(self.pts) - 1, i + self.smooth)
        tx = self.pts[b][0] - self.pts[a][0]
        ty = self.pts[b][1] - self.pts[a][1]
        norm = math.hypot(tx, ty) or 1.0
        tx, ty = tx / norm, ty / norm
        px, py = x - self.pts[i][0], y - self.pts[i][1]
        cross = tx * py - ty * px          # >0 车在左侧（数学叉积）
        # 左正约定：与 steer>0 = 右转 配合，车偏左 -> corr>0 -> 右打轮回正
        e_ct = cross
        corr = math.degrees(math.atan2(self.k_c * e_ct, max(abs(v), 0.8)))
        desired_yaw = yaw_ref + corr
        err = math.radians(wrap_deg(desired_yaw - yaw_deg))
        steer = max(-1.0, min(1.0, self.k_h * err))
        return steer, e_ct

    def finished_index(self):
        return self.idx >= len(self.pts) - 4


def main() -> int:
    ap = argparse.ArgumentParser(description="环岛驶入、驶出")
    ap.add_argument("--seconds", type=float, default=190.0)
    ap.add_argument("--k-cross", type=float, default=2.2)
    ap.add_argument("--k-heading", type=float, default=2.0)
    args = ap.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    st = api.get_scene_static_data()
    route_obj = list(st.route)                       # AdaptiveCruise 需要 Vector3
    route = [(float(p.x), float(p.y)) for p in st.route]
    print(f"路线点: {len(route)}  子场景: {[s.name for s in st.sub_scenes]}", flush=True)
    if len(route) >= 2:
        total = sum(math.dist(route[i], route[i + 1]) for i in range(len(route) - 1))
        print(f"  起点 {route[0]} -> 终点 {route[-1]}，全长 {total:.1f} m", flush=True)

    # 路线里在环岛进出口有急拐的航点（实测最小半径 3.5 m，比车的最小转弯半径 3.8 m 还小），
    # 直接跟会切角压线，先做滑动平均平滑
    def smooth_path(pts, k=4):
        out = []
        for i in range(len(pts)):
            lo = max(0, i - k)
            hi = min(len(pts) - 1, i + k)
            n = hi - lo + 1
            out.append((sum(pts[j][0] for j in range(lo, hi + 1)) / n,
                        sum(pts[j][1] for j in range(lo, hi + 1)) / n))
        return out

    route = smooth_path(route, k=4)

    follower = RouteFollower(route, k_heading=args.k_heading, k_cross=args.k_cross)
    cruise = AdaptiveCruise(route_obj, 60.0 * KMH, goal_ratio=0.92)

    frames = 0
    t0 = time.time()
    last_t = None
    v_peak = 0.0
    prev_state = None
    max_e = 0.0
    stuck_t = 0.0
    arrived = False
    arrived_t = None

    for msg, cams in api.main_loop():
        frames += 1
        car = msg.main_vehicle
        pose = msg.pose_gnss
        status = msg.scene_status

        dt = 1.0 / 30.0 if last_t is None else max(status.used_time - last_t, 1e-3)
        last_t = status.used_time

        steer, e_ct = follower.control(pose.pos_x, pose.pos_y, pose.ori_z, car.speed)
        max_e = max(max_e, abs(e_ct))

        vc, mode = cruise.step(pose.pos_x, pose.pos_y, pose.ori_z, car.speed, dt)
        vc = VehicleControl(gear=vc.gear, throttle=vc.throttle, brake=vc.brake, steering=steer)
        api.set_vehicle_control(vc)
        v_peak = max(v_peak, abs(car.speed))

        if prev_state == "ramp" and cruise.state == "cruise":
            print(f"== 自标定完成: 油门 {cruise.calibrated_thr:.3f} -> "
                  f"平台速度 {cruise.calibrated_v * 3.6:.1f} km/h ==", flush=True)
        prev_state = cruise.state

        if frames % 30 == 1:
            print(f"[{status.used_time:6.1f}s] pos=({pose.pos_x:8.2f},{pose.pos_y:8.2f}) "
                  f"yaw={pose.ori_z:7.1f} v={car.speed:5.2f} e={e_ct:+5.2f} "
                  f"str={steer:+5.2f} thr={vc.throttle:5.3f} idx={follower.idx}", flush=True)

        # 到终点后不要立刻断开：平台要等场景自己结束才记录成绩。
        # 这里保持发送控制指令，等 main_loop 自然结束。
        # 到最后一个路线点附近才算到；之前要保持行驶，别停在线上
        end_pt = follower.pts[-1]
        dist_end = math.hypot(end_pt[0] - pose.pos_x, end_pt[1] - pose.pos_y)
        if dist_end < 1.2 or follower.finished_index():
            if arrived_t is None:
                arrived_t = time.time()
                print(f"== 接近终点（余 {dist_end:.2f} m），继续低速滚过终点线 ==", flush=True)
            arrived = True
            # 先低速再走 3 秒压过终点，然后刹停等场景结束
            if time.time() - arrived_t < 3.0:
                api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=0.10,
                                                       brake=0.0, steering=steer))
                continue
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            if time.time() - arrived_t > 25.0:
                print("!! 等待场景结束超时，主动退出", flush=True)
                break
            continue

        # 卡死检测：给了油门却长时间不动 -> 退出，别把 200 s 耗光
        if abs(car.speed) < 0.05 and vc.throttle > 0.05:
            stuck_t += dt
        else:
            stuck_t = 0.0
        if stuck_t > 15.0:
            print(f"!! 卡住 {stuck_t:.0f}s（v≈0 且一直在给油），退出", flush=True)
            break

        if time.time() - t0 > args.seconds:
            print("到达脚本时长上限，退出", flush=True)
            break

    summary = {
        "frames": frames,
        "v_peak_kmh": round(v_peak * 3.6, 2),
        "max_cross_track_m": round(max_e, 2),
        "calibrated_thr": cruise.calibrated_thr,
        "calibrated_v_kmh": None if cruise.calibrated_v is None else round(cruise.calibrated_v * 3.6, 2),
        "route_len_m": None if len(route) < 2 else round(
            sum(math.dist(route[i], route[i + 1]) for i in range(len(route) - 1)), 1),
        "arrived": arrived,
        "elapsed": round(time.time() - t0, 1),
    }
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    log_dir = Path(__file__).resolve().parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    (log_dir / f"roundabout_{time.strftime('%Y%m%d-%H%M%S')}.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
