# -*- coding: utf-8 -*-
"""加速度判别探针：恒定油门、零刹车、零换挡、只转向（全程不改控制量）。

判什么（见 docs/accel-research.md §0.4）：

    P1「指令变化」模型 → 全程一个油门、刹车恒 0 ⇒ 应当**没有** −50；
    P2「实测加速度」模型 → 起步瞬态与巡航抖动仍在 ⇒ 仍会 −50。

顺带标定「恒定油门 → 稳态车速」（取最后 10 s 的中位数），这是「恒定油门巡检」
方案能不能把配速压下来的唯一依据。

跑法（脚本先起到 connect 阻塞，**由用户点「启动场景」**）：

    .\\.venv\\Scripts\\python.exe tools\\probes\\probe_accel_const.py --thr 0.030 --seconds 42

注意：本探针**不提交任何结果**，也不在收尾踩刹车（那会让「控制量恒定」这个前提失效）。
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics as st
import sys
import time
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[1]      # tools/（本文件在 tools/probes/ 下）
_REPO = _TOOLS.parent
sys.path.insert(0, str(_TOOLS))

from wp_follower import WpFollower  # noqa: E402

ENTRY = (34.60, -4.05)
CIRCLE_C = (0.0, 0.0)
CIRCLE_R = 10.0


def build_waypoints(laps: int = 2) -> list[tuple[float, float]]:
    """进场两段 + 绕中心 r=10 m 的圆 laps 圈（每圈 8 个点）。"""
    wps: list[tuple[float, float]] = [(26.0, -3.5), (16.0, -1.5)]
    ring = []
    for k in range(8):
        th = math.radians(45.0 * k)
        ring.append((CIRCLE_C[0] + CIRCLE_R * math.cos(th),
                     CIRCLE_C[1] + CIRCLE_R * math.sin(th)))
    # 从进场方向切进圆：先到 (10,0) 再沿逆时针走
    wps.append(ring[0])
    for _ in range(max(1, laps)):
        for p in ring[1:] + [ring[0]]:
            wps.append(p)
    return wps


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--thr", type=float, default=0.030, help="全程恒定油门（0~1）")
    ap.add_argument("--seconds", type=float, default=42.0)
    ap.add_argument("--lookahead", type=float, default=6.0)
    ap.add_argument("--accept", type=float, default=3.0)
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--self-check", action="store_true")
    ap.add_argument("--out", default=str(Path(
        r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\runs\accel-const-r1")))
    args = ap.parse_args()

    wps = build_waypoints(args.laps)
    total = sum(math.hypot(wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1])
                for i in range(len(wps) - 1))

    if args.self_check:
        print(f"航点={len(wps)} 总程≈{total:.1f} m 恒定油门={args.thr:.3f} "
              f"上限时≈{args.seconds:.0f} s 刹车=0（全程）")
        print("前 6 个航点:", [(round(a, 1), round(b, 1)) for a, b in wps[:6]])
        assert 0.0 < args.thr <= 1.0, "油门必须 (0,1]"
        assert len(wps) >= 10, "航点太少"
        print("self-check OK")
        return 0

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    from metacar import GearMode, SceneAPI, VehicleControl

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"场景: {[s.name for s in static.sub_scenes]}", flush=True)
    if static.vla_extension is not None:
        for reg in static.vla_extension.regions:
            print(f"  区域 id={reg.id} type={reg.type} pos=({reg.pos_x},{reg.pos_y}) "
                  f"{reg.length}x{reg.width}", flush=True)

    follower = WpFollower(wps, args.lookahead, args.accept)
    csv_path = out / "accel_const.csv"
    fh = open(csv_path, "w", newline="", encoding="utf-8")
    wr = csv.writer(fh)
    wr.writerow(["t", "sim_t", "x", "y", "yaw", "v", "thr", "brk", "steer", "acc"])

    t0 = time.time()
    prev_v = None
    prev_t = None
    events: list[tuple[float, float, float]] = []   # (t, a, v)
    vs: list[tuple[float, float]] = []              # (t, v)
    sent = 0
    last_log = -1e9

    for msg, _frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))
        sim_t = float(getattr(msg.scene_status, "used_time", 0.0) or 0.0)

        follower.update(x, y)
        steer = follower.steering(x, y, yaw)

        # ===== 全程唯一一处控制下发：油门恒定、刹车恒 0、挡位恒 DRIVE =====
        api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE,
                                               throttle=args.thr, brake=0.0,
                                               steering=steer))
        sent += 1

        acc = ((v - prev_v) / max(now - prev_t, 1e-3)) if prev_v is not None else 0.0
        wr.writerow([f"{now:.3f}", f"{sim_t:.3f}", f"{x:.2f}", f"{y:.2f}",
                     f"{yaw:.1f}", f"{v:.3f}", args.thr, 0.0, f"{steer:+.3f}",
                     f"{acc:+.2f}"])
        if prev_v is not None and abs(acc) > 4.5:
            events.append((now, acc, v))
            print(f"[{now:5.1f}s] |a|={abs(acc):5.1f} v={v:.2f} "
                  f"指令 thr={args.thr:.3f} brk=0.000 steer={steer:+.2f}", flush=True)
        prev_v, prev_t = v, now
        vs.append((now, v))

        if now - last_log >= 2.0:
            last_log = now
            d = math.hypot(x - CIRCLE_C[0], y - CIRCLE_C[1])
            print(f"[{now:5.1f}s] ({x:6.1f},{y:6.1f}) yaw={yaw:6.1f} v={v:5.2f} "
                  f"thr={args.thr:.3f} 距圆心={d:5.1f} 段={follower.seg}/{len(wps) - 1}",
                  flush=True)

        if now > args.seconds:
            print(f"[{now:5.1f}s] 到时间（未刹车、未停车），结束采集", flush=True)
            break

    fh.close()

    tail = [v for t, v in vs if t >= max(0.0, vs[-1][0] - 10.0)]
    steady = st.median(tail) if tail else 0.0
    print(f"\n===== 汇总 =====")
    print(f"下发帧数={sent} 时长={vs[-1][0]:.1f} s 恒定油门={args.thr:.3f}")
    print(f"末 10 s 车速中位={steady:.2f} m/s（{steady * 3.6:.1f} km/h）"
          f" 区间 {min(tail) if tail else 0:.2f}~{max(tail) if tail else 0:.2f}")
    print(f"|a|>4.5 事件数={len(events)}（其中起步前 3 s 内="
          f"{sum(1 for t, _, _ in events if t < 3.0)}）")
    print(f"CSV: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
