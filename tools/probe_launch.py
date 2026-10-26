# -*- coding: utf-8 -*-
"""起步响应探针（只诊断，不参与跑分）。

目的：连续场景从静止起步时，速度在 0.15 s 内从 0 跳到 2.4 m/s（约 12~20 m/s²），
拿不到 `|a| <= 5` 的 +10。已知油门斜率限幅只能把"开始动"的油门门槛从 0.120
压到 0.033、峰值 20.45 压到 11.65，压不到 5 以下。剩下没量过的自由度只有一个：
**刹车**（实测 0.35 约 -5 m/s²、1.0 约 -18 m/s²，是连续可调的）。

于是本探针把 (油门, 刹车) 当作二维输入，逐个"从静止起步"试一遍，量出：
  - 每个组合下真正的最大 |dv/dt|（30 Hz 逐帧差分，和平台判定同口径）
  - 车是"动不了"还是"一动就冲"

脚本只发 throttle/brake，steering 恒 0，gear 恒 DRIVE；车速超过 2.6 m/s 立刻
用刹车兜住，所以行驶距离很短，不会开出赛道。

用法（先跑脚本，再在客户端点"启动场景"，选连续场景1）：
    .venv\\Scripts\\python.exe -u tools\\probe_launch.py

产物：logs/probe_launch/<时间戳>/trace.csv + summary.txt
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, VehicleControl, SceneAPI  # noqa: E402

# (label, throttle, brake, seconds)
PLAN = [
    ("relax",              0.000, 0.00, 0.6),
    # 基线：已知 0.05 会以约 12 m/s² 冲出去
    ("A_thr0.050",         0.050, 0.00, 2.5),
    ("settle",             0.000, 0.50, 2.0),
    # 用刹车把净加速度往下压（假设油门/刹车加速度可加）
    ("B_thr0.050_brk0.10", 0.050, 0.10, 2.5),
    ("settle",             0.000, 0.50, 2.0),
    ("C_thr0.050_brk0.20", 0.050, 0.20, 2.5),
    ("settle",             0.000, 0.50, 2.0),
    ("D_thr0.050_brk0.30", 0.050, 0.30, 2.5),
    ("settle",             0.000, 0.50, 2.0),
    ("E_thr0.120_brk0.60", 0.120, 0.60, 2.5),
    ("settle",             0.000, 0.50, 2.0),
    # 只比门槛高一点点的油门（看能不能"蠕动"）
    ("F_thr0.035",         0.035, 0.00, 2.5),
    ("settle",             0.000, 0.50, 2.0),
    # 参考：更大的油门，看加速度是否与油门无关
    ("G_thr0.150",         0.150, 0.00, 2.0),
    ("settle",             0.000, 0.50, 2.0),
]

V_SAFE = 2.60      # 超过就用刹车兜住
V_RESET = 0.15     # 兜到这么慢才放行


def main() -> int:
    parser = argparse.ArgumentParser(description="launch response probe")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--v-safe", type=float, default=V_SAFE)
    args = parser.parse_args()

    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_dir = args.out or (REPO / "logs" / "probe_launch" / stamp)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_f = (out_dir / "trace.csv").open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_f)
    writer.writerow(["frame", "sim_t", "seg", "idx", "thr_cmd", "brk_cmd",
                     "v", "x", "y", "clamped"])

    print("waiting for scene ... start the scene in the client now", flush=True)
    api = SceneAPI()
    api.connect()
    _static = api.get_scene_static_data()

    plan_i = 0
    seg_t = 0.0
    sim_t = 0.0
    prev_t = None
    prev_v = None
    clamped = False

    # 每个 plan 段的统计
    stats: dict[int, dict] = {}

    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            now = float(msg.scene_status.used_time)
            dt = 0.0 if prev_t is None else max(0.0, now - prev_t)
            prev_t = now
            sim_t = now

            v = float(msg.main_vehicle.speed)
            x = float(msg.pose_gnss.pos_x)
            y = float(msg.pose_gnss.pos_y)

            if plan_i < len(PLAN):
                label, thr, brk, secs = PLAN[plan_i]
            else:
                label, thr, brk, secs = "done", 0.0, 1.0, 0.0

            # 安全兜底
            if abs(v) > args.v_safe:
                clamped = True
                thr, brk = 0.0, 0.50
            elif clamped and abs(v) < V_RESET:
                clamped = False

            api.set_vehicle_control(VehicleControl(
                gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=0.0))

            writer.writerow([frame, round(now, 4), label, plan_i,
                             round(thr, 4), round(brk, 4), round(v, 4),
                             round(x, 2), round(y, 2), int(clamped)])

            st = stats.setdefault(plan_i, {
                "label": label, "max_a": 0.0, "max_a_t": 0.0, "max_v": 0.0,
                "first_move_t": None, "first_move_thr": None, "n": 0})
            st["n"] += 1
            st["max_v"] = max(st["max_v"], abs(v))
            if prev_v is not None and dt > 1e-6:
                a = (v - prev_v) / dt
                if abs(a) > abs(st["max_a"]):
                    st["max_a"] = a
                    st["max_a_t"] = now
            if st["first_move_t"] is None and abs(v) > 0.05:
                st["first_move_t"] = now
                st["first_move_thr"] = thr
            prev_v = v

            seg_t += dt
            if seg_t >= secs:
                seg_t = 0.0
                plan_i += 1
                clamped = False
                if plan_i >= len(PLAN):
                    break
    finally:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        csv_f.close()

    lines = ["seg,label,frames,max_v,max_a,max_a_t,first_move_t,first_move_thr"]
    for i in sorted(stats):
        st = stats[i]
        lines.append("%d,%s,%d,%.3f,%+.2f,%.3f,%s,%s" % (
            i, st["label"], st["n"], st["max_v"], st["max_a"], st["max_a_t"],
            "None" if st["first_move_t"] is None else "%.3f" % st["first_move_t"],
            "None" if st["first_move_thr"] is None else "%.3f" % st["first_move_thr"]))
    (out_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines), flush=True)
    print("out: %s" % out_dir, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
