# -*- coding: utf-8 -*-
"""离线决策回放：只吃"已有的观测数据"，不跑仿真。

四道"不是记忆"的检验：
  A 改时限：扫一条响应曲线（预算越小，计划越短，且永不吃穿预算）
  B 挪车：把禁停区里的一辆车整体挪 3 m，规划必须跟着变
  C 换起点：从不同位姿起跑，第一步动作必须自己变
  D 反事实：车在合法车位 vs 车在禁停区，规划必须从"几乎不读"变成"优先去读"

用法：
  python tools/vla_plan_probe.py --inventory inv.json [--path-csv logs/.../run.csv]
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vla2.planner import CarObs, Estimators, Orienteer  # noqa: E402

# 相机水平视场：640 px 宽、焦距 554 px ⇒ 半角 atan(320/554)
HFOV_HALF = math.atan2(320.0, 554.0)


def load_inventory(p: Path):
    data = json.loads(p.read_text(encoding="utf-8"))
    region = tuple(data["region"])
    cars = [CarObs(c["x"], c["y"], c.get("ori", 90.0), c.get("p_violation", 0.5),
                   c.get("p_label", 0.0), c.get("label", ""))
            for c in data["cars"]]
    return region, cars


def path_length(csv_path: Path) -> float:
    rows = list(csv.DictReader(io.open(csv_path, encoding="utf-8")))
    tot = 0.0
    for a, b in zip(rows, rows[1:]):
        tot += math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
    return tot


def make_est(v_now: float, samples, submit: float) -> Estimators:
    est = Estimators(log=lambda *_: None)
    est.v_now = v_now
    for d, ok in samples:
        est._samples.append((d, ok))
    est._fit_read()
    est.set_submit_cost(submit)
    return est


def run(tag: str, region, cars, pose, t_left: float, v_now=2.0,
        samples=((4.0, True), (4.5, True), (11.0, False), (14.0, False)),
        submit=0.4, swept=None, quiet=False) -> dict:
    est = make_est(v_now, samples, submit)
    ori = Orienteer(est, region, observed_area=swept, log=lambda *_: None)
    d = ori.decide(cars, pose, t_left)
    if not quiet:
        print(f"\n=== {tag} ===")
        print(f"  {d['estimates']}")
        print(f"  视野节点 {d['n_view']} / 覆盖节点 {d['n_cov']} / 剩余 {t_left:.0f}s")
        print(f"  规划 {len(d['plan'])} 步, 预计 {d['cost']:.1f}s, 收益 {d['prize']:.2f}")
        if d["next"] is None:
            print("  决策：**直接交卷**（没有正收益动作）")
        else:
            n = d["next"]
            extra = "" if n.car < 0 else f" 读车#{n.car}"
            print(f"  决策：去 ({n.x:.1f},{n.y:.1f}) {n.kind}{extra} 收益={n.prize:.2f}")
    return d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--inventory", required=True)
    ap.add_argument("--path-csv", help="用真实跑次的位姿算已观测面积")
    ap.add_argument("--pose", type=float, nargs=2, default=[0.0, 0.0])
    ap.add_argument("--limit", type=float, default=100.0)
    args = ap.parse_args()

    region, cars = load_inventory(Path(args.inventory))
    swept = None
    if args.path_csv:
        L = path_length(Path(args.path_csv))
        swept = L * 2 * 14.0 * math.tan(HFOV_HALF)   # 里程 × 14 m 处的视场宽
        print(f"已观测面积：里程 {L:.1f} m × 视场 {2 * 14.0 * math.tan(HFOV_HALF):.1f} m "
              f"= {swept:.0f} ㎡")
    print(f"盘点：{len(cars)} 辆车，禁停区 {region}")
    pose = tuple(args.pose)

    # D 先算：把车挪进禁停区（反事实），后面的 B/C 用它才有意义
    cf = [CarObs(c.x, 18.0 + 2.0 * i, c.ori_deg, 0.95, c.p_label, c.label)
          for i, c in enumerate(cars)]
    base = run("基准（车在合法车位）", region, cars, pose, args.limit, swept=swept)
    d = run("D 反事实：车挪进禁停区", region, cf, pose, args.limit, swept=swept)

    # A 时限响应曲线
    sweep = []
    for T in (120.0, 100.0, 60.0, 30.0, 15.0):
        r = run("", region, cf, pose, T, swept=swept, quiet=True)
        sweep.append((T, len(r["plan"]), r["cost"], r["prize"],
                      r["next"].kind if r["next"] else "submit"))
    print("\n=== A 时限响应曲线（车在禁停区）===")
    print("   时限   步数   预计耗时   收益   首步")
    for T, n, c, p, k in sweep:
        flag = "" if c <= T - 0.4 + 1e-6 else "  <-- 吃穿预算！"
        print(f"  {T:5.0f}s {n:5d}  {c:8.1f}s {p:7.2f}  {k}{flag}")
    ok_a = all(c <= T - 0.4 + 1e-6 for T, _, c, _, _ in sweep) and \
        all(sweep[i][1] >= sweep[i + 1][1] for i in range(len(sweep) - 1)) and \
        sweep[0][1] > sweep[-1][1]

    # B 挪车（禁停区里离起点最近的那辆 +3 m）
    idx = min(range(len(cf)), key=lambda i: math.hypot(cf[i].x - pose[0], cf[i].y - pose[1]))
    moved = [CarObs(c.x, c.y, c.ori_deg, c.p_violation, c.p_label, c.label) for c in cf]
    moved[idx] = CarObs(cf[idx].x, cf[idx].y + 3.0, cf[idx].ori_deg, cf[idx].p_violation,
                        cf[idx].p_label, cf[idx].label)
    b = run(f"B 挪车：禁停区车#{idx} y+3m", region, moved, pose, args.limit, swept=swept)

    # C 换起点
    c = run("C 换起点 (30,50)", region, cf, (30.0, 50.0), args.limit, swept=swept)

    print("\n=== 判定 ===")
    first = lambda r: (r["next"] and (round(r["next"].x, 1), round(r["next"].y, 1))) or None
    ok_b = first(b) != first(d) or len(b["plan"]) != len(d["plan"])
    ok_c = first(c) != first(d)
    ok_d = (first(d) != first(base)) or (d["prize"] > base["prize"] + 0.5) or \
           ((base["next"] is None) != (d["next"] is None))
    print(f"  A 时限响应（单调变短且不吃穿预算）：{'通过' if ok_a else '未通过'}")
    print(f"  B 挪车后规划变化：{'通过' if ok_b else '未变化 —— 可疑'}")
    print(f"  C 换起点后首步变化：{'通过' if ok_c else '未变化 —— 可疑'}")
    print(f"  D 合法↔禁停区 规划变化：{'通过' if ok_d else '未变化 —— 可疑'}")
    return 0 if (ok_a and ok_b and ok_c and ok_d) else 1


if __name__ == "__main__":
    raise SystemExit(main())
