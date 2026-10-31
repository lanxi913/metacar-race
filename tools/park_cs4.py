"""连续场景 4 驱动的 CLI 入口（薄壳）。

真正的逻辑全在 `metacar_race/scene4/` 包里；这个文件只做三件事：解析命令行 →
组装 `DriverConfig` → `Driver.run()`。保留原来那套参数名，历史命令还能直接跑。

    .venv\\Scripts\\python.exe tools\\park_cs4.py --skip-to-index 0 \\
        --out D:\\...\\runs\\full-r5

包结构与分层理由见 `metacar_race/scene4/__init__.py`；旧的整体式实现（1519 行）
留在 git 历史 `9bdb2e8:tools/park_cs4.py`，需要对照行为时 `git show` 它。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from metacar_race.scene4 import Driver, DriverConfig  # noqa: E402
from metacar_race.scene4.config import HOLD_END_S      # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="连续场景 4 驱动（元素级策略路由，框架版）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--skip-to-index", type=int, default=0,
                   help="已停用（本地 AGENTS.md 铁律 9）：只接受 0，非 0 直接报错退出")
    p.add_argument("--vmax", type=float, default=8.0, help="速度上限 m/s")
    p.add_argument("--max-seconds", type=float, default=700.0, help="墙钟超时秒数")
    p.add_argument("--end-hold", type=float, default=HOLD_END_S, help="终点驻车保持秒数")
    p.add_argument("--shots", type=float, default=0.0, help="每隔多少秒存一张前视图（0=不存）")
    p.add_argument("--out", default=None, help="产物目录（默认 <repo>/logs/scene4/<时间戳>）")

    p.add_argument("--no-overtake", action="store_true", help="不用并线状态机")
    p.add_argument("--no-stanley", action="store_true", help="环岛段也不用 Stanley")
    p.add_argument("--no-light", action="store_true", help="不处理信号灯与停止线")
    p.add_argument("--soft-start", action="store_true", help="油门上升率限幅（默认关）")
    p.add_argument("--no-accel-gov", action="store_true", help="关掉收油治理（对照用）")

    p.add_argument("--bonus", action="store_true",
                   help="+10 模式：油门 ≤0.62、刹车恒 0、|a_cmd| ≤4.5")
    p.add_argument("--thr-cap", type=float, default=None, help="油门上限（默认按 --bonus 推导）")
    p.add_argument("--brk-cap", type=float, default=None, help="刹车上限（默认按 --bonus 推导）")
    p.add_argument("--a-cmd-limit", type=float, default=None, help="指令加速度预算")
    p.add_argument("--fixed-throttle", type=float, default=0.0,
                   help=">0 时绕过速度环，整段直发这个油门（直行/恒速元素拿 +10 用）")

    p.add_argument("--no-elements", action="store_true", help="关掉元素级策略，全走基线")
    p.add_argument("--elements-only", default="起步直行,车辆直行,连续转弯,环岛",
                   help="只让这些关键字对应的元素接管控制量（逗号分隔，空=全部）")
    p.add_argument("--lane-element", default="",
                   help="名字含该字样的元素改走「真变道」（跨虚线整条平移），例如 车辆直行")
    return p


def make_config(args: argparse.Namespace) -> DriverConfig:
    only = tuple(s.strip() for s in str(args.elements_only or "").split(",") if s.strip())
    return DriverConfig(
        skip_to_index=int(args.skip_to_index),
        vmax=float(args.vmax),
        max_seconds=float(args.max_seconds),
        end_hold=float(args.end_hold),
        shots=float(args.shots),
        out=Path(args.out) if args.out else None,
        overtake=not args.no_overtake,
        stanley=not args.no_stanley,
        light=not args.no_light,
        soft_start=bool(args.soft_start),
        accel_gov=not args.no_accel_gov,
        bonus=bool(args.bonus),
        thr_cap=args.thr_cap,
        brk_cap=args.brk_cap,
        a_cmd_limit=args.a_cmd_limit,
        fixed_throttle=float(args.fixed_throttle),
        elements=not args.no_elements,
        elements_only=only,
        lane_element=str(args.lane_element or "").strip(),
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if int(args.skip_to_index) > 0:
        # 铁律 9（本地 AGENTS.md，2026-10-25）：不再跳关，验证一律从起点真实驾驶。
        parser.error("跳关已停用（本地 AGENTS.md 铁律 9）：请去掉 --skip-to-index，"
                     "用 --elements-only 限定要接管的元素")
    cfg = make_config(args)
    return Driver(cfg).run()


if __name__ == "__main__":
    raise SystemExit(main())
