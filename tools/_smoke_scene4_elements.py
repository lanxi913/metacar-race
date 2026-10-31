# -*- coding: utf-8 -*-
"""离线冒烟：不连仿真，用假 static 把 ElementRouter 的 12 条分支都走一遍。

目的：把「构造失败 / 参数名不对 / 返回形态不对」这类错误在开引擎之前挡掉。
"""
import math
import sys
import types
from pathlib import Path

REPO = Path(r"D:\download3\算法邀請賽\metacar-race")
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

from scene4_elements import ElementContext, ElementRouter, handler_kind  # noqa: E402


class P:
    __slots__ = ("x", "y", "z")

    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class Static:
    def __init__(self, route, spans_pts, names):
        self.route = route
        self.roads = []
        self.sub_scenes = [types.SimpleNamespace(
            name=n, start_point=P(a[0], a[1]), end_point=P(b[0], b[1]))
            for n, a, b in zip(names, spans_pts, spans_pts[1:])]


NAMES = [
    "车辆直行", "连续转弯", "环岛驶入", "车辆跟随", "十字路口通行", "复杂路况超车",
    "匝道行驶", "车辆紧急停止", "辅路驶入驶出", "车辆变道", "车辆切出", "平行车道泊车",
]


def build_world():
    # 一条 1200 m 的直线，12 个元素各 100 m
    route = [P(i * 1.0, 0.0) for i in range(1201)]
    pts = [(i * 100.0, 0.0) for i in range(13)]
    static = Static(route, pts, NAMES)
    spans = [{"index": k + 1, "name": NAMES[k], "s_start": float(k * 100),
              "s_end": float((k + 1) * 100)} for k in range(12)]
    return static, spans  # noqa: F821


class Pose:
    def __init__(self, x, y, yaw=0.0):
        self.pos_x, self.pos_y, self.ori_z = float(x), float(y), float(yaw)


def main() -> int:
    static, spans = build_world()
    logs = []
    router = ElementRouter(static, static.route, spans, vmax=8.0,
                           ego_length=4.70, ego_width=1.90, lane_width=3.50,
                           log=lambda m: logs.append(m))
    print("路由表：")
    for name in NAMES:
        print(f"  {name:12s} -> {handler_kind(name)}")

    bad = 0
    for k, name in enumerate(NAMES):
        s_now = k * 100.0 + 50.0
        x = s_now
        ctx = ElementContext(
            pose=Pose(x, 0.0), speed=3.0, dt=1 / 30.0, obstacles=[], static=static,
            sim_t=10.0 + k, s_now=s_now, x=x, y=0.0, yaw=0.0, lane_width=3.50,
            ego_length=4.70, ego_width=1.90, vmax=8.0, time_limit=100.0,
            end_point=P((k + 1) * 100.0, 0.0), trajectory=None, a_now=0.0,
            light=None, stop_gap=None,
        )
        try:
            ctl = router.step(ctx)
            # 同一元素连打 5 帧，模拟真实调用节奏
            for _ in range(5):
                ctl = router.step(ctx)
            if ctl is None:
                print(f"  [{name}] 未接管（走通用路径）")
            else:
                # None = 该通道沿用通用栈（环岛只接管方向盘），不当成越界
                ok = all(v is None or lo <= v <= hi for v, lo, hi in (
                    (ctl.throttle, 0.0, 1.0), (ctl.brake, 0.0, 1.0),
                    (ctl.steering, -1.0, 1.0)))
                fmt = lambda v: "  --" if v is None else f"{v:+.2f}"
                print(f"  [{name}] {ctl.note:16s} thr={fmt(ctl.throttle)} "
                      f"brk={fmt(ctl.brake)} steer={fmt(ctl.steering)} "
                      f"gear={getattr(ctl.gear, 'name', None)} {'OK' if ok else 'RANGE-BAD'}")
                if not ok:
                    bad += 1
        except Exception as exc:  # noqa: BLE001
            bad += 1
            print(f"  [{name}] 异常: {exc!r}")

    print("\nrouter 日志：")
    for m in logs:
        print("  " + m)
    print(f"\n统计: {router.stats}  降级: {sorted(router.disabled)}  失败分支: {bad}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
