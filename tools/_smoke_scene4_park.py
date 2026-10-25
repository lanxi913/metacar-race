"""离线冒烟：元素 12 平行车道泊车（`scene4/park.py`）。

分两段验证：
    1) 接线：`kind_of("平行车道泊车")=="park"`、策略挂着、无车位时退回基线、车位/远端车解析；
    2) 几何：用运动学模型把车从元素 12 起点开一遍（接近段 → staging → 两段圆弧 → FIT），
       检查两段圆弧是否把车送到车位中心（主控制器已由 main 的 5 次 210 分验证，
       这里查的是**旋转坐标系**有没有把几何映错）。

运动学：`x += v·dt·cos(−yaw)`、`y += v·dt·sin(−yaw)`、`yaw += v·dt·tan(0.6·steer)/2.6`（度），
`yaw = −atan2(dy, dx)`、`steer > 0 = 右转`——这套约定是 run.csv 里实测出来的
（基线右转 steer=+1.000 时 yaw 270→311）。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from metacar import GearMode  # noqa: E402

from metacar_race.scene4 import config as C  # noqa: E402
from metacar_race.scene4.park import ParkStrategy  # noqa: E402
from metacar_race.scene4.strategies import (  # noqa: E402
    Command, ElementContext, STRATEGY_BY_KIND, kind_of)

FAILED: list[str] = []
COUNT = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global COUNT
    COUNT += 1
    print(f"  {'OK  ' if ok else 'FAIL'} {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        FAILED.append(name)


# ---------------------------------------------------------------- 场景数据
SLOT = {"id": -700001, "type": 30, "x": -2154.00, "y": -1380.69, "v": 0.0,
        "len": 5.5, "wid": 2.5, "ori": 270.0, "dist": 0.0, "lat": 0.0}
CAR_FRONT = {"id": -712754, "type": 6, "x": -2154.00, "y": -1374.60, "v": 0.0,
             "len": 4.0, "wid": 1.8, "ori": 270.0, "dist": 0.0, "lat": 0.0}
CAR_BACK = {"id": -711744, "type": 6, "x": -2154.00, "y": -1386.97, "v": 0.0,
            "len": 4.0, "wid": 1.8, "ori": 270.0, "dist": 0.0, "lat": 0.0}
OBSTACLES = [CAR_FRONT, CAR_BACK, SLOT]

START = (-2154.92, -1453.06, 270.0)          # 元素 12 起点（elem89-r1 实测）
WHEELBASE = 2.60
MAX_STEER_RAD = 0.60


def ctx_at(x, y, yaw, speed, obstacles=OBSTACLES, dt=1.0 / 30.0):
    return ElementContext(x=x, y=y, yaw=yaw, speed=speed, dt=dt, sim_t=0.0, s_now=4700.0,
                          pose=SimpleNamespace(pos_x=x, pos_y=y, ori_z=yaw),
                          obstacles=list(obstacles), static=None, vmax=9.0,
                          s_start=4645.4, s_end=4718.5, platform_element="平行车道泊车")


def integrate(x, y, yaw, speed, cmd, dt):
    """一帧运动学 + 简化纵向执行器（油门→车速目标，刹车→减速）。"""
    gear = cmd.gear if cmd.gear is not None else GearMode.DRIVE
    thr = 0.0 if gear == GearMode.PARKING else float(cmd.throttle or 0.0)
    brk = 0.0 if gear == GearMode.PARKING else float(cmd.brake or 0.0)
    steer = float(cmd.steer or 0.0)

    # 纵向：目标速度按挡位定符号；油门 0.01 起步能蠕行（真车就是这样入库的）
    if gear == GearMode.REVERSE:
        target = -max(0.55, 8.0 * thr) if thr > 0.0 else 0.0
    else:
        target = max(0.0, 9.0 * thr)
    if brk > 0.0:
        target = 0.0
    rate = 6.0 * brk + 0.6 if brk > 0.0 else 1.6
    step = rate * dt
    speed = min(target, speed + step) if speed < target else max(target, speed - step)
    if abs(speed) < 1e-3:
        speed = 0.0

    x += speed * dt * math.cos(math.radians(-yaw))
    y += speed * dt * math.sin(math.radians(-yaw))
    yaw += math.degrees(speed * dt * math.tan(MAX_STEER_RAD * steer) / WHEELBASE)
    yaw = (yaw + 180.0) % 360.0 - 180.0
    return x, y, yaw, speed


def slot_offset(x, y, yaw):
    """真实坐标下相对车位中心的 (dx, dy) 与航向误差（车位 270°/90° 都算平行）。"""
    dx = x - SLOT["x"]
    dy = y - SLOT["y"]
    err = abs((yaw - SLOT["ori"] + 180.0) % 360.0 - 180.0)
    err = min(err, abs(180.0 - err))
    return dx, dy, err


def main() -> int:
    print("[1] 路由与解析")
    check("kind_of('平行车道泊车') == 'park'", kind_of("平行车道泊车") == "park",
          kind_of("平行车道泊车"))
    check("STRATEGY_BY_KIND 挂了 park", STRATEGY_BY_KIND.get("park") is ParkStrategy)
    check("别的元素不受影响",
          kind_of("车辆切出") == "" and kind_of("辅路驶入、驶出") == "fudao")

    strat = ParkStrategy(ctx_at(*START, 6.7), route=[], log=None)
    cmd = strat.step(ctx_at(*START, 6.7, obstacles=[CAR_FRONT, CAR_BACK]))
    check("无车位时退回基线（返回 None）", cmd is None, repr(cmd))

    strat = ParkStrategy(ctx_at(*START, 6.7), route=[], log=None)
    cmd = strat.step(ctx_at(*START, 6.7))
    check("车位识别", strat.slot is not None and abs(strat.slot[1] + 1380.69) < 1e-6,
          str(strat.slot))
    check("接近段接管（park=True、挂 D 挡）",
          isinstance(cmd, Command) and cmd.park and cmd.gear == GearMode.DRIVE
          and cmd.throttle is not None and (cmd.brake or 0.0) <= C.PARK_APPROACH_BRAKE + 1e-9,
          f"thr={cmd.throttle:.2f} brk={cmd.brake:.2f}")
    far_y = strat._far_car_y(ctx_at(*START, 6.7))
    check("远端车取车位前方那辆（旋 180° 后 y 更小）",
          abs(far_y - (2 * SLOT["y"] - CAR_FRONT["y"])) < 1e-6,
          f"far_y={far_y:.2f} 期望={2 * SLOT['y'] - CAR_FRONT['y']:.2f}")
    xp, yp, yawp = strat._canonical(*START)
    check("规范坐标系：起点朝 −y（yaw=90）", abs(yawp - 90.0) < 1e-6,
          f"({xp:.2f}, {yp:.2f}) yaw={yawp:.1f}")
    check("规范坐标系：起点在车位 +x 侧 0.92 m",
          abs((xp - SLOT["x"]) - 0.92) < 0.02, f"侧向 {xp - SLOT['x']:.2f} m")
    tx, ty = strat._staging(ctx_at(*START, 6.7))
    check("staging 侧距 = PARK_LATERAL_M", abs((tx - SLOT["x"]) - C.PARK_LATERAL_M) < 1e-6,
          f"侧距 {tx - SLOT['x']:.2f} m")
    check("staging 在车位中心前方 7.3~7.6 m（Δ=2R·sinθ）",
          7.30 <= (SLOT["y"] - ty) <= 7.65, f"Δ={SLOT['y'] - ty:.2f} m")

    print("[2] 全程运动学仿真（接近 → staging → 两弧 → FIT → 驻车）")
    x, y, yaw = START
    speed = 6.7
    strat = ParkStrategy(ctx_at(x, y, yaw, speed), route=[], log=None)
    handover_frame = None
    final = None
    max_steer = 0.0
    for frame in range(6000):
        ctx = ctx_at(x, y, yaw, speed)
        cmd = strat.step(ctx)
        if cmd is None:
            break
        if strat.handed and handover_frame is None:
            handover_frame = frame
        max_steer = max(max_steer, abs(float(cmd.steer or 0.0)))
        x, y, yaw, speed = integrate(x, y, yaw, speed, cmd, ctx.dt)
        if strat.controller is not None and strat.controller.done() and abs(speed) < 0.05:
            final = (x, y, yaw, speed, frame)
            break

    check("接近段在 6000 帧内交给控制器", handover_frame is not None,
          f"frame={handover_frame}")
    check("控制器跑完（DONE 且停稳）", final is not None,
          "" if final is None else f"frame={final[4]}")
    if final is not None:
        dx, dy, err = slot_offset(*final[:3])
        check("入库后横向 |dx| ≤ 0.35 m", abs(dx) <= 0.35, f"dx={dx:+.2f}")
        check("入库后纵向 |dy| ≤ 0.60 m", abs(dy) <= 0.60, f"dy={dy:+.2f}")
        check("入库后航向误差 ≤ 5°", err <= 5.0, f"{err:.1f}°")
    check("舵角从不打满（≤ 0.80）", max_steer <= 0.80, f"max={max_steer:.2f}")

    print()
    if FAILED:
        print(f"失败 {len(FAILED)} 项：" + "；".join(FAILED))
        return 1
    print(f"全部通过（{COUNT} 项）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
