"""离线冒烟：元素 12 平行车道泊车（`scene4/parkslot.py`，PR#43 移植版）。

分两段验证：
    1) 接线：`kind_of("平行车道泊车")=="parking"`、策略挂着、无车位时退回基线、车位解析、
       基向量（模板系 yaw_t 恒 90）；
    2) 几何：用运动学模型把车从元素 12 起点开一遍（ALIGN → STOP → PARK），检查
       staging 落点、交班时控制器从 STOP 起手、倒挡真的动起来、入库落点（dx/dy）。

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
from metacar_race.scene4.parkslot import (  # noqa: E402
    SlotParkStrategy, find_slot)
from metacar_race.scene4.strategies import (  # noqa: E402
    STRATEGY_BY_KIND, ElementContext, kind_of)

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
DT = 1.0 / 30.0


def ctx_at(x, y, yaw, speed, obstacles=OBSTACLES, dt=DT):
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


def main() -> int:
    print("[1] 接线")
    check('kind_of("平行车道泊车") == "parking"', kind_of("平行车道泊车") == "parking",
          kind_of("平行车道泊车"))
    check('STRATEGY_BY_KIND["parking"] 是 SlotParkStrategy',
          STRATEGY_BY_KIND.get("parking") is SlotParkStrategy)

    # 无车位 → 交回基线
    x, y, yaw = START
    strat = SlotParkStrategy(ctx_at(x, y, yaw, 6.0),
                             route=[SimpleNamespace(x=SLOT["x"], y=START[1]),
                                    SimpleNamespace(x=SLOT["x"], y=SLOT["y"])])
    check("没有车位（type=30）时返回 None",
          strat.step(ctx_at(x, y, yaw, 6.0, obstacles=[CAR_FRONT, CAR_BACK])) is None)

    # 有车位 → 解析 + 定基向量
    cmd = strat.step(ctx_at(x, y, yaw, 6.0))
    check("看到车位后接管（Command 非 None）", cmd is not None)
    check("接管即 final=True（driver 不许抢 EndPark/升挡）", bool(cmd and cmd.final))
    check("车位解析 5.5x2.5 / ori=270",
          strat.frame is not None and abs(strat.frame.length - 5.5) < 1e-6
          and abs(strat.frame.width - 2.5) < 1e-6 and abs(strat.frame.ori - 270.0) < 1e-6)
    X0, Y0, yaw0 = strat.to_t(x, y, yaw)
    check("模板系 yaw_t = 90°（按自车来向定轴）", abs(yaw0 - 90.0) < 0.5, f"yaw_t={yaw0:.1f}")
    check("侧向偏置取经验值 3.15 m（本机没有场景包）",
          abs(abs(strat.lat_target) - 3.15) < 1e-6,
          f"lat={strat.lat_target:+.2f} 来源={strat.lat_source}")
    check("staging Y = −5.90 m（车位半长 + 半车长 + 0.8）",
          abs(strat.stage_y + 5.90) < 1e-6, f"stage_y={strat.stage_y:+.2f}")

    print("[2] ALIGN → STOP")
    speed = 6.0
    t = 0.0
    frames = 0
    while frames < 4000 and strat.phase == "ALIGN":
        cmd = strat.step(ctx_at(x, y, yaw, speed))
        if cmd is None:
            break
        x, y, yaw, speed = integrate(x, y, yaw, speed, cmd, DT)
        t += DT
        frames += 1
    check("ALIGN 阶段能在 60 s 内到达 staging", strat.phase in ("STOP", "PARK"),
          f"phase={strat.phase} t={t:.1f}s frames={frames}")
    _, Ys, _ = strat.to_t(x, y, yaw)
    check("staging 纵向落点 |Y − stage_y| ≤ 1.2 m", abs(Ys - strat.stage_y) <= 1.2,
          f"Y={Ys:+.2f} 目标={strat.stage_y:+.2f}")
    check("staging 横向落点 |X − lat_target| ≤ 1.0 m",
          abs(strat.to_t(x, y, yaw)[0] - strat.lat_target) <= 1.0,
          f"X={strat.to_t(x, y, yaw)[0]:+.2f} 目标={strat.lat_target:+.2f}")

    # STOP 段：先把速度压到 0
    for _ in range(120):
        cmd = strat.step(ctx_at(x, y, yaw, speed))
        if cmd is None:
            break
        x, y, yaw, speed = integrate(x, y, yaw, speed, cmd, DT)
        if strat.phase != "STOP":
            break
    check("停稳后控制器从 STOP 起手（绕开 0.010 油门推不动的 APPROACH）",
          strat.ctl is not None and strat.ctl.phase in ("STOP", "ARC1", "ARC2", "FIT", "DONE"),
          f"ctl.phase={None if strat.ctl is None else strat.ctl.phase}")

    print("[3] PARK：两段圆弧倒车入库")
    got_reverse = False
    done = False
    frames = 0
    while frames < 3600:
        cmd = strat.step(ctx_at(x, y, yaw, speed))
        if cmd is None:
            break
        if cmd.gear == GearMode.REVERSE:
            got_reverse = True
        x, y, yaw, speed = integrate(x, y, yaw, speed, cmd, DT)
        frames += 1
        if strat.phase == "DONE":
            done = True
            break
    check("倒车段真的挂过 R 挡", got_reverse)
    check("入库完成（controller.done()）", done,
          f"phase={strat.phase} frames={frames} t={frames * DT:.1f}s")
    if strat.ctl is not None:
        dx = float(getattr(strat.ctl, "dx", 0.0) or 0.0)
        dy = float(getattr(strat.ctl, "dy", 0.0) or 0.0)
        check("入库落点 |dx| ≤ 0.8 m", abs(dx) <= 0.8 or not done, f"dx={dx:+.2f}")
    # DONE 之后：驻车保持
    hold = strat.step(ctx_at(x, y, yaw, 0.0))
    check("DONE 后输出驻车保持（P 挡 + 满刹 + final）",
          hold is not None and hold.gear == GearMode.PARKING
          and float(hold.brake or 0.0) >= 0.99 and bool(hold.final))

    print("[4] 复位")
    strat.sync(0.0, 0.0)
    check("sync() 回到 WAIT 并清掉控制器", strat.phase == "WAIT" and strat.frame is None)

    print()
    if FAILED:
        print(f"失败 {len(FAILED)}/{COUNT}：{FAILED}")
        return 1
    print(f"全部通过（{COUNT} 项）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
