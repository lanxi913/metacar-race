# -*- coding: utf-8 -*-
"""临时验证脚本：把 `scene4/parkslot.py`（车位坐标系侧方停车）单独放到
「平行车道泊车」单场景里跑一遍，用来确认包装层的坐标系换算与 staging 逻辑
和参考实现（`templates/parking/zyyxnjydhcg/parallel_run.py`，单场景 210）等价。

跑法（先脚本、后场景）：
    .venv\\Scripts\\python.exe -u tools\\_park_slot_test.py
    客户端第 2 页 →「平行车道泊车」→ 启动场景
"""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from metacar import GearMode, SceneAPI, VehicleControl

from metacar_race.scene4.parkslot import SlotParkStrategy
from metacar_race.scene4.strategies import ElementContext


def main() -> int:
    api = SceneAPI()
    print("waiting scene ...", flush=True)
    api.connect()
    print("connected", flush=True)
    static = api.get_scene_static_data()
    route = list(getattr(static, "route", []) or [])
    print("sub_scenes:", [s.name for s in static.sub_scenes], flush=True)

    strat = None
    last_t = None
    frame = 0
    t0 = time.time()
    try:
        for msg, _cam in api.main_loop():
            frame += 1
            pose, car, st = msg.pose_gnss, msg.main_vehicle, msg.scene_status
            dt = 1.0 / 30.0 if last_t is None else max(st.used_time - last_t, 1e-3)
            last_t = st.used_time
            ctx = ElementContext(
                x=float(pose.pos_x), y=float(pose.pos_y), yaw=float(pose.ori_z),
                speed=float(car.speed), dt=dt, sim_t=float(st.used_time), s_now=0.0,
                pose=pose, obstacles=list(msg.obstacles or []), static=static,
                ego_length=float(getattr(car, "length", 4.70) or 4.70),
                ego_width=float(getattr(car, "width", 1.90) or 1.90),
                vmax=8.0,
            )
            if strat is None:
                strat = SlotParkStrategy(ctx, route, log=lambda s: print(s, flush=True))
            cmd = strat.step(ctx)
            if cmd is None:
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.30)
            else:
                vc = VehicleControl(
                    gear=cmd.gear if cmd.gear is not None else GearMode.DRIVE,
                    throttle=float(cmd.throttle or 0.0),
                    brake=float(cmd.brake or 0.0),
                    steering=float(cmd.steer or 0.0),
                )
            api.set_vehicle_control(vc)
            if frame % 30 == 1:
                try:
                    tx_, ty_, tyaw_ = strat.to_t(ctx.x, ctx.y, ctx.yaw)
                except Exception:                        # noqa: BLE001
                    tx_, ty_, tyaw_ = float("nan"), float("nan"), float("nan")
                print("[%6.1fs] %-9s x=%8.2f y=%8.2f yaw=%6.1f | X=%+6.2f Y=%+7.2f yaw_t=%6.1f "
                      "v=%5.2f thr=%.3f brk=%.2f steer=%+.2f %s"
                      % (st.used_time, strat.phase, ctx.x, ctx.y, ctx.yaw, tx_, ty_, tyaw_,
                         ctx.speed, vc.throttle, vc.brake, vc.steering,
                         "" if cmd is None else cmd.note), flush=True)
            if strat.phase == "DONE" or getattr(strat, "done", False) or (time.time() - t0) > 150.0:
                print("finish: phase=%s frames=%d" % (strat.phase, frame), flush=True)
                break
    except Exception as exc:                             # noqa: BLE001
        print("!! loop error:", repr(exc), flush=True)
        raise
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0))
        except Exception:                                # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
