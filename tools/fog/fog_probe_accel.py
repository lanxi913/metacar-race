# -*- coding: utf-8 -*-
"""加速度响应探针：摸清平台怎么算「加速度超限」，以及它到底给了我们哪些量。

做三件事：
1. dump 平台暴露的字段（vehicle_state / task_state / events 的原始内容）；
2. 阶跃实验：静止 → 油门阶跃 0.55 → 刹车阶跃 0.9 → 油门 0.30 → 刹车 1.0，10 Hz 采样；
3. 同时算三种加速度，对齐平台记的 violation：
      a_speed = d(speed_mps)/dt          标量速度差分
      a_ax    = d(vel_x)/dt              速度矢量的 x 分量差分
      a_vec   = |d(v_vec)/dt|            速度矢量变化率的模长
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from metacar.models import GearMode, VehicleControl                 # noqa: E402
from fog_map import connect_retry, to_plain                         # noqa: E402


def plain(x):
    try:
        return to_plain(x)
    except Exception:                                          # noqa: BLE001
        pass
    if isinstance(x, dict):
        return x
    d = getattr(x, "__dict__", None)
    if isinstance(d, dict):
        return {k: v for k, v in d.items() if not k.startswith("_")}
    return {"repr": repr(x)}


def pose_vec(st):
    """从 vehicle_state 里取 (speed, vx, vy, x, y, ori_z)。"""
    pose = st.get("pose")
    txt = pose if isinstance(pose, str) else str(pose)
    kv = {}
    for tok in txt.replace(",", " ").split():
        if "=" in tok:
            k, _, v = tok.partition("=")
            try:
                kv[k.strip()] = float(v)
            except ValueError:
                pass
    sp = float(st.get("speed_mps") or 0.0)
    return (sp, kv.get("vel_x", 0.0), kv.get("vel_y", 0.0),
            kv.get("pos_x", 0.0), kv.get("pos_y", 0.0), kv.get("ori_z", 0.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    log = open(os.path.join(args.out, "probe.log"), "w", encoding="utf-8")

    def w(msg):
        line = "[%6.2f] %s" % (time.time() - t0, msg)
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

    api = connect_retry()
    t0 = time.time()
    vids = list(api.get_vehicle_ids())
    w("vehicles=%s" % vids)

    # ---------- 1) 平台给我们的字段 ----------
    st = plain(api.get_vehicle_state("Car_D"))
    w("[field] vehicle_state keys = %s" % sorted(st.keys()))
    w("[field] vehicle_state = %s" % json.dumps(st, ensure_ascii=False, default=str)[:800])
    try:
        ts = plain(api.get_task_state())
        w("[field] task_state keys = %s" % sorted(ts.keys()))
        objs = ts.get("shared_task_objects") or []
        w("[field] task_state.shared_task_objects 首条 = %s"
          % (str(objs[0])[:400] if objs else "-"))
    except Exception as exc:                                   # noqa: BLE001
        w("[field] task_state 读取失败 %s" % exc)
    try:
        static = plain(api.get_scene_static_data())
        w("[field] static keys = %s" % sorted(static.keys()))
        veh = (static.get("vehicles") or [None])[0]
        w("[field] 车辆规格 = %s" % (str(veh)[:300] if veh else "-"))
    except Exception as exc:                                   # noqa: BLE001
        w("[field] static 读取失败 %s" % exc)
    try:
        evs = plain(api.get_events(0))
        if isinstance(evs, dict):
            evs = evs.get("events") or []
        w("[field] events 类型=%s 条数=%s 首条=%s"
          % (type(evs).__name__, len(evs),
             json.dumps(evs[0], ensure_ascii=False, default=str)[:300] if evs else "-"))
    except Exception as exc:                                   # noqa: BLE001
        w("[field] events 读取失败 %s" % exc)

    # ---------- 2) 阶跃实验 ----------
    car = "Car_C"
    # (hold_s, tag, thr, brk, gear)   gear: 1=蠕动 2=前进 3=驻车
    # 脉冲实验：on=给油帧数, off=断开帧数（10 Hz）
    SEQ = [(2.0, "静止", 0.0, 0.0, 2),
           (4.0, "脉冲1on1off", 0.30, 0.0, 2),
           (1.5, "刹停", 0.0, 1.0, 2),
           (4.0, "脉冲1on2off", 0.30, 0.0, 2),
           (1.5, "刹停", 0.0, 1.0, 2),
           (4.0, "脉冲1on3off", 0.30, 0.0, 2),
           (1.5, "刹停", 0.0, 1.0, 2),
           (3.0, "连续0.30", 0.30, 0.0, 2)]
    # 其余三辆车先驻车（四车并排停在起点，碰一下就刷一串 collision）
    for other in vids:
        if other == car:
            continue
        try:
            veh = VehicleControl(throttle=0.0, brake=1.0, steering=0.0)
            veh.gear = GearMode.PARKING
            api.set_vehicle_controls({other: veh})
        except Exception:                                      # noqa: BLE001
            pass
    w("=== 阶跃实验开始（%s，其余车已驻车）===" % car)
    rows = []
    ev_seq = 0
    for hold_s, tag, thr, brk, gear in SEQ:
        w("--- %s（%.1f s）thr=%.2f brk=%.2f gear=%d ---" % (tag, hold_s, thr, brk, gear))
        t_end = time.time() + hold_s
        on_n = off_n = 0
        if "on" in tag:
            parts = tag.replace("脉冲", "").split("off")[0].split("on")
            on_n = int(parts[0] or 1)
            off_n = int(parts[1].replace("off", "") or 1)
        frame = 0
        while time.time() < t_end:
            use_thr = thr
            if on_n:
                use_thr = thr if (frame % (on_n + off_n)) < on_n else 0.0
            frame += 1
            veh = VehicleControl(throttle=use_thr, brake=brk, steering=0.0)
            try:
                veh.gear = {2: GearMode.DRIVE, 3: GearMode.PARKING,
                            4: GearMode.REVERSE}.get(gear, GearMode.DRIVE)
            except Exception:                                  # noqa: BLE001
                pass
            try:
                api.set_vehicle_controls({car: veh})
                stv = plain(api.get_vehicle_state(car))
            except Exception as exc:                           # noqa: BLE001
                w("  控制/读取失败 %s" % exc)
                break
            sp, vx, vy, px, py, oy = pose_vec(stv)
            rows.append((time.time() - t0, tag, thr, brk, sp, vx, vy, px, py, oy))
            try:
                evs = plain(api.get_events(ev_seq))
                if isinstance(evs, dict):
                    evs = evs.get("events") or []
                for e in evs:
                    d = e if isinstance(e, dict) else plain(e)
                    if not isinstance(d, dict):
                        continue
                    seq = int(d.get("sequence") or 0)
                    if seq > ev_seq:
                        ev_seq = seq
                    if (d.get("event_type") or "") in ("violation", "warning", "penalty"):
                        w("  !! EVENT #%s t=%.2fs [%s] %s"
                          % (d.get("sequence"), float(d.get("elapsed_seconds") or -1),
                             d.get("event_type"), d.get("message")))
            except Exception:                                  # noqa: BLE001
                pass
            time.sleep(0.1)

    try:
        veh = VehicleControl(throttle=0.0, brake=1.0, steering=0.0)
        veh.gear = GearMode.PARKING
        api.set_vehicle_controls({car: veh})
    except Exception:                                          # noqa: BLE001
        pass

    # ---------- 3) 三种加速度 ----------
    w("=== 采样表：t tag thr brk speed a_speed a_ax a_vec ===")
    prev = None
    for (t, tag, thr, brk, sp, vx, vy, px, py, oy) in rows:
        if prev is None:
            da = dax = dvec = 0.0
        else:
            dt = max(t - prev[0], 1e-3)
            da = (sp - prev[1]) / dt
            dax = (vx - prev[2]) / dt
            dvec = math.hypot(vx - prev[2], vy - prev[3]) / dt
        w("%6.2f %-12s %.2f %.2f v=%.3f a_s=%+6.2f a_ax=%+6.2f a_vec=%5.2f (%.1f,%.1f,ori=%.0f)"
          % (t, tag, thr, brk, sp, da, dax, dvec, px, py, oy))
        prev = (t, sp, vx, vy)
    log.close()


if __name__ == "__main__":
    main()
