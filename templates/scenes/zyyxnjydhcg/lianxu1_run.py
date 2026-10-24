# -*- coding: utf-8 -*-
r"""连续场景 1 · 组合控制器入口

组合思路（"用好已训练好的子赛题模型"）：
    行驶元素（直角转弯 / 车辆变道 / 车辆直行 / 连续转弯）
        → `GeoFollower`：规则 + 车辆 + 场景数据驱动，无经验参数
    泊车元素（垂直泊车）
        → `VerticalPark`：直接复用 `tools/park_v4.py`（元素场景实测 210 分）的动作序列

切换依据是平台下发的 `scene_status.sub_scene_name`，不写死任何坐标。

用法：
    .venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\lianxu1_run.py
    → 客户端 第 3 页 →「连续场景1」→ 点「启动场景」
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from metacar_race.control.geo_follower import GeoFollower  # noqa: E402
from metacar_race.control.vertical_park import VerticalPark  # noqa: E402

PARK_KEYWORD = "泊车"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-seconds", type=float, default=900.0)
    ap.add_argument("--jsonl", type=str, default="logs/lianxu1.jsonl")
    args = ap.parse_args()

    print("等待场景连接（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"已连接。子场景: {[s.name for s in static.sub_scenes]}", flush=True)

    pts = [(float(p.x), float(p.y)) for p in static.route]
    geo = GeoFollower(pts)
    park = None
    mode = "ROUTE"
    print(f"路线 {len(pts)} 点，总长 {geo.path.total:.1f} m；"
          f"限速 {geo.v_cap:.1f} m/s，横向加速度上限 5.0 m/s²", flush=True)

    fh = open(args.jsonl, "w", encoding="utf-8")
    frame = 0
    t0 = None
    last_sub = ""
    try:
        for msg, _cam in api.main_loop():
            frame += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            st = msg.scene_status
            if t0 is None:
                t0 = st.used_time
            sub = getattr(st, "sub_scene_name", "") or ""
            if sub != last_sub:
                last_sub = sub
                print(f"  == 子场景切换：{sub} ==", flush=True)

            # 泊车元素一开始就切会离车位太远（实测 180 m）；等进入车位附近再交给泊车控制器
            slot_now = None
            for o in msg.obstacles:
                if getattr(getattr(o, "type", None), "value", None) == 30:
                    slot_now = o
                    break
            near_slot = False
            if slot_now is not None:
                d_s = ((slot_now.pos_x - float(pose.pos_x)) ** 2 +
                       (slot_now.pos_y - float(pose.pos_y)) ** 2) ** 0.5
                near_slot = d_s < 8.0
            # 只有开到车位口（<8 m）且路线基本走完时才交给泊车控制器，避免远距离倒车入场
            want_park = (PARK_KEYWORD in sub) and near_slot and (geo.path.total - (geo._s_hint or 0.0)) < 12.0
            if want_park and mode != "PARK":
                mode = "PARK"
                park = VerticalPark()
                park._latched = True      # 进入泊车模式后不再切回循迹，避免模式抖动
                slot_dbg = None
                for o in msg.obstacles:
                    if getattr(getattr(o, "type", None), "value", None) == 30:
                        slot_dbg = o
                        break
                if slot_dbg is not None:
                    d_slot = ((slot_dbg.pos_x - float(pose.pos_x)) ** 2 +
                              (slot_dbg.pos_y - float(pose.pos_y)) ** 2) ** 0.5
                    print(f"  → 切入垂直泊车：车位({float(slot_dbg.pos_x):.1f},{float(slot_dbg.pos_y):.1f}) "
                          f"ori={float(slot_dbg.ori_z):.0f} 距车 {d_slot:.1f} m "
                          f"车({float(pose.pos_x):.1f},{float(pose.pos_y):.1f}) 剩余路线 {geo.path.total - geo._s_hint if geo._s_hint else -1:.1f} m",
                          flush=True)
                else:
                    print("  → 切入垂直泊车：当前没有 PARKING_SLOT 障碍物", flush=True)
            elif not want_park and mode != "ROUTE" and not getattr(park, "_latched", False):
                mode = "ROUTE"
                print("  → 回到几何循迹", flush=True)

            if mode == "PARK":
                vc, phase = park.update(pose, float(car.speed), msg.obstacles, 1 / 30)
                info = {"s": 0.0, "e": 0.0, "rem": 0.0, "v_cmd": 0.0, "k": 0.0,
                        "L": 0.0, "steer": getattr(vc, "steering", 0.0), "k_plant": 0.0}
                tag = f"PARK:{phase}"
            else:
                vc, info = geo.update(pose, float(car.speed), 1 / 30)
                tag = "ROUTE"

            api.set_vehicle_control(vc)
            fh.write(f'{{"t":{st.used_time:.2f},"sub":"{sub}","x":{float(pose.pos_x):.2f},'
                     f'"y":{float(pose.pos_y):.2f},"v":{float(car.speed):.2f},'
                     f'"s":{info["s"]:.2f},"e":{info["e"]:.2f},"tag":"{tag}"}}\n')
            if frame % 30 == 1:
                print(f"[{st.used_time:6.1f}s] {tag:12s} s={info['s']:7.1f} e={info['e']:+5.2f} "
                      f"v={float(car.speed):5.2f}/{info['v_cmd']:5.2f} κ={info['k']:.3f} "
                      f"pos=({float(pose.pos_x):7.1f},{float(pose.pos_y):7.1f})", flush=True)

            if (mode == "PARK" and park and park.done) or geo.done or st.used_time >= args.max_seconds:
                print(f"结束：mode={mode} geo_done={geo.done} frames={frame} "
                      f"s={info['s']:.1f}/{geo.path.total:.1f}", flush=True)
                break
    except KeyboardInterrupt:
        print("手动中断", flush=True)
    finally:
        fh.close()
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())




