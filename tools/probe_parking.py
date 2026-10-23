# -*- coding: utf-8 -*-
"""探测脚本：连上场景后只读不控，把车位/障碍物几何打出来。

用法（先跑脚本，再开场景）：
    .venv\\Scripts\\python.exe -u tools/probe_parking.py

输出：
    - 静态数据（路线点、子场景、VLA 标记）
    - 前 40 个障碍物：类型、位置、朝向、尺寸
    - 其中 PARKING_SLOT（类型 30）的单独汇总，用来标定 staging 距离
"""
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

PARKING_SLOT_TYPE = 30


def main() -> int:
    print("等待场景连接（先跑本脚本，再点启动场景）...")
    api = SceneAPI()
    api.connect()
    print("已连接。")

    static = api.get_scene_static_data()
    print(f"路线点: {len(static.route)}")
    print(f"子场景: {[s.name for s in static.sub_scenes]}")

    seen = False
    start = time.time()

    for sim_car_msg, _frames in api.main_loop():
        # 只刹停，不动车
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))

        if not seen and sim_car_msg.obstacles:
            seen = True
            obs = list(sim_car_msg.obstacles)
            kinds = Counter(getattr(getattr(o, "type", None), "name", str(getattr(o, "type", "?")))
                            for o in obs)
            print(f"\n障碍物 {len(obs)} 个，类型统计: {dict(kinds)}")

            print("\n--- 前 40 个 ---")
            print(f"{'idx':>3} {'type':<18} {'pos_x':>8} {'pos_y':>8} {'ori_z':>7} "
                  f"{'len':>6} {'wid':>6} extra")
            for i, o in enumerate(obs[:40]):
                tname = getattr(getattr(o, "type", None), "name", str(getattr(o, "type", "?")))
                print(f"{i:>3} {tname:<18} {o.pos_x:>8.2f} {o.pos_y:>8.2f} "
                      f"{o.ori_z:>7.1f} {o.length:>6.2f} {o.width:>6.2f} "
                      f"{getattr(o, 'extra_info', '') or ''}")

            slots = []
            for o in obs:
                v = getattr(getattr(o, "type", None), "value", getattr(o, "type", None))
                if v == PARKING_SLOT_TYPE:
                    slots.append(o)
            print(f"\n--- 车位（类型 {PARKING_SLOT_TYPE}）共 {len(slots)} 个 ---")
            for i, s in enumerate(slots):
                print(f"  #{i} center=({s.pos_x:.2f}, {s.pos_y:.2f}) "
                      f"ori_z={s.ori_z:.1f} 尺寸={s.length:.2f}x{s.width:.2f} "
                      f"extra={getattr(s, 'extra_info', '') or ''}")

            pose = sim_car_msg.pose_gnss
            print(f"\n自车: pos=({pose.pos_x:.2f}, {pose.pos_y:.2f}) yaw={pose.ori_z:.2f}")
            print("\n采集完成，5 秒后退出（不控制车辆）")
            time.sleep(5)
            break

        if time.time() - start > 30:
            print("30 秒内没有收到障碍物数据，退出")
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
