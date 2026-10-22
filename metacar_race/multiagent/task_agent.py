# -*- coding: utf-8 -*-
"""港口迷雾任务状态机骨架（负责人：D）。

按 400 → 600 → 800 → 1000 四段推进：
  1) 巡检车按 A→B→C 提交验证码（7 m 内），得 240
  2) 三信号齐 → 巡检车 10 m 内 scan 路障，得 160
  3) 运输车 12 m 内 request_entry → 驶入 → 对侧出口 → release，每车 100
  4) 泊位 3 m 内停稳 → deliver，每个 200

注意：控制心跳 1 秒，超时会被安全停车；封闭道路 7 m 带内进入直接归零。
"""
from __future__ import annotations

import time

from metacar_multiagent import GearMode, ScenarioTaskAPI, VehicleControl


def pick_signal(state):
    """当前待核验的信号对象（object_type == 'signal'）。"""
    for obj in state.shared_task_objects:
        if obj.object_type == "signal":
            return obj
    return None


def idle_controls(vehicle_ids, brake=1.0):
    """四车统一的刹停控制，用于心跳占位。"""
    return {
        vid: VehicleControl(gear=GearMode.PARKING, brake=brake)
        for vid in vehicle_ids
    }


def run():
    with ScenarioTaskAPI(port=9100) as api:
        vehicles = api.get_vehicle_ids()
        while True:
            state = api.get_task_state()

            if state.verified_signal_count < state.total_signal_count:
                signal = pick_signal(state)
                # TODO(D): 派巡检车开到 signal.position 附近（≤5 m 停），
                #          读取画面里的验证码，再 submit_key。
                #          没读到就继续靠近，别硬提交。
                _ = signal

            elif state.roadblock_active:
                # TODO(D): 巡检车在 10 m 内 scan，成功后读 road_ids / lane_ids
                pass

            else:
                # TODO(D): 运输车申请路口、驶出后 release；再做泊位交付
                pass

            api.set_vehicle_controls(idle_controls(vehicles))
            time.sleep(0.2)


if __name__ == "__main__":
    run()
