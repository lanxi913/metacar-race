# -*- coding: utf-8 -*-
r"""港口迷雾 · 基线模板（先拿 400 分）—— 骨架，只有心跳与状态打印

400 分怎么来（规则 PDF）：
    3 个信号各 80 分：巡检车按 A→B→C 顺序，7 m 内读到验证码后 submit_key
    路障扫描 160 分：三信号齐了以后，任一辆巡检车进 10 m 内 scan

三条一票归零的红线（碰一条本局 0 分）：
    穿越封闭道路（路障中心线两侧各 7 m）／未获通行权进中央路口 6 m 互斥区／
    车辆中心出界 1 m 持续 1 秒

心跳纪律：
    每 ≤1 s（队内约定 0.2 s）必须给四辆车都发一次控制，超时会被安全停车。

本文件提供：
    1) ScenarioTaskAPI(port=9100) 连接（多车场景是另一个端口，别用 5061）
    2) 每 0.2 s 发一次四车心跳 + 打印任务状态（verified_signal_count 等）
    3) 400 分主干的状态机骨架（信号 → 扫描），移动/读码部分是 TODO

用法：
    .venv\Scripts\python.exe -u templates\portfog\zyyxnjydhcg\run.py
    然后在客户端启动「港口迷雾」
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar_multiagent import GearMode, ScenarioTaskAPI, VehicleControl  # noqa: E402

from metacar_race.config import (  # noqa: E402
    CONTROL_PERIOD_SECONDS,
    READ_KEY_DISTANCE_M,
    SCAN_ROADBLOCK_DISTANCE_M,
)


def idle_controls(vehicle_ids):
    """四车统一刹停，用于心跳占位（安全第一）。"""
    return {
        vid: VehicleControl(gear=GearMode.PARKING, brake=1.0)
        for vid in vehicle_ids
    }


def pick_pending_signal(state):
    """当前待核验的信号对象。"""
    for obj in state.shared_task_objects:
        if getattr(obj, "object_type", "") == "signal":
            return obj
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="港口迷雾 400 分骨架")
    parser.add_argument("--max-seconds", type=float, default=600.0)
    args = parser.parse_args()

    print("连接多车场景（端口 9100，先跑脚本再点启动场景）...", flush=True)
    with ScenarioTaskAPI(port=9100) as api:
        vehicles = api.get_vehicle_ids()
        print(f"车辆: {vehicles}", flush=True)
        if not vehicles:
            print("没读到车辆列表，检查场景是否真的起来了。", flush=True)
            return 1

        last_report = -1.0
        while True:
            state = api.get_task_state()
            if state.used_time >= args.max_seconds:
                break

            if state.verified_signal_count < state.total_signal_count:
                signal = pick_pending_signal(state)
                # TODO(D)：派巡检车开到 signal.position 5 m 内停稳，
                #          读画面里的验证码，再 api.interact(vid, object_id, "submit_key", {"key": ...})
                #          没读到就继续靠近，别硬提交（提交错 key 浪费时间）。
                _ = signal
            elif getattr(state, "roadblock_active", False):
                # TODO(D)：巡检车进 10 m 内 scan，成功后队伍拿到 road_ids / lane_ids
                pass
            else:
                # TODO(D)：运输车申请路口、对侧驶出、release；最后泊位交付。
                pass

            api.set_vehicle_controls(idle_controls(vehicles))

            # 每 5 秒打一行状态，方便看进度
            if state.used_time - last_report >= 5.0:
                last_report = state.used_time
                print(
                    f"[{state.used_time:6.1f}s] "
                    f"信号 {state.verified_signal_count}/{state.total_signal_count}  "
                    f"读码距离阈值={READ_KEY_DISTANCE_M} 扫描阈值={SCAN_ROADBLOCK_DISTANCE_M}",
                    flush=True,
                )


            time.sleep(CONTROL_PERIOD_SECONDS)

        api.set_vehicle_controls(idle_controls(vehicles))
    print("结束。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

