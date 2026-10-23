# -*- coding: utf-8 -*-
r"""VLA 文本导航 · 基线模板（目标 1000 = 完成 600 + 文本 400）—— 骨架，OCR 未接

评分拆解（规则 PDF）：
    完成分 600：100 s 内到达楼门口 8 m × 8 m 有效区并停稳
    文本分 400：OCR 160 / 时间 80 / 位置 80 / 动作 80，按归一化编辑距离算

提交格式硬要求（直接决定编辑距离分）：
    不换行、不加空格、英文标点、小写
    系统只把中文逗号/冒号转英文，换行和全角空格不会被清 —— 千万别有

提交方式：不是交文件，而是**每帧随控制一起**发一个 VLAExtensionOutput：
    api.set_vehicle_control(VehicleControl(...), VLAExtensionOutput(text_info=..., ...))

本文件提供：
    1) 连接场景 → 判断是不是 VLA 场景（static.vla_extension）
    2) 取摄像头帧 → 调用 perception.ocr 的分句函数 → 组装 VLATextOutput
    3) 控制 + 提交的完整调用链（OCR 那段是 TODO，先跑通链路）

用法：
    .venv\Scripts\python.exe -u templates\vla-text\zyyxnjydhcg\run.py --dry-run

    --dry-run 只打印识别到的文本与拆分结果，不提交（没有 OCR 时安全）
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from metacar import (  # noqa: E402
    GearMode,
    SceneAPI,
    VehicleControl,
    VLAExtensionOutput,
    VLATextOutput,
)

from metacar_race.perception.ocr import split_instruction  # noqa: E402

TARGET_SPEED_MS = 8.0        # VLA 限速 30 km/h ≈ 8.3 m/s，留一点余量
VLA_LIMIT_KMH = 30.0


def read_sign(_frame) -> str:
    """TODO(C)：说明牌 OCR。

    建议顺序：转灰度 → 二值化 → 只保留中间 1/2 区域（上下常有干扰）→
    交给 OCR（平台镜像里没有 tesseract，先用模板匹配或自训练的小模型）。
    识别不出来就返回空串，别乱猜——错文本比没文本扣得更多。
    """
    return ""


def main() -> int:
    parser = argparse.ArgumentParser(description="VLA 文本导航骨架")
    parser.add_argument("--dry-run", action="store_true", help="不提交，只打印")
    parser.add_argument("--max-seconds", type=float, default=100.0)
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    print("已连接。", flush=True)

    static = api.get_scene_static_data()
    print(
        f"VLA 场景: {'是' if static.vla_extension else '否'}  "
        f"子场景: {[s.name for s in static.sub_scenes]}",
        flush=True,
    )
    if not static.vla_extension:
        print("这不是 VLA 场景，换场景再跑。", flush=True)
        return 1

    frames = 0
    last_text = ""
    try:
        for msg, camera_frames in api.main_loop():
            frames += 1
            car = msg.main_vehicle
            status = msg.scene_status
            frame = camera_frames[0].frame if camera_frames else None

            if frame is not None:
                text = read_sign(frame)
                if text and text != last_text:
                    last_text = text
                    parts = split_instruction(text)
                    print(
                        f"[{status.used_time:6.1f}s] OCR={text!r} "
                        f"time={parts['time']!r} place={parts['place']!r} "
                        f"action={parts['action']!r}",
                        flush=True,
                    )

            vc = VehicleControl(
                gear=GearMode.DRIVE,
                throttle=0.3,
                brake=0.0,
                steering=0.0,
            )
            if not args.dry_run and last_text:
                parts = split_instruction(last_text)
                out = VLAExtensionOutput(
                    text_info=VLATextOutput(
                        ocr_text=last_text,
                        time_phrase=parts["time"],
                        location_phrase=parts["place"],
                        action_phrase=parts["action"],
                    ),
                    parking_result=None,
                    function_zone_result=None,
                )
                api.set_vehicle_control(vc, out)
            else:
                api.set_vehicle_control(vc)

            if frames % 30 == 1:
                print(
                    f"[{status.used_time:6.1f}s] v={float(car.speed):5.2f} "
                    f"frames={frames} text={'有' if last_text else '无'}",
                    flush=True,
                )
            if status.used_time >= args.max_seconds:
                break
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass

    print(f"结束，共 {frames} 帧。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
