# -*- coding: utf-8 -*-
"""感知与 VLA 模块（负责人：C）。

VLA 的结果不是打印，而是随控制指令一起提交：

    out = VLAExtensionOutput(
        text_info=VLATextOutput(ocr_text=..., time_phrase=..., location_phrase=..., action_phrase=...),
        parking_result=ParkingResult(violating_sticker_ids=[...]),
        function_zone_result=None,
    )
    api.set_vehicle_control(vc, out)
"""
