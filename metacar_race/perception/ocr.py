# -*- coding: utf-8 -*-
"""指令文本解析（负责人：C）。

输入：摄像头 OCR 出来的整句，例如
    "在100秒内到达锦汇华庭B栋一单元门口，并且全程不得超过30km/h"
输出：拆成 时间 / 地点 / 动作 三段，供 VLATextOutput 提交。

提交格式硬要求（得分直接相关）：
  - 不换行、不加空格
  - 用英文标点（系统只把中文逗号冒号转成英文，其它标点不处理）
  - 小写

TODO(C): 先用正则 + 关键词表做出能用的版本，别一上来就上大模型。
"""
import re


def split_instruction(text: str) -> dict:
    """把一句任务指令拆成 {time, place, action}。

    目前只做最粗的切分：按"到达/去/前往"定位动作，按"内/秒/分钟"定位时间。
    真实的拆分口径见规则文档附录 A.4 的示例。
    """
    if not text:
        raise ValueError("text 为空")

    s = text.strip()

    # 时间：'100秒内' / '30秒内' / '2分钟内'
    time_match = re.search(r"\d+\s*(?:秒|分钟)\s*内?", s)
    time_phrase = time_match.group(0) if time_match else ""

    # 动作：从"到达/去/前往/找到"开始，到下一个分句结束
    action_match = re.search(r"(到达|去|前往|找到|停|泊车).*", s)
    action_phrase = action_match.group(0) if action_match else ""

    # 地点：去掉时间短语和动作词之后剩下的部分
    place = s
    if time_phrase:
        place = place.replace(time_phrase, "")
    for verb in ("到达", "前往", "去到", "去", "找到"):
        place = place.replace(verb, "")
    place_phrase = place.strip(" ，,。")

    return {
        "time": time_phrase,
        "place": place_phrase,
        "action": action_phrase,
    }
