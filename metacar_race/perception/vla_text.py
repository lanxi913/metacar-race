# -*- coding: utf-8 -*-
"""VLA 文本导航任务：指令拆分 + 目标匹配（纯函数，可离线单测）。

拆分口径严格照《VLA场景任务说明与评分规则》附录 A.4 的四条标准示例：

    在100秒内到达锦汇华庭B栋一单元门口，并且全程不得超过30km/h
      → 时间: 在100秒内
        地点: 锦汇华庭B栋一单元门口
        行动: 到达，并且全程不得超过30km/h

    100秒内找到空的停车位停车，并且全程不得超过30km/h
      → 时间: 100秒内
        地点: 空的停车位
        行动: 找到并停车，并且全程不得超过30km/h

要点（评分是归一化编辑距离，差一个字按比例扣）：
  - 行动段必须把"并且全程不得超过30km/h"一起带上，不能只留"到达"
  - 标点只把中文逗号/冒号换成英文，其它标点保留
  - 去掉普通空格，全角空格和换行不主动删（原样保留更安全）
"""
from __future__ import annotations

import re

# 动作词（按长度从长到短匹配，避免"去"先吃掉"去到"）
_ACTIONS = ("去到", "到达", "抵达", "前往", "找到", "开到", "驶到", "去", "到")

_TIME_RE = re.compile(
    r"(?:最迟在|最迟|在|限时|于)?\s*\d+\s*(?:秒|分钟|min|s)\s*(?:时间限制|限制)?\s*(?:内|以内)?"
)

# 行动里常见的"限速尾巴"，从它开始一直到句尾都算行动段。
# 正式赛题（邀请赛）的牌子至少有四种措辞（从场景包里读出来的）：
#   …停车，并且全程不得超过30km/h
#   …停车，行驶全程时速小于等于30km/h
#   …停车，途中最高车速须小于等于30km/h      ← 没有"全程"，旧正则切不出来
#   …停车，并全程控制时速小于等于30km/h
# 旧实现用一条正则找尾巴，`[，,]?` 是可选的，`.` 又能吃掉逗号，于是会从
# "…入口停车，行驶全程…" 的"车"字开始匹配，把"车，"切进行动段（实测
# place='…西入口停' / action='抵达车,…'）。改成"按逗号切句、取最后一个含限速
# 关键字的从句"，不再依赖正则的回溯。
_TAIL_KEYS = ("全程", "时速", "km/h", "KM/H", "最高车速", "不得超过", "小于等于")


def _split_tail(rest: str) -> tuple[str, str]:
    """把"地点句 + 限速从句"拆开，返回 (head, tail)；没有限速从句时 tail 为空。"""
    if not rest:
        return rest, ""
    clauses = re.split(r"[，,]", rest)
    for i in range(len(clauses) - 1, 0, -1):
        if any(k in clauses[i] for k in _TAIL_KEYS):
            # 用 ASCII 逗号拼接：normalize() 已经把逗号统一成 "，"→","，
            # 1000 分那几局提交的也是 "," 形态，别改回中文逗号。
            head = ",".join(clauses[:i])
            tail = "," + ",".join(clauses[i:])
            return head, tail
    return rest, ""


def normalize(text: str) -> str:
    """清洗成"平台标准答案"的形态。

    平台拿我们提交的字符串和标准答案做编辑距离，而标准答案是一行、无空格、
    英文标点；牌子上的折行/缩进都是 OCR 噪声，必须在这里去掉，否则相似度会被
    换行符吃掉一大截（实测过：带 \\n 时行动段落直接匹配失败）。
    """
    s = (text or "").strip()
    s = s.replace("，", ",").replace("：", ":").replace("；", ";")
    for ch in ("\r", "\n", "\t", "\u3000", " "):
        s = s.replace(ch, "")
    return s


def split_instruction(text: str) -> dict:
    """拆成 {ocr_text, time, place, action}，口径对齐官方附录 A.4。

    结构是：  [时间] + [动词] + [地点] + (并停车) + [限速尾巴]
    行动段 = 动词 + (并停车) + 尾巴；地点段 = 动词与尾巴之间去掉"停车"的部分。
    """
    ocr_text = normalize(text)
    if not ocr_text:
        return {"ocr_text": "", "time": "", "place": "", "action": ""}

    # 1) 时间片段
    m_time = _TIME_RE.search(ocr_text)
    time_phrase = m_time.group(0) if m_time else ""
    rest = ocr_text.replace(time_phrase, "", 1) if time_phrase else ocr_text

    # 2) 限速尾巴：从最后一个含"全程/时速/最高车速"的从句开始一直到句尾
    head, tail = _split_tail(rest)

    # 3) 动词 + 地点
    verb, verb_at = "", -1
    for v in _ACTIONS:
        i = head.find(v)
        if i >= 0 and (verb_at < 0 or i < verb_at):
            verb, verb_at = v, i

    if verb_at < 0:
        return {"ocr_text": ocr_text, "time": time_phrase,
                "place": head.strip(" ,.;"), "action": tail}

    remainder = head[verb_at + len(verb):].strip(" ,.;")
    action_extra = ""
    # 动词后面紧跟的"并停入/并驶入/并停车"要算进行动段，不能留在地点里。
    # 正式赛题的停车位变体就是这种写法："100秒内找到并停入147培训基地停车区01号停车位，…"
    for pre in ("并停入", "并驶入", "并进入", "停入", "驶入", "进入"):
        if remainder.startswith(pre):
            action_extra = "并" + pre.lstrip("并")
            remainder = remainder[len(pre):].strip(" ,.;")
            break
    if remainder.startswith("并"):
        action_extra = action_extra or "并"
        remainder = remainder[1:].strip(" ,.;")
    # 结尾的"停车"是动作（"…门口停车"），而"停车位"是地点的一部分，两者别搞混
    if remainder.endswith("停车"):
        remainder = remainder[: -len("停车")].strip(" ,.;")
        action_extra = "并停车"

    return {
        "ocr_text": ocr_text,
        "time": time_phrase,
        "place": remainder,
        "action": verb + action_extra + tail,
    }

def similarity(a: str, b: str) -> float:
    """归一化编辑距离相似度 = 1 - d / max(len)。用于自检 OCR 质量。"""
    a, b = normalize(a), normalize(b)
    if not a and not b:
        return 1.0
    if not a or not b:
        return 0.0
    la, lb = len(a), len(b)
    prev = list(range(lb + 1))
    for i in range(1, la + 1):
        cur = [i] + [0] * lb
        for j in range(1, lb + 1):
            cur[j] = min(
                prev[j] + 1,
                cur[j - 1] + 1,
                prev[j - 1] + (0 if a[i - 1] == b[j - 1] else 1),
            )
        prev = cur
    return 1.0 - prev[lb] / max(la, lb)


def _bigrams(s: str) -> set:
    s = normalize(s)
    return {s[i:i + 2] for i in range(len(s) - 1)} or {s}


def match_place(place_phrase: str, candidates) -> list:
    """把地点片段匹配到 buildings/regions 里，返回按分数降序的 [(score, item)]。

    评分用"相似度 + 字符覆盖率"的组合：OCR 常把楼号认错一个字，
    纯编辑距离会掉太狠，所以对"标准名是地点的后缀/子串"这种情况额外加权。
    """
    phrase = normalize(place_phrase)
    if not phrase:
        return []

    pb = _bigrams(phrase)
    out = []
    for item in candidates:
        name = normalize(getattr(item, "name", "") or getattr(item, "id", ""))
        if not name:
            continue
        base = similarity(phrase, name)

        # 覆盖率：地点片段里有多少 bigram 出现在标准名里
        nb = _bigrams(name)
        cover = len(pb & nb) / max(len(pb), 1)

        # 标准名被地点片段包含（"锦汇华庭B栋" ⊂ "锦汇华庭B栋一单元门口"）→ 强加分
        bonus = 0.25 if name and name in phrase else 0.0
        # 反之地点在标准名里（OCR 少认了几个字）
        bonus = max(bonus, 0.15 if phrase in name else 0.0)

        out.append((base * 0.5 + cover * 0.5 + bonus, item))

    out.sort(key=lambda t: t[0], reverse=True)
    return out
