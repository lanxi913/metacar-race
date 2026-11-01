# -*- coding: utf-8 -*-
"""离线验证：把实测到过的几种 OCR 原文，清洗成尽量贴近标准答案的字符串。"""
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
def normalize(s: str) -> str:
    s = (s or "").strip()
    for ch in ("\r", "\n", "\t", "\u3000", " "):
        s = s.replace(ch, "")
    return s

LEAD_JUNK = re.compile(r"^[0-9][^\u4e00-\u9fff0-9]{1,3}[0-9]{0,3}")   # 实测的 "0生00" 这类开头乱码
BAD_HEAD = re.compile(r"^[0-9A-Za-z]{0,6}[^\u4e00-\u9fff0-9]{1,3}(?=(?:最|在|限|到|\d))")


def clean(s: str) -> str:
    t = normalize(s)
    # 1) 截到 km/h（牌子最后一行常带装饰符，OCR 会吐出 "…30km/h0生000"）
    m = re.search(r"30\s*km\s*/\s*h", t, re.I)
    if m:
        t = t[: m.end()]
        t = re.sub(r"30\s*km\s*/\s*h", "30km/h", t, flags=re.I)
    # 2) 去掉开头乱码：实测有 "0生00最迟在100秒内…"，用已知句子开头做锚点
    m0 = re.search(r"(最迟在|限时|请在|在\d)", t)
    if m0 is not None and m0.start() <= 8:
        t = t[m0.start():]
    # 3) 常见错字
    for bad, good in (("停车停车", "停车位停车"), ("肝达", "到达"), ("昼全", "并且全"),
                      ("锦汇华亭", "锦汇华庭"), ("锦汇华廷", "锦汇华庭")):
        t = t.replace(bad, good)
    # 4) 标点统一成中文（标准答案用中文标点）
    t = t.replace(",", "，").replace(":", "：").replace(";", "；")
    return t


CASES = [
    "1 00 秒 内 找 到 空 的 停 车\n停 车 ， 并 且 全 程 不 得 超 过\n30km/h\n0 生 0 0 0",
    "0生00最迟在100秒内肝达锦汇华庭G单元门口停车,途中最高车速须小于等于30km/h0",
    "在 1 00 秒 内 到 达 锦 汇 华 庭\nB 栋 一 单 元 门 口 停 车 ， 昼\n全 程 不 得 超 过 30km / h",
    "在100秒内到达锦汇华庭B栋一单元门口停车，并且全程不得超过30km/h",
]

for c in CASES:
    out = clean(c)
    print(f"IN : {c!r}")
    print(f"OUT: {out!r}  len={len(out)}")
    print()
