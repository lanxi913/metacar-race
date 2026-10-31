# -*- coding: utf-8 -*-
"""从平台 API 读某个场景最近的跑局成绩（比截结算画面可靠）。

用法：
    .venv\\Scripts\\python.exe tools/last_run.py straight            # 最近 1 局
    .venv\\Scripts\\python.exe tools/last_run.py lianxu4 3 --det     # 最近 3 局 + 逐元素明细

token 优先取 tools/.mylab_token 文件（一行），其次环境变量 MYLAB_TOKEN。
"""
from __future__ import annotations

import json
import os
import sys
import urllib.request
from pathlib import Path

BASE = "https://api.i-mylab.com/api/"
TOKEN_FILE = Path(__file__).resolve().parent / ".mylab_token"


def token() -> str:
    if TOKEN_FILE.exists():
        t = TOKEN_FILE.read_text(encoding="utf-8").strip()
        if t:
            return t
    return os.environ.get("MYLAB_TOKEN", "")


def get(path: str, tok: str):
    req = urllib.request.Request(BASE + path)
    req.add_header("Authorization", "Token " + tok)
    req.add_header("User-Agent", "Mozilla/5.0")
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode("utf-8", "replace"))


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    slug = sys.argv[1]
    n = 1
    detail = False
    for a in sys.argv[2:]:
        if a in ("--det", "--details"):
            detail = True
        else:
            n = int(a)
    tok = token()
    if not tok:
        print("no token: put it in tools/.mylab_token or MYLAB_TOKEN")
        return 2

    d = get(f"car/scoring/run-records/?page_size={max(n, 5)}&scene={slug}", tok)
    rows = d.get("results") or []
    if not rows:
        print(f"no run records for {slug}")
        return 1
    rows.sort(key=lambda r: r["start_time"], reverse=True)
    for r in rows[:n]:
        extra = ""
        if detail:
            det = get(f"car/scoring/run-records/{r['id']}/", tok)
            parts = []
            for s in det.get("subscene_details") or []:
                reasons = ", ".join(f"{x['reason']} {x['score']}" for x in (s.get("reasons") or []))
                parts.append(f"{s.get('subscene_name')}={s.get('score')}" + (f" [{reasons}]" if reasons else ""))
            extra = "  " + " | ".join(parts)
        print(f"{r['id']:>7}  {r['start_time'][:19]}  {float(r['score'] or 0):>8.2f}{extra}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
