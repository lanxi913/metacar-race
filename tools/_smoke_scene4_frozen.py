"""离线冒烟：元素 1–6 冻结快照（防止后续改 7–12 时把前面的满分碰掉）。

跑法（仓库根）：

    .venv\\Scripts\\python.exe tools\\_smoke_scene4_frozen.py            # 比对
    .venv\\Scripts\\python.exe tools\\_smoke_scene4_frozen.py --update   # 重写快照

它做三件事：
1. `kind_of` 对 7 个冻结元素名的映射必须与 `frozen.FROZEN_KIND` 完全一致；
2. 冻结常量必须与 `docs/scene4-frozen.json` 逐项相等（含浮点 1e-9 容差）；
3. 快照文件缺失时给 `--update` 提示（首次建立快照用）。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from metacar_race.scene4 import frozen  # noqa: E402
from metacar_race.scene4.strategies import kind_of  # noqa: E402

SNAP = REPO / "docs" / "scene4-frozen.json"
FAILS: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  OK   " if ok else "  FAIL ") + name + (("  " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def main() -> int:
    update = "--update" in sys.argv[1:]
    now = frozen.snapshot()

    print("[1] 元素 1–6 的策略映射")
    for name, want in frozen.FROZEN_KIND.items():
        got = kind_of(name)
        check("%s → %r" % (name, want), got == want, "实际 %r" % (got,))

    print("[2] 冻结常量：%d 项" % len(now["constants"]))
    if update or not SNAP.exists():
        SNAP.parent.mkdir(parents=True, exist_ok=True)
        SNAP.write_text(json.dumps(now, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("  已写入快照 %s（%d 项）" % (SNAP.name, len(now["constants"])))
        if not SNAP.exists():  # 只读环境兜底
            print("内联快照：")
            print(json.dumps(now, ensure_ascii=False, indent=2))
        return 0 if not FAILS else 1

    expected = json.loads(SNAP.read_text(encoding="utf-8"))
    diffs = frozen.diff_against(expected)
    if diffs:
        for d in diffs:
            print("  FAIL " + d)
        FAILS.append("冻结快照漂移 %d 项" % len(diffs))
    else:
        check("冻结快照与当前配置一致（%d 项常量）" % len(now["constants"]), True)

    print()
    if FAILS:
        print("失败 %d 项：%s" % (len(FAILS), ", ".join(FAILS)))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
