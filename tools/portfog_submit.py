# -*- coding: utf-8 -*-
"""读取 hint 里的现场验证码并 submit_key，然后看下一个信号。"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar_multiagent import ScenarioTaskAPI  # noqa: E402

KEY_RE = re.compile(r"KEY[-_ ]?[A-Z0-9]{1,8}", re.IGNORECASE)


def find_key(text: str) -> str:
    m = KEY_RE.search(text or "")
    return m.group(0).replace(" ", "") if m else ""


def main() -> int:
    with ScenarioTaskAPI() as api:
        vid = "Car_C"
        # 两个巡检车都看一眼谁在信号旁
        for cand in ("Car_D", "Car_C"):
            obs = api.get_observation(cand)
            pos = obs.position
            for ob in obs.visible_objects:
                key = find_key(ob.hint)
                print(f"{cand} @({pos.x:.1f},{pos.y:.1f}) 看见 {ob.object_id} "
                      f"d={ob.distance_m:.1f} hint={ob.hint!r} key={key!r}", flush=True)
                if ob.object_type == "signal" and key and ob.distance_m <= 7.5:
                    r = api.interact(cand, ob.object_id, "submit_key", {"key": key})
                    print(f"  -> submit_key({key}) granted={r.granted} msg={r.message}", flush=True)
                    st = api.get_task_state()
                    print(f"  信号进度 {st.verified_signal_count}/{st.total_signal_count}", flush=True)
                    nxt = next((o for o in st.shared_task_objects
                                if o.object_type == "signal" and o.state != "verified"), None)
                    if nxt:
                        print(f"  下一个信号: {nxt.object_id} @({nxt.position.x:.1f},"
                              f"{nxt.position.y:.1f}) 精度={nxt.information_level}", flush=True)
                    return 0
        print("没找到可提交的信号", flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
