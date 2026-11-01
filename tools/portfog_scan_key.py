# -*- coding: utf-8 -*-
"""在信号附近扫描视野（倒车小弧旋转），一旦看见带 Key 的 hint 就提交。"""
from __future__ import annotations

import json
import re
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar_multiagent import ScenarioTaskAPI  # noqa: E402
import socket  # noqa: E402

KEY_RE = re.compile(r"KEY[-_ ]?[A-Z0-9]{1,8}", re.IGNORECASE)


def raw_ctrl(sock, buf, vid, thr=0.0, brk=0.0, st=0.0, gear=2):
    sock.sendall((json.dumps({"command": "set_vehicle_control", "vehicle_id": vid,
                              "vehicle_control": {"gear": gear, "throttle": thr,
                                                  "brake": brk, "steering": st}}) + "\n").encode())
    while b"\n" not in buf[0]:
        c = sock.recv(1 << 20)
        if not c:
            raise ConnectionError
        buf[0] += c
    line, rest = buf[0].split(b"\n", 1)
    buf[0] = rest
    return json.loads(line.decode("utf-8", "replace"))


def scan_and_submit(vid_hint=("Car_D", "Car_C")):
    sock = socket.create_connection(("127.0.0.1", 9100), timeout=3)
    sock.settimeout(10)
    buf = [b""]
    try:
        for rnd in range(14):
            with ScenarioTaskAPI() as api:
                for cand in vid_hint:
                    obs = api.get_observation(cand)
                    for ob in obs.visible_objects:
                        m = KEY_RE.search(ob.hint or "")
                        if ob.object_type == "signal" and m and ob.distance_m <= 7.5:
                            key = m.group(0).replace(" ", "")
                            print(f"[轮{rnd}] {cand} 看见 {ob.object_id} d={ob.distance_m:.1f} "
                                  f"key={key}", flush=True)
                            r = api.interact(cand, ob.object_id, "submit_key", {"key": key})
                            print(f"  submit granted={r.granted} msg={r.message}", flush=True)
                            st = api.get_task_state()
                            print(f"  信号 {st.verified_signal_count}/{st.total_signal_count}",
                                  flush=True)
                            nxt = next((o for o in st.shared_task_objects
                                        if o.object_type == "signal"
                                        and o.state != "verified"), None)
                            if nxt:
                                print(f"  下一个: {nxt.object_id} @({nxt.position.x:.1f},"
                                      f"{nxt.position.y:.1f})", flush=True)
                            return 0 if r.granted else 1
                vis = {c: [b.object_id for b in api.get_observation(c).visible_objects]
                       for c in vid_hint}
            print(f"[轮{rnd}] 可见={vis}，倒车小弧转向", flush=True)
            # 倒车小弧（左右交替），只求转视野
            st_dir = 1.0 if rnd % 2 == 0 else -1.0
            t0 = time.time()
            while time.time() - t0 < 0.7:
                raw_ctrl(sock, buf, vid_hint[0], thr=0.40, st=st_dir, gear=4)
                time.sleep(0.07)
            raw_ctrl(sock, buf, vid_hint[0], brk=1.0, gear=3)
            time.sleep(0.25)
        print("扫了 14 轮都没读到 Key", flush=True)
    finally:
        sock.close()
    return 2


if __name__ == "__main__":
    raise SystemExit(scan_and_submit())
