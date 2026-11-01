# -*- coding: utf-8 -*-
"""港口迷雾：探索剩余命令（观察 / 规划问题 / 运行报告）。"""
import json
import socket


def ask(payload: dict, wait: float = 4.0):
    s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
    s.settimeout(wait)
    try:
        s.sendall((json.dumps(payload) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        return json.loads(buf.split(b"\n")[0].decode("utf-8", "replace"))
    except Exception as e:
        return {"ok": None, "message": f"ERR {e}"}
    finally:
        s.close()


NAMES = ["observe", "get_observation", "local_observation", "get_local_observation",
         "get_planning_problem", "planning_problem", "get_run_report", "run_report",
         "mission_report", "get_score", "score"]
for n in NAMES:
    r = ask({"command": n, "vehicle_id": "Car_C"})
    msg = str(r.get("message"))[:80]
    print(f"{n:26s} ok={r.get('ok')} msg={msg}")
    if r.get("ok"):
        keys = [k for k in r.keys() if r.get(k) not in (None, "", [], {})]
        print(f"    non-empty keys: {keys}")
