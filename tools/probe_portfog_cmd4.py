# -*- coding: utf-8 -*-
"""试 set_target / get_task_state / get_events / interact，找启动任务与得分的入口。"""
import json
import socket
import time


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
        line = buf.split(b"\n")[0].decode("utf-8", "replace")
        return json.loads(line)
    except Exception as e:
        return {"ok": None, "message": f"ERR {e}"}
    finally:
        s.close()


print("== get_task_state ==")
r = ask({"command": "get_task_state"})
print(" ok:", r.get("ok"), "msg:", r.get("message"), "phase:", r.get("phase"))
print(" task_state:", json.dumps(r.get("task_state"), ensure_ascii=False)[:300])

print("\n== get_events ==")
r = ask({"command": "get_events"})
print(" ok:", r.get("ok"), "msg:", r.get("message"))
print(" events:", json.dumps(r.get("events"), ensure_ascii=False)[:400])

print("\n== set_target 试验 ==")
for tgt in ["Signal_A", "Berth_A", "Car_A"]:
    r = ask({"command": "set_target", "vehicle_id": "Car_C", "target_id": tgt})
    print(f" target={tgt:10s} ok={r.get('ok')} msg={str(r.get('message'))[:80]}")

print("\n== interact 试验 ==")
for act in ["scan", "submit_key", "report_observation", "request_entry", "release", "deliver"]:
    r = ask({"command": "interact", "action": act, "vehicle_id": "Car_C"})
    print(f" {act:20s} ok={r.get('ok')} msg={str(r.get('message'))[:100]}")
