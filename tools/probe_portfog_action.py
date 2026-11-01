# -*- coding: utf-8 -*-
"""摸 action 的调用形状：action 字段？子命令？还是塞进 set_vehicle_control？"""
import json
import socket


def ask(payload: dict, wait: float = 3.0):
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


CASES = [
    ("action field", {"command": "action", "action": "scan", "vehicle_id": "Car_C"}),
    ("do_action", {"command": "do_action", "action": "scan", "vehicle_id": "Car_C"}),
    ("task_action", {"command": "task_action", "action": "scan", "vehicle_id": "Car_C"}),
    ("in set_vehicle_control", {"command": "set_vehicle_control", "vehicle_id": "Car_C",
                                "action": "scan", "throttle": 0.0, "brake": 1.0,
                                "steering": 0.0, "gear": 1}),
    ("act", {"command": "act", "action": "scan", "vehicle_id": "Car_C"}),
    ("perform", {"command": "perform", "action": "scan", "vehicle_id": "Car_C"}),
    ("submit action obj", {"command": "submit_action", "action": "scan", "vehicle_id": "Car_C"}),
]
for name, payload in CASES:
    r = ask(payload)
    print(f"{name:24s} ok={r.get('ok')} msg={str(r.get('message'))[:110]}")
