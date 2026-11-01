# -*- coding: utf-8 -*-
"""继续摸命令名：找控制/心跳类的命令，并看 action 命令需要什么参数。"""
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


NAMES = ["vehicle_control", "heartbeat", "set_vehicle_control", "control_vehicle",
         "drive_vehicle", "step", "tick", "sync", "update", "command",
         "set_control", "apply_control", "vehicle_command"]
for n in NAMES:
    r = ask({"command": n, "vehicle_id": "Car_C", "throttle": 0.0, "brake": 0.0,
             "steering": 0.0, "gear": 1})
    print(f"{n:22s} ok={r.get('ok')} msg={str(r.get('message'))[:100]}")

print("\n=== action 命令的参数探查 ===")
for cmd in ("scan", "submit_key", "report_observation", "request_entry", "release", "deliver"):
    r = ask({"command": cmd, "vehicle_id": "Car_C"})
    print(f"{cmd:20s} ok={r.get('ok')} msg={str(r.get('message'))[:120]}")
