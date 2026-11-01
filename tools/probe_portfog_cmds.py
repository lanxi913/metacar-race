# -*- coding: utf-8 -*-
"""猜 9100 的命令名，靠错误信息摸清协议。"""
import json
import socket


def ask(payload: dict, wait: float = 2.0):
    s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
    s.settimeout(wait)
    try:
        s.sendall((json.dumps(payload) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(8192)
            if not chunk:
                break
            buf += chunk
        line = buf.split(b"\n")[0].decode("utf-8", "replace")
        return json.loads(line) if line.strip().startswith("{") else {"raw": line[:200]}
    except Exception as e:
        return {"error": str(e)}
    finally:
        s.close()


COMMANDS = [
    "help", "state", "get_state", "status", "observe", "scan", "list",
    "vehicles", "get_vehicles", "control", "set_control", "drive", "task",
    "get_task", "report", "submit", "read_code", "signal", "confirm",
]
for c in COMMANDS:
    r = ask({"command": c})
    msg = r.get("message") or r.get("error") or r.get("raw")
    ok = r.get("ok")
    print(f"{c:14s} ok={ok} msg={str(msg)[:110]}")
