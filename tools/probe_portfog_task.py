# -*- coding: utf-8 -*-
"""拉取港口迷雾的公开任务定义与车辆清单。"""
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
        return json.loads(line) if line.strip().startswith("{") else {"raw": line[:500]}
    finally:
        s.close()


for cmd in ("get_task", "get_vehicles"):
    r = ask({"command": cmd})
    print("=" * 20, cmd)
    print(json.dumps(r, ensure_ascii=False, indent=1)[:4000])
