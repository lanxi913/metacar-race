# -*- coding: utf-8 -*-
"""探 9100 的 JSON 行协议：发一条 JSON，读一条回包。"""
import json
import socket
import time


def ask(payload: dict, wait: float = 2.0) -> dict | None:
    s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
    s.settimeout(wait)
    try:
        s.sendall((json.dumps(payload) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        line = buf.split(b"\n")[0].decode("utf-8", "replace")
        return json.loads(line) if line.strip().startswith("{") else {"raw": line[:300]}
    except Exception as e:
        return {"error": str(e)}
    finally:
        s.close()


for name, payload in [
    ("empty", {}),
    ("ping", {"cmd": "ping"}),
    ("state", {"action": "state"}),
    ("all", {"vehicle_id": "C"}),
]:
    r = ask(payload)
    print(f"== {name} -> {json.dumps(r, ensure_ascii=False)[:600]}")
    time.sleep(0.3)
