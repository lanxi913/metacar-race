# -*- coding: utf-8 -*-
"""在 5061/5063 上监听，看港口迷雾场景会不会像普通场景那样连过来。"""
from __future__ import annotations

import socket
import struct
import time


def listen(port: int):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(1)
    s.settimeout(40)
    return s


s1 = listen(5061)
s3 = listen(5063)
print("listening on 5061/5063, waiting up to 40s ...", flush=True)
t0 = time.time()
try:
    c1, a1 = s1.accept()
    print(f"5061 connected from {a1} after {time.time()-t0:.1f}s", flush=True)
    c1.settimeout(5)
    try:
        head = c1.recv(4)
        if len(head) == 4:
            n = struct.unpack("!I", head)[0]
            data = b""
            while len(data) < n:
                chunk = c1.recv(n - len(data))
                if not chunk:
                    break
                data += chunk
            print("first message len:", n, "preview:", data[:300], flush=True)
        else:
            print("no header", flush=True)
    except Exception as e:
        print("recv err:", e, flush=True)
    try:
        c3, a3 = s3.accept()
        print(f"5063 connected from {a3}", flush=True)
    except Exception as e:
        print("5063 no conn:", e, flush=True)
except Exception as e:
    print("no connection on 5061:", e, flush=True)
finally:
    for x in (s1, s3):
        try:
            x.close()
        except Exception:
            pass
