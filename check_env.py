# -*- coding: utf-8 -*-
r"""环境自检：Python 版本、SDK 导入、仿真端口。

用法：
    .venv\Scripts\python.exe check_env.py

三项全绿才可以开始写算法；9100 端口未监听是正常的（场景没开）。
"""
import socket
import sys


def main() -> int:
    ok = True
    print("Python :", sys.version.split()[0], f"({sys.executable})")

    try:
        import metacar
        print("metacar:", getattr(metacar, "__version__", "unknown"))
    except Exception as exc:  # noqa: BLE001
        ok = False
        print("metacar 导入失败:", exc)

    try:
        import metacar_multiagent
        print("metacar_multiagent:", getattr(metacar_multiagent, "__version__", "ok"))
    except Exception as exc:  # noqa: BLE001
        ok = False
        print("metacar_multiagent 导入失败:", exc)

    for port in (9100,):
        sock = socket.socket()
        sock.settimeout(1.0)
        try:
            sock.connect(("127.0.0.1", port))
            print(f"port {port}: 可连接")
        except Exception:  # noqa: BLE001
            print(f"port {port}: 未监听（先跑脚本、再开场景）")
        finally:
            sock.close()

    print("\n结果:", "通过" if ok else "有失败项")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
