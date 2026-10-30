# -*- coding: utf-8 -*-
"""港口迷雾：探一遍"到底有没有图像接口"。

YOLO 能不能用，先取决于场景给不给画面。这里把常见的取图命令名都试一遍，
只发命令、解析原始回包，不碰控制。任何一条返回带图像数据（base64 / 文件路径 / 宽高），
就说明可以接视觉；全部 ok=false 或 error，就说明这个场景只给结构化观测。

用法：python -u fog_probe_cmd.py --out ...\\logs\\probe_cmd
"""
import argparse
import json
import os
import socket
import time

HOST = "127.0.0.1"
PORT = 9100

CANDIDATES = [
    "get_image", "get_images", "get_camera", "get_camera_image", "get_frame",
    "get_video_frame", "capture_image", "capture", "screenshot", "get_screen",
    "get_render", "get_photo", "get_png", "get_picture", "demo_get_image",
    "get_observation_image", "get_sensor_data", "get_lidar", "get_rgb",
]


class C(object):
    def __init__(self):
        self.s = socket.create_connection((HOST, PORT), timeout=5)
        self.s.settimeout(8)
        self.buf = b""

    def call(self, payload):
        self.s.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        while b"\n" not in self.buf:
            chunk = self.s.recv(1 << 20)
            if not chunk:
                raise ConnectionError("closed")
            self.buf += chunk
        line, rest = self.buf.split(b"\n", 1)
        self.buf = rest
        return json.loads(line.decode("utf-8", "replace"))


def short(obj, n=260):
    try:
        s = json.dumps(obj, ensure_ascii=False)
    except Exception:                                          # noqa: BLE001
        s = str(obj)
    return s if len(s) <= n else s[:n] + "..."


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    log = open(os.path.join(args.out, "cmd.log"), "w", encoding="utf-8")

    def note(msg):
        print(msg, flush=True)
        log.write(msg + "\n")
        log.flush()

    c = None
    for _ in range(60):
        try:
            c = C()
            break
        except Exception:                                      # noqa: BLE001
            time.sleep(1.0)
    if c is None:
        note("!! 连不上 9100")
        return

    note("=== 1. 每条候选命令都试一遍（Car_C）===")
    for cmd in CANDIDATES:
        for payload in ({"command": cmd, "vehicle_id": "Car_C"},
                        {"command": cmd, "vehicle_ids": ["Car_C"]}):
            try:
                r = c.call(payload)
            except Exception as exc:                           # noqa: BLE001
                note("%-24s %s -> 异常 %s" % (cmd, "vid" if "vehicle_id" in payload else "vids",
                                              type(exc).__name__))
                continue
            ok = r.get("ok")
            keys = [k for k in r.keys() if k not in ("ok", "command")]
            note("%-24s %-5s ok=%-5s keys=%s %s"
                 % (cmd, "vid" if "vehicle_id" in payload else "vids", ok, keys, short(r)))
            if ok and any(k for k in keys if "image" in k.lower() or "frame" in k.lower()
                          or "base64" in k.lower()):
                note("    ^^^ 有戏：这条返回了图像相关字段")

    note("=== 2. 现有观测/任务接口里有没有图像字段 ===")
    for cmd, extra in (("get_observation", {"vehicle_id": "Car_C"}),
                       ("get_task_state", {}),
                       ("get_task", {}),
                       ("get_vehicles", {})):
        p = {"command": cmd}
        p.update(extra)
        try:
            r = c.call(p)
            note("%-16s ok=%s keys=%s" % (cmd, r.get("ok"), list(r.keys())))
            obs = r.get("observation") or {}
            if obs:
                note("    observation keys: %s" % list(obs.keys()))
        except Exception as exc:                               # noqa: BLE001
            note("%-16s 异常 %s" % (cmd, exc))

    note("=== 3. 车辆状态里有没有相机/传感器字段 ===")
    try:
        r = c.call({"command": "get_vehicle_state", "vehicle_id": "Car_C"})
        note("get_vehicle_state ok=%s keys=%s" % (r.get("ok"), list(r.keys())))
        vs = r.get("vehicle_state") or {}
        note("vehicle_state keys: %s" % list(vs.keys()))
        note("  %s" % short(vs, 500))
    except Exception as exc:                                   # noqa: BLE001
        note("get_vehicle_state 异常 %s" % exc)

    log.close()


if __name__ == "__main__":
    main()
