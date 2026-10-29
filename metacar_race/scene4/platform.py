"""场景 IO 与产物落盘。

* `SceneSession`：把 SDK 那套「先跑脚本 → 阻塞 connect → 再点启动场景 → main_loop
  yield (msg, frames)」包起来，driver 只跟它打交道。
* `SceneLog`：`run.csv` / `obs.jsonl` / `events.jsonl` / `summary.json` 四件产物，
  和旧 `park_cs4.py` 的目录布局与列名保持一致（历史日志还能直接对照）。
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from metacar import GearMode, SceneAPI, VehicleControl

REPO = Path(__file__).resolve().parents[2]

RUN_HEADER = [
    "frame", "sim_t", "sub_scene", "elem_i", "phase", "x", "y", "yaw", "speed",
    "gear", "throttle", "brake", "steering", "v_tgt", "route_s", "lat_err",
    "blk_dist", "blk_type", "blk_id", "obs_n", "stuck", "lat_off",
    "light", "stop_gap", "ov_state",
]


class SceneSession:
    """一次仿真会话。"""

    def __init__(self) -> None:
        self.api = SceneAPI()
        self.static = None

    def connect(self) -> object:
        """阻塞等场景连上来，返回静态数据（路线 / 道路 / 子场景）。"""
        self.api.connect()
        self.static = self.api.get_scene_static_data()
        return self.static

    def frames(self):
        """逐帧 yield (msg, camera_frames)。场景结束时生成器自然结束。"""
        yield from self.api.main_loop()

    def skip_level(self) -> None:
        try:
            self.api.skip_level()
        except Exception:  # noqa: BLE001
            pass

    def set_control(self, vc: VehicleControl) -> None:
        self.api.set_vehicle_control(vc)

    def stop(self, throttle: float = 0.0, brake: float = 0.0,
             steering: float = 0.0, gear: GearMode = GearMode.DRIVE) -> None:
        """发一帧控制量（异常时吞掉：场景可能已经关了）。"""
        try:
            self.set_control(VehicleControl(gear=gear, throttle=throttle,
                                            brake=brake, steering=steering))
        except Exception:  # noqa: BLE001
            pass

    def park(self) -> None:
        self.stop(brake=1.0, gear=GearMode.PARKING)


class SceneLog:
    """四件产物 + 统一的 emit 入口（`sim_t`/`frame` 由 driver 每帧回填）。"""

    def __init__(self, out_dir: Path) -> None:
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.run_fh = (self.out_dir / "run.csv").open("w", newline="", encoding="utf-8-sig")
        import csv as _csv
        self.run_w = _csv.writer(self.run_fh)
        self.run_w.writerow(RUN_HEADER)
        self.obs_fh = (self.out_dir / "obs.jsonl").open("w", encoding="utf-8")
        self.ev_fh = (self.out_dir / "events.jsonl").open("w", encoding="utf-8")
        self.sim_t = 0.0
        self.frame = 0
        self._closed = False

    # ---------------------------------------------------------------- 事件
    def event(self, kind: str, **kw) -> None:
        # 收尾顺序里 summary() 可能在 close() 之后被调用（老实现真踩过：ValueError:
        # I/O operation on closed file），这里统一挡住，别让日志把正常收尾搞崩。
        if self._closed or self.ev_fh.closed:
            return
        rec = {"sim_t": round(self.sim_t, 3), "frame": self.frame, "event": kind}
        rec.update(kw)
        self.ev_fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self.ev_fh.flush()

    def obstacle(self, record: dict) -> None:
        self.obs_fh.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.obs_fh.flush()

    # ---------------------------------------------------------------- 遥测
    def run_row(self, row: list) -> None:
        self.run_w.writerow(row)

    def flush_run(self) -> None:
        self.run_fh.flush()

    # ---------------------------------------------------------------- 收尾
    def summary(self, ending: str, phase: str, route_s: float,
                final: tuple[float, float, float]) -> dict:
        data = {
            "ending": ending,
            "phase": phase,
            "frames": self.frame,
            "route_s": round(float(route_s), 1),
            "final": [round(final[0], 3), round(final[1], 3), round(final[2], 2)],
            "dir": str(self.out_dir),
        }
        (self.out_dir / "summary.json").write_text(
            json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        self.event("end", ending=ending, phase=phase)
        return data

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for fh in (self.run_fh, self.obs_fh, self.ev_fh):
            try:
                fh.close()
            except Exception:  # noqa: BLE001
                pass


def save_shot(camera_frames, out_dir: Path, tag: str) -> None:
    """存一张前视相机图（cv2 直接吃 ndarray）；没装 cv2 就静默跳过。"""
    try:
        import cv2  # noqa: PLC0415
    except Exception:  # noqa: BLE001
        return
    for cam in camera_frames or []:
        img = getattr(cam, "frame", None)
        if img is None:
            continue
        try:
            path = Path(out_dir) / "shots" / ("%s_%s.jpg" % (tag, getattr(cam, "id", "cam")))
            path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(path), img)
        except Exception:  # noqa: BLE001
            pass


def default_out_dir(tag: str = "run") -> Path:
    return REPO / "logs" / "scene4" / (time.strftime("%Y%m%d-%H%M%S") + "-" + tag)
