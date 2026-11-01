# -*- coding: utf-8 -*-
"""存一张「停车区违规检测」场景的相机画面，看看贴纸到底长什么样。"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import cv2  # noqa: E402

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

OUT = Path(r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\sticker_frames")


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    api = SceneAPI()
    api.connect()
    saved = 0
    frames = 0
    try:
        for msg, cams in api.main_loop():
            frames += 1
            if cams and frames in (30, 60, 90, 150, 240):
                for i, cam in enumerate(cams):
                    p = OUT / f"f{frames:04d}_cam{i}.png"
                    cv2.imwrite(str(p), cam.frame)
                print(f"saved frame {frames}: {len(cams)} cams", flush=True)
                saved += 1
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=0.0))
            if saved >= 5:
                break
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:
            pass
    print("saved", saved, "frames", frames, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
