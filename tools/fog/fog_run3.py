# -*- coding: utf-8 -*-
"""港口迷雾：执行层公共件（位姿估计 / 限幅 / 车辆几何）。

`fog_mission.py` 只从这里取五个符号：:

    pose_of(api, vid) -> ((x, y), yaw_deg, speed_mps)
    clip(v, lo, hi)
    KEY_RE                        现场铭牌验证码的正则
    WHEELBASE_RATIO、MAX_STEER     纯跟踪用到的车辆几何

## 为什么航向要用位移估，不读 SDK 的 orientation

2026-10-29 的实测（三次重复）：直线行驶时 SDK 给的角度几乎不动，而相邻两帧位姿的
位移角是稳定的真值；倒车时欧拉角还会反向。所以这里统一用**位移方向**估计航向：

* 从锚点走够 1.0 m 采一个样本，按 0.55 做角度 EMA（每次都 wrap 到 ±180，否则会累到 800°+）；
* 两帧之间位移 > 8 m 视为场景重置/瞬移，样本全部作废，重新等 1 m；
* 还没走够 1 m 时用 `orientation.y + 180°` 兜底（10-29 00:10 的标定：
  车身航向 = rotation_y + 180°）。这个兜底只在开头不到 1 m 内有效，
  第一段位移样本一到就被真值取代，所以即使它猜反了也不会把车带偏。
"""
from __future__ import annotations

import math
import re

# 现场铭牌：`现场铭牌 Key：KEY-A7`
KEY_RE = re.compile(r"KEY[-_ ]?[A-Z0-9]{1,8}", re.IGNORECASE)

# 轴距占车长比例（Car_C 长 4.81 m → 轴距约 2.65 m）
WHEELBASE_RATIO = 0.55

# 满舵对应的等效前轮转角（弧度）。纯跟踪把 atan2(2L·sinα, ld) 归一化到 -1~1 用它当量程。
MAX_STEER = 0.62

SAMPLE_M = 1.0     # 位移采样步长
JUMP_M = 8.0       # 瞬移判定
EMA = 0.55

_POSE = {}


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def reset_pose(vid=None):
    """清空航向样本（倒车、场景重置、换车时调用）。"""
    if vid is None:
        _POSE.clear()
    else:
        _POSE.pop(vid, None)


def heading_ready(vid):
    """该车是否已经有位移航向（False 表示还在开头那不到 1 m 的兜底期）。"""
    st = _POSE.get(vid)
    return bool(st and st["ready"])


def _seed_yaw(pose):
    """兜底航向。

    2026-10-29 22:44 从 get_vehicle_state 原始回包标定：车头朝西时
    `orientation.z = 179.9998`，与位移法算出的 180° 一致 —— 这个场景的 `ori_z`
    **就是车头航向**，不要再加 180°。拿不到 ori_z 时才退到旧的 `ori_y + 180°`。
    """
    try:
        return wrap180(float(getattr(pose, "ori_z")))
    except Exception:                                  # noqa: BLE001
        pass
    try:
        return wrap180(float(getattr(pose, "ori_y")) + 180.0)
    except Exception:                                  # noqa: BLE001
        return 0.0


def pose_of(api, vid):
    """返回 ((x, y), 航向deg, 速度m/s)。每辆车每帧调用一次，内部维护航向样本。"""
    st = api.get_vehicle_state(vid)
    pose = st.pose
    x, y = float(pose.pos_x), float(pose.pos_y)
    speed = float(getattr(st, "speed_mps", 0.0) or 0.0)

    s = _POSE.get(vid)
    if s is None:
        _POSE[vid] = {"x": x, "y": y, "yaw": _seed_yaw(pose), "ready": False}
        return (x, y), _POSE[vid]["yaw"], speed

    d = math.hypot(x - s["x"], y - s["y"])
    if d > JUMP_M:
        s.update(x=x, y=y, yaw=_seed_yaw(pose), ready=False)
    elif d >= SAMPLE_M:
        h = math.degrees(math.atan2(y - s["y"], x - s["x"]))
        if not s["ready"]:
            s["yaw"] = h
            s["ready"] = True
        else:
            s["yaw"] = wrap180(s["yaw"] + EMA * wrap180(h - s["yaw"]))
        s["x"], s["y"] = x, y
    return (x, y), s["yaw"], speed
