# -*- coding: utf-8 -*-
"""全局常量。改这里之前先在群里说一声，所有人都依赖它。

数值来源：平台「帮助」页规则 PDF + 实测。
"""

# ---- 连接 ----
SINGLE_SCENE_PORT = 5061      # metacar.SceneAPI 的 JSON 通道（脚本是服务端）
SINGLE_VIDEO_PORT = 5063      # metacar.SceneAPI 的视频流通道
MULTI_AGENT_PORT = 9100       # metacar_multiagent.ScenarioTaskAPI

# ---- 限速（三套规则不同，别混用）----
LIMIT_SCENE_ELEMENT_KMH = 60.0    # 场景元素 / 连续场景
LIMIT_VLA_KMH = 30.0              # VLA 通用扣分
LIMIT_MULTI_AGENT_KMH = 20.0      # 港口迷雾
LIMIT_MULTI_AGENT_TOLERANCE_KMH = 20.5   # 超 20.5 且持续 0.5 s 才扣分

# ---- 时间 ----
ELEMENT_FAIL_SECONDS = 100.0      # 元素题多数 100 s
ELEMENT_FAIL_SECONDS_LONG = 200.0 # 复杂题 200 s
VLA_TASK_FAIL_SECONDS = 100.0
MULTI_AGENT_TOTAL_SECONDS = 600.0
CONTROL_HEARTBEAT_SECONDS = 1.0   # 超过 1 s 没发控制 → 安全停车
CONTROL_PERIOD_SECONDS = 0.2      # 队内约定 5 Hz 发控制

# ---- 扣分 ----
ACCEL_BONUS_LIMIT = 5.0           # |a| ≤ 5 m/s²，每题 +10
COLLISION_PENALTY_PER_MS = 5.0    # 场景元素：碰撞扣 5×v
VLA_SPEED_PENALTY = -50
VLA_ACCEL_PENALTY = -50
VLA_COLLISION_PENALTY = -100

# ---- 港口迷雾任务阈值 ----
OBSERVE_DISTANCE_M = 30.0
OBSERVE_FOV_DEG = 120.0
READ_KEY_DISTANCE_M = 7.0
SCAN_ROADBLOCK_DISTANCE_M = 10.0
INTERSECTION_APPLY_DISTANCE_M = 12.0
INTERSECTION_MUTEX_RADIUS_M = 6.0
BERTH_DELIVER_RADIUS_M = 3.0
OBSERVATION_TOKEN_TTL_S = 30.0

# ---- 控制参数（初步值，待标定）----
TARGET_SPEED_MS = 8.0
LOOKAHEAD_M = 6.0
STEER_GAIN = 1.8
THROTTLE_GAIN = 0.12
