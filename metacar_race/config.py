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

# ---- 循迹（feat/control-tracking 标定中）----
TRACK_SPEED_MS = 10.0     # 直行目标速度，限速 60 km/h 下留足余量
ACCEL_LIMIT_MS2 = 2.0     # 速度指令斜坡上限，留出余量保证 |a| < 5
MAX_STEER_RAD = 0.6       # 前轮最大转角（估），用于把转角归一化到 -1~1
WHEELBASE_M = 2.6         # 轴距（估），纯跟踪用


UPSHIFT_SPEED_THRESHOLD = 4.9   # 低于这个速度说明还在一挡（17.2 km/h 附近），需要 kick 升挡

# ---- 车辆停止标定（feat/control-calibration，2026-10-23 实车数据）----
STOP_THROTTLE_COMMAND = 0.01     # 稳态约 4.78 m/s，加速度窗口 < 5
STOP_BRAKE_COMMAND = 0.10        # 从 4.78 m/s 停车约 4.22 m
STOP_BRAKE_DECEL_MS2 = 2.70      # 上述刹车指令的等效平均减速度
STOP_BRAKE_LEAD_M = 0.10         # 实测再晚一帧刹车，消除 0.125 m 位置误差
STOP_CAPTURE_RADIUS_M = 0.20     # 进入该半径且速度足够低后锁存停车
STOP_STILL_SPEED_MS = 0.30       # 静止时速度信号仍有约 ±0.23 的抖动
STOP_SETTLE_SECONDS = 0.80       # 低速持续该时间后才锁存驻车
STOP_HOLD_SECONDS = 5.0          # 题目要求停稳 5 s
STOP_EXIT_GRACE_SECONDS = 1.0    # 停稳计时完成后再保持制动一段时间
STOP_STEER_LOCK_DISTANCE_M = 2.5 # 终点附近回正，避免越过目标后继续打方向

# ---- 直角转弯标定（feat/control-calibration）----
TURN_THROTTLE_COMMAND = 0.01     # 与停车题相同，稳态约 4.78 m/s
TURN_LOOKAHEAD_M = 3.0           # 匹配题目的 3 m 曲率，避免大前视切弯

# ---- 垂直车道泊车标定（feat/control-calibration）----
PARK_LOOKAHEAD_M = 2.5           # 车位窄，前视距离小于直角转弯
PARK_APPROACH_DISTANCE_M = 8.5   # 从这个剩余距离开始重建泊车末段
PARK_ALIGN_EXTENSION_M = 3.0     # 转向引导延伸到车位中心之后，停车仍停在中心
PARK_THROTTLE_COMMAND = 0.005    # 泊车段速度更低，减少切弯和碰撞
PARK_REVERSE_THROTTLE = 0.01     # 第一次入位后倒车调整姿态
PARK_ADJUST_THROTTLE = 0.005     # 回正时低速前进
PARK_REVERSE_DISTANCE_M = 0.7    # 只做小幅倒车，避免把车横向带出车位
PARK_RECOVERY_CONTROL_M = 1.2    # 回正曲线起止切线长度
PARK_RECOVERY_LOOKAHEAD_M = 1.2  # 短距离回正时的小前视距离
PARK_PAUSE_SECONDS = 0.40        # 换挡前先停稳
PARK_ALIGN_TOLERANCE_DEG = 2.0   # 车位轴向角度容差
PARK_ENTRY_RADIUS_M = 0.75       # 进入该范围且停稳后允许倒车调姿
PARK_APPROACH_OFFSET_M = 0.0     # 回退实验无效，恢复使用真实车位中心
PARK_POSITION_TOLERANCE_M = 0.25 # 与车位中心的允许误差
PARK_FINAL_ALIGN_DEG = 3.0       # 最终车身与车位轴线的角度容差
PARK_MAX_ADJUST_CYCLES = 4       # 最多倒车/回正轮数，防止无限循环
PARK_ALIGN_TIMEOUT_SECONDS = 10.0  # 前向回正阶段的最长持续时间

# ---- 入口对齐 + 直线入位 ----
PARK_ENTRY_LOOKAHEAD_M = 3.0     # 入位前转弯段的前视距离
PARK_ENTRY_ALIGN_DEG = 4.0       # 车位入口允许的航向误差
PARK_ENTRY_SWITCH_RADIUS_M = 0.75  # 进入入口切换区的距离
