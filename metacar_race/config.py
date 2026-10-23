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

# ---- 泊车（垂直车道泊车标定中）----
PARK_APPROACH_SPEED_MS = 0.35   # 接近对位点的油门开度（低速）
PARK_REVERSE_SPEED_MS = 0.30    # 倒车油门开度
PARK_STAGING_BACK_M = 6.0       # 对位点距车位中心的前向距离
PARK_TARGET_TOL_M = 0.30        # 入库判定：中心误差
PARK_HEADING_TOL_DEG = 5.0      # 入库判定：航向误差

# ---- 环形转弯 ----
RING_TURN_LOOKAHEAD_RESPONSE_S = 0.90    # 车速对应的前视时间
RING_TURN_LOOKAHEAD_CAR_LENGTHS = 0.60   # 低速前视基准，按主车长度推导
RING_TURN_LOOKAHEAD_LANE_MIN_RATIO = 0.80
RING_TURN_LOOKAHEAD_LANE_MAX_RATIO = 3.00
RING_TURN_CURVATURE_SAMPLE_M = 3.00      # 局部曲率采样弧长
RING_TURN_LATERAL_ACCEL_MS2 = 1.60       # 弯道允许侧向加速度，取 5 m/s² 上限的 32%
RING_TURN_REACTION_TIME_S = 0.50
RING_TURN_PLAN_DECEL_MS2 = 1.50          # 弯前规划减速度，取 5 m/s² 上限的 30%
RING_TURN_TIME_BUDGET_RATIO = 0.80      # 计划在平台时限的 80% 内完成，保留建模误差余量
RING_TURN_SPEED_MARGIN_RATIO = 1.30      # 相对计划剩余时间所需均速的安全余量
RING_TURN_MAX_SPEED_RATIO = 0.40         # 巡航上限占平台 60 km/h 限速的比例
RING_TURN_MIN_SPEED_MS = 2.20
RING_TURN_SHIFT_URGENCY_RATIO = 0.75     # ETA 超过计划剩余时间的比例时允许升挡
RING_TURN_TARGET_SPEED_SLEW_MS2 = 1.20   # 目标速度变化对应的规划加速度
RING_TURN_THROTTLE_SLEW_PER_SECOND = 0.045  # 0.0015/帧，沿用主线已验证的加速度安全斜率
RING_TURN_THROTTLE_RELEASE_PER_SECOND = 0.12  # 0.004/帧，减速先平滑松油
RING_TURN_BRAKE_SLEW_PER_SECOND = 0.025       # 刹车继续使用更保守的斜率
RING_TURN_BRAKE_RELEASE_PER_SECOND = 0.06
RING_TURN_CRUISE_THROTTLE_MAX = 0.12
RING_TURN_BRAKE_START_ERROR_MS = 0.70
RING_TURN_SHIFT_THROTTLE_RAMP_PER_S = 1.50
RING_TURN_SHIFT_THROTTLE_RELEASE_PER_S = 2.50
RING_TURN_SHIFT_MAX_SECONDS = 0.55
RING_TURN_SHIFT_STABLE_SECONDS = 0.45
RING_TURN_SHIFT_COOLDOWN_SECONDS = 1.50
RING_TURN_SHIFT_EXIT_SPEED_MS = 5.05
RING_TURN_POST_SHIFT_SETTLE_SECONDS = 0.80
RING_TURN_SHIFT_CURVATURE_MARGIN_MS = 0.30
RING_TURN_START_THROTTLE_MAX = 0.004     # 静止起步的低挡位安全油门，来自 U 型弯标定
RING_TURN_START_RAMP_SECONDS = 3.00      # 从零建立起步油门的时长
RING_TURN_START_EXIT_SPEED_MS = 3.00     # 低挡位速度建立后交回巡航控制器
RING_TURN_CROSS_TRACK_BAND_RATIO = 0.25  # 不降速的横向误差占车道宽度比例
RING_TURN_CROSS_TRACK_MIN_SPEED_RATIO = 0.70
# ---- 平行车道泊车（feat/parking-parallel，2026-10-23 连续 5 次 210.00）----
PARALLEL_SLOT_TYPE = 30          # 障碍物里车位对象的 type 值
PARALLEL_CAR_LEN = 4.00          # 车长（车辆属性）
PARALLEL_WHEELBASE = 2.60        # 轴距（车辆属性）
PARALLEL_MAX_STEER_RAD = 0.60    # 最大前轮转角（车辆属性）
PARALLEL_THROTTLE_MAX = 0.010    # 油门最小有效值
PARALLEL_STEER = 0.75            # 倒车舵角；满舵 1.0 会顶住静摩擦导致车不动
PARALLEL_A_MAX = 0.5             # 规划加速度上限（m/s²）
PARALLEL_V_APPROACH = 3.2        # 接近段目标速度
PARALLEL_V_REVERSE = 0.9         # 倒车目标速度
PARALLEL_BRAKE_CMD = 0.10        # 常规刹车指令
PARALLEL_SETTLE_BRAKE = 0.06     # 收尾轻刹（纯滑行会多滚 1 m，把 dy 带偏）
PARALLEL_STAGING_MARGIN_M = 0.15 # staging 相对远端车中心的余量
PARALLEL_TOL_DX_M = 0.35         # 入位判据：横向
PARALLEL_TOL_DY_M = 0.35         # 入位判据：纵向
PARALLEL_HOLD_SECONDS = 5.0      # 停稳保持
# ---- 车道级循迹 + 换道避让（feat/scenes-page2-zyy，左侧车道超车实测 110）----
LANE_CRUISE_MS = 4.2       # 巡航目标速度
LANE_ACCEL_MAX = 1.2       # 规划加速度上限（m/s²）
LANE_HOLD_SECONDS = 5.0    # 终点停稳保持
LANE_LAUNCH_HOLD_S = 1.5   # 起步前先驻车的时间，压掉挂挡首帧跳变
LANE_STEER_LIMIT = 0.8     # 转向限幅，满舵低速会卡死
LANE_WHEELBASE = 2.6       # 轴距（车辆属性）

# ---- 路线级循迹（辅路驶入驶出实测 110，2026-10-23）----
ROUTE_CRUISE_MS = 6.0      # 巡航目标速度（车实际最高 4.8 m/s，长路线按此跑）
ROUTE_ACCEL_MAX = 2.0      # 规划加速度上限
ROUTE_HOLD_SECONDS = 5.0   # 终点停稳保持
ROUTE_STEER_LIMIT = 0.8    # 转向限幅
ROUTE_WHEELBASE = 2.6      # 轴距
# ---- 车辆跟随（首版直线跟车）----
FOLLOW_CANDIDATE_MAX_LATERAL_M = 2.0   # 只选择当前车道前方的车辆
FOLLOW_CANDIDATE_MAX_HEADING_DEG = 75.0
FOLLOW_STANDSTILL_GAP_M = 10.0         # 前车停止后的目标净车距
FOLLOW_HEADWAY_SECONDS = 1.2           # 随前车速度增加的时间车头距
FOLLOW_GAP_GAIN = 0.40                 # 车距误差换算为速度修正
FOLLOW_MAX_SPEED_MS = 5.8
FOLLOW_LEADER_LOST_SPEED_MS = 1.5
FOLLOW_MIN_GAP_M = 4.5                 # 硬安全净距
FOLLOW_SAFE_DECEL_MS2 = 2.5
FOLLOW_END_DECEL_MS2 = 2.0
FOLLOW_END_TOL_M = 0.60
FOLLOW_SPEED_DEADBAND_MS = 0.25
FOLLOW_THROTTLE_MAX = 0.50
FOLLOW_THROTTLE_SLEW_PER_FRAME = 0.010
FOLLOW_BRAKE_SLEW_PER_FRAME = 0.005
FOLLOW_HOLD_SECONDS = 3.0
FOLLOW_LOOKAHEAD_M = 6.0
