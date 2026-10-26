"""连续场景 4（lianxu4）驱动的集中参数。

这里是**唯一**允许出现魔数的地方；其余模块只引用这些名字。参数分三类，来源都写在
注释里，改参数前请先读 `docs/SCENE4-ELEMENT-ROUTER.md` 的验证记录，别把已实测的
结论当默认值随手调掉。

层级（见 `metacar_race/scene4/__init__.py`）：

    config       本文件
    track        路线几何（累计里程 / 投影 / 停止线 / 信号灯几何 / 车道宽）
    obstacles    障碍投影、起步破挡、卡死检测
    lateral      转向（PurePursuit 复用 control.tracking；这里放 Stanley / 并线）
    longitudinal 纵向（加速度治理、指令限幅、油门上升限幅）
    safety       信号灯闸门、终点驻车
    platform     场景 IO、日志、瞬移检测
    strategies   元素策略：只声明自己接管哪条通道
    router       里程选元素 + 名称路由 + 瞬移复位 + 降级
    driver       主循环装配
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

# ---------------------------------------------------------------- 场景元素常量
PARKING_SLOT_TYPE = 30          #: 停车位（ObstacleType.PARKING_SLOT）
SPEED_SIGN_TYPE = 26            #: 限速标志（ObstacleType.SPEED_LIMIT_SIGN）
IGNORE_TYPES = {PARKING_SLOT_TYPE, SPEED_SIGN_TYPE}   #: 不参与碰撞/跟车的类型
LANE_HALF_M = 1.90              #: 本车道半宽：横向偏差超过它就认为不在本道

# ---------------------------------------------------------------- 障碍/跟车
BLOCK_HORIZON_M = 45.0          #: 障碍投影的纵向搜索范围
BLOCK_MARGIN_M = 4.80           #: 跟车时预留的车头余量
BLOCK_SLOW_M = 30.0             #: 进入这个距离开始按 v=FOLLOW_DECEL*(d-1.5) 收速
BLOCK_STOP_M = 7.0              #: 进到这个距离且还有速度，直接硬刹（安全优先）
FOLLOW_DECEL = 0.45             #: 跟车收速斜率（m/s per m）

# ---------------------------------------------------------------- 超车/并线
OVERTAKE_OFFSET_M = 3.50        #: 左并的横向偏移
OVERTAKE_PREP_M = 50.0          #: 前车静止且在这个距离内才考虑并线
PASS_CLEAR_M = 60.0             #: 左道正前方要空出这么远
RETURN_CLEAR_M = 45.0           #: 回中间时本道要空出这么远
RETURN_BEHIND_M = 8.0           #: 且最后被超的车要在身后这么远
ABORT_GAP_M = 12.0              #: 目标道净空小于它就退回
SHIFT_LENGTH_M = 20.0           #: 横移按走过的纵向距离推进（余弦 S 曲线）
SHIFT_SECONDS = 2.5             #: 横移的时间下限（低速时按距离算还会更长）
SHIFT_HOLD_MIN_MS = 4.00        #: 高于这个车速横移时锁速（横向/纵向不同时做）
SHIFT_TIMEOUT_S = 30.0          #: 横移兜底超时
PASS_MAX_S = 30.0               #: 在左侧待太久就找机会回去
OVERTAKE_BAND_M = 2.20          #: 判定"同一条车道"的横向带宽

# --- 元素 6「复杂路况超车」专用（2026-10-25 用户口径：左并一次 → 超两车 → 回中线贴终点）---
# 平台推进元素靠 end_point 约 2 m 内的近距离检查点（elem6-r1 实测 0.48~1.95 m 触发、
# cont-r2 横向 2.75 m 时穿过终点**没有**触发，元素一直张着 → 该行 0 分、后面也不结算），
# 所以末段必须硬锁中线。
OVERTAKE_LANE_OFF_M = 3.50      #: 左并一次的横向偏移（约一个车道宽）
OVERTAKE_RAMP_M = 20.0          #: 并出去的横移里程（余弦，a_lat≈3.5 @9 m/s）
OVERTAKE_PASS_BEHIND_M = 4.0    #: 被超车辆落到车后 4 m 才算"超过"
OVERTAKE_TARGET_PASSES = 2      #: 要确认超过的车辆数
OVERTAKE_COUNT_LAT_M = 1.20     #: 只数"本道"被越过的车（|lat| 在这个带宽内）
OVERTAKE_RETURN_M = 32.0        #: 剩这么多里程开始回中线（PR#35 200 分局的回中终点在距终点 28.5 m）
OVERTAKE_RETURN_RAMP_M = 24.0   #: 回中的横移里程（16 m 时 @9 m/s 的 a_lat=5.46 会超标，24 m 只 3.50）
OVERTAKE_RETURN_CLEAR_M = 45.0  #: 回中前要求原车道前方这么多里程内没车（否则保持左侧继续走）
OVERTAKE_CHECKPOINT_M = 12.0    #: 剩这么多里程硬锁中线（终点检查点半径约 2 m）
OVERTAKE_LOOKAHEAD_M = 6.0      #: 纯跟踪前视距离
LANE_MAX_M = 60.0               #: 车道净空的封顶值
LANE_MOVING_MS = 1.00           #: 前车高于这个速度算"在动"，不并线
FOLLOW_BAND_SWITCH = 0.30       #: 横移超过这个量，跟车带心改按目标车道算

# ---------------------------------------------------------------- 转向
K_HEADING = 2.0                 #: Stanley 的航向项增益
K_CROSS = 2.2                   #: Stanley 的横向项增益
STANLEY_TAIL_S = 3.0            #: 离开环岛元素后再用一会儿 Stanley 的时间

# ---------------------------------------------------------------- 信号灯/停止线
LIGHT_LOOK_M = 60.0             #: 找信号灯的最远距离
LIGHT_STOP_DIST_M = 30.0        #: 红灯开始收速的距离（会再按 ETA 放大）
LIGHT_STOP_GAP_M = 2.60         #: 停止线前这么多米就算停住
LIGHT_STOP_TOL_M = 0.30         #: 停住的容差
LIGHT_DECEL = 1.20              #: 停车减速（m/s²）
LIGHT_ETA_MARGIN_S = 0.50       #: 绿灯过线的安全余量：到线时间 + margin > 剩余时间就不冲
LIGHT_REFRESH_EVERY = 0.0       #: 0 = 每帧刷新（老代码只取一次，闯过一次红灯）
# 多路口（2026-10-25 r14 复盘）：元素 5「十字路口通行」一公里内有 5 条停止线
# （s≈1923.4 / 1935.4 / 1943.4 / 1951.4 / 1963.4），老实现只看最近一条。
LIGHT_LINE_DEDUP_M = 4.00       #: 两条停止线里程差 < 这个值就当同一条
LIGHT_MATCH_M = 12.00           #: 灯与停止线的投影里程差 ≤ 这个值才算同路口
# 2026-10-25（用户 m04149 定的方案）：**不做任何「车速够不够过线」的判断**，
# 只认一个状态——灯从「非绿」跳到「绿」的那一帧立刻放行起步。
# 依据：r19 用 ETA 判「整组 5 条线 43 m 要 43.5 s 绿灯」，永远等不到放行，
# 车停在第一条线前 2 m 直到场景结束（curve-r19，s=1921.4）。
LIGHT_DECIDE_M = 130.0          #: 开始「看灯 → 松油滑行」的距离；9 m/s 用 0.35 滑行需 ≈116 m
LIGHT_COAST_DECEL_MS2 = 0.35    #: 计划滑行减速度（只用来算允许车速，实际减速靠松油）
# 2026-10-25（m04276 用户裁定）：把 30 m 内的多条停止线聚成**一个路口组**，
# 组内**第一次看到绿**就承诺通过，组内后面几条线一律不再减速——r20 就是在路口
# 中间被第 2、第 3 条线的红灯连续拦停（三次急刹，483 帧 |a|≥5，峰值 18.6）。
LIGHT_GROUP_CHAIN_M = 30.0      #: 相邻停止线间距 ≤ 这个值 → 算同一个路口组
# 接近路口时先把巡航压下来（用户 m04276：「把巡航速度往下压，不合适下次再改数字」）。
# 目的：给「红灯突然跳」留出滑行余量，避免 r20 那种 10 m 处才发现红、−11 m/s² 急刹。
LIGHT_APPROACH_MS = 5.00        #: 进 `LIGHT_DECIDE_M` 后、过线前的巡航上限（m04362：先试 5 m/s）
# 「承诺通过」的距离（m04276 方案 A 的关键补丁）：只有在组首停止线前
# `LIGHT_COMMIT_M` 以内看到绿灯，才把整个路口组锁成「一次通过」；
# 远远看到绿灯不锁——否则 130 m 处见到绿灯就承诺，半路变红也会硬闯（−39.66）。
LIGHT_COMMIT_M = 25.0           #: 距组首停止线 ≤ 此值且绿灯 → 锁定整组通过
# 停车终点：按「斑马线」放。平台静态数据里读不到斑马线（`LineType.ZEBRA_CROSSING=5`
# 在本场景一次都没出现：左边线只有 {6,1,2,3}、右边线 {2,6,3}；障碍流只有
# CAR(6)/STATIC(7)/PARKING_SLOT(30)，没有斑马线/路面标线对象），所以拿停止线
# 当锚点，用一个可标定常量表达「停在线的哪一侧」：
#   0.80 ≈ 车头压线（默认，m04149 说「刹车位置不对」，r19 停在线前 2.9 m 太靠后）
#   想再往前顶就改小/改负数（负数=过线），改完在仿真里看一眼车头与斑马线的相对位置。
LIGHT_STOP_AT_M = 0.80          #: 停车终点距停止线的米数（正=线前，负=线后）
LIGHT_WAIT_BRAKE_M = 12.0       #: 距线 ≤ 这个值且还没停住 → 直接给刹车
LIGHT_WAIT_BRAKE = 0.35         #: 停在线前用的刹车量
LIGHT_WAIT_MAX_S = 60.0         #: 等灯兜底：超过这个时长还没见绿灯就按绿灯走（防死等）
# 2026-10-25 m04362：r21 在 12 m 处按固定 0.35 刹车，1 s 内 3.24→0.47，停在
# 线前 10 m，平台判「红灯时停车位置不正确」−20。改成按剩余距离反算需要的减速度、
# 只补「滑行给不了」的那部分；如果还是停早了，就小油门蠕动补位到停车点。
LIGHT_CREEP_MS = 0.30           #: 蠕动：车速低于此值才补油
LIGHT_CREEP_M = 1.60            #: 蠕动：离停车点还有这么远才补油
LIGHT_CREEP_THROTTLE = 0.05     #: 蠕动油门（≈0.6 m/s²，离 5 m/s² 很远）

# ---------------------------------------------------------------- 起步破挡
STALL_SPEED_MS = 0.25           #: 低于这个速度算"没动"
STALL_THROTTLE = 0.50           #: 油门已经踩到这么深还没动
STALL_TRIGGER_S = 1.50          #: 持续这么久就开始阶梯补油
STALL_STEP = 0.05               #: 每次补多少油门
STALL_STEP_EVERY = 0.50         #: 每多久补一次

# ---------------------------------------------------------------- 卡死/瞬移/终点
STUCK_WINDOW_S = 4.0            #: 位置不变判定窗口
STUCK_MOVE_M = 0.30             #: 窗口内位移小于它算卡死
TELEPORT_MIN_M = 5.0            #: 一帧位移超过它（且超过 2*v*dt+1.5）算平台瞬移
IDX_GUARD_M = 40.0              #: 跟踪索引漂移超过它就拉回
HOLD_END_S = 8.0                #: 到终点后驻车保持时间
SOFT_START_SLEW = 0.0015        #: 油门上升率限幅（每帧）

# ---------------------------------------------------------------- 加速度（13 局实测）
# 口径：平台判的是**指令加速度** a_cmd = A·throttle − B·brake，要求全过程 |a_cmd| ≤ 5；
# 油门 0.50/0.55/0.60 恒定 + 零刹车 → +10；油门 1.00 → 扣；刹车 0.05/0.10/0.20/0.30
# （哪怕实测车速没掉）→ 扣。见 accel_window_probe/FINDINGS.md。
ACCEL_GOV_WIN_S = 1.00          #: 观察窗（对齐平台口径）
ACCEL_GOV_TARGET = 4.00         #: 目标加速度（平台线 5.0，留 20% 余量）
ACCEL_GOV_STEP = 0.010          #: 旧版刹车增量（方案 B 不再使用，保留接口）
ACCEL_GOV_FF = 0.09             #: 旧版前馈（保留接口）
ACCEL_GOV_FF_GAIN = 0.06        #: 旧版前馈增益（保留接口）
ACCEL_GOV_BRAKE_MAX = 0.05      #: 治理器自己最多给 0.05 刹车（方案 B 不再产出刹车）
ACCEL_GOV_TRIM_MIN = 0.00
ACCEL_GOV_FF_THROTTLE = 0.02    #: 油门低于它就当滑行，不治理
ACCEL_GOV_FF_DEMAND = 0.30      #: 目标比实际高 0.3 m/s 以上算在加速
ACCEL_GOV_HARD_THROTTLE = 0.15  #: 油门大于它就是硬加速，不看车速差

ACCEL_CMD_A = 8.0               #: 油门折算加速度
ACCEL_CMD_B = 100.0             #: 刹车折算加速度（brk=0.05 就触发一次超限）
ACCEL_CMD_LIMIT = 4.5           #: 指令加速度预算（平台线 5.0，留 10%）
ACCEL_CMD_SLEW = 1.5            #: 指令加速度每帧最大上升量（下跌不限）
THROTTLE_CAP_BONUS = 0.62       #: 拿 +10 的油门上限
BRAKE_CAP_BONUS = 0.00          #: 拿 +10 时刹车必须恒 0
COAST_TRIM_STEP = 0.10          #: 收油治理：目标车速下调步长
COAST_TRIM_MAX = 2.00           #: 目标车速最多下调
COAST_THROTTLE_CUT = 0.30       #: 超限时本帧油门压到这个值以下

# ---------------------------------------------------------------- 元素路由
START_SPAN_NAME = "起步直行"     #: 合成出来的起步段名字（交给直行策略）
START_SPAN_MIN_M = 1.0          #: 第一个元素起点离 0 超过它才需要合成起步段
STRAIGHT_THROTTLE = 0.55        #: 直行/起步的定油门（实测 110）
#: 升挡 kick（2026-10-25，curve-r7 之后）：车辆是「油门 → 稳态速度」快跟随模型，
#: 一挡在 4.78 m/s（17.2 km/h）顶死，**必须把油门顶上去触发升挡**才可能超过它。
#: docs/08-continuous-scene.md 实测：0.85 的持续油门会把车甩到 8.2 m/s 并带出
#: 9.6~13.5 m/s² 的尖峰（curve-r7 元素 2 就是这样丢掉 +10 的）；
#: 温和做法是 0.45~0.50 顶 0.4 s —— 只要够触发升挡，不需要踩到底。
#: 升挡闸门（2026-10-25，curve-r9 之后）：一挡墙 4.78 m/s，**0.50 的温和 kick 实测
#: 一次也没升上去**（7 次 kick 只让车速从 4.78 动到 4.89）——墙就在 4.78，油门不过墙
#: 就永远在墙上。docs/08 实测的稳态映射是：0.40 → 4.78 m/s、0.85 → 8.2 m/s，所以真正
#: 能触发升挡的是 0.85 以上。这里给窄门：只在贴墙 + 目标 >5.2 时把油门顶到 0.86，
#: 车速一过 5.25 立刻收回（最多顶 1.2 s），**不再插入 0 油门段**（curve-r9 的
#: 0→0.5→0→0.56 台阶对本身就是丢 +10 的嫌疑）。
SHIFT_THROTTLE = 0.86           #: 升挡闸门期间的油门
SHIFT_DONE_MS = 5.25            #: 车速过这个值就算升挡完成
SHIFT_MAX_S = 1.20              #: 闸门最长开启时间
SHIFT_MIN_GAP_S = 8.0           #: 两次闸门之间的最小间隔
#: 只有「长元素」才值得为升挡丢 +10：元素 1（车辆直行 51 m）在 4.78 m/s 下 10.7 s
#: 就跑完（限时 100 s），不需要换挡；元素 2（连续转弯 1023 m / 200 s）必须 ≥5.12 m/s，
#: 不换挡就是 0 分。用元素长度当代理判据，短元素一律不许开闸门。
SHIFT_MIN_SPAN_M = 200.0
#: 预升挡窗口：距当前元素结束还有多少米时，可以为了「下一个长元素」提前换挡。
#: 依据 curve-r5（12:51:30 那局）：换挡发生在元素 1 里，元素 1 仍是 110，
#: 元素 2 一进去就在 2 挡 7.91 m/s、油门 0.01~0.31，整段平稳拿 110。
PRE_SHIFT_M = 40.0
#: 闸门的开闸目标速：curve-r11 里元素 2 又零星开了 4 次闸门（tgt 只有 5.2~5.6，
#: 是弯前松油把车速压到墙上触发的）。2 挡以后在弯里掉到 4.x 属于正常滑行，
#: 不该再顶油门——目标速要 >6.0 才值得换挡。每个元素最多开一次。
SHIFT_TARGET_MS = 6.00
#: 长元素（≥SHIFT_MIN_SPAN_M）的速度下限（m03278：防超时，同时把车速钉在二挡稳态区，
#: 避免在 4.3~4.8 的一挡墙上反复穿越——curve-r11 里 4 次误开闸门就是这么来的）。
LONG_SPAN_MIN_MS = 6.00
#: 元素尾巴这么多米内不再抬下限，留出到终点降速的余量。
LONG_SPAN_FLOOR_TAIL_M = 40.0

# ---- 真变道（跨虚线整条平移，lanechange.py）--------------------------------
LANE_CHANGE_RAMP_M = 16.0       #: 斜进目标车道的长度（m）
LANE_CHANGE_HOLD_M = 12.0       #: 在新车道上保持的长度（m）
LANE_CHANGE_LOOKAHEAD_M = 4.0   #: 纯跟踪前视距离（m）
LANE_CHANGE_MIN_M = 2.60        #: 小于这个间距就当成“没有相邻车道”
LANE_CHANGE_MAX_M = 4.20        #: 单次变道最大横移（一条 3.95 m 车道 + 余量）
KICK_THROTTLE = 0.50            #: （旧）温和 kick，实测无法升挡，保留常量仅作对照
KICK_TIME_S = 0.40              #: kick 时长（docs 实测 0.4 s 足以升挡）
KICK_COAST_S = 0.50             #: kick 之后先松油，避免「升挡 + 继续加油」叠成尖峰
KICK_WALL_MS = 4.78             #: 一挡墙（= 17.2 km/h）
KICK_WALL_TOL_MS = 0.15         #: 车速贴住一挡墙的判定带宽
KICK_TARGET_MS = 5.20           #: 目标速度高于这个值才值得升挡
KICK_MIN_GAP_S = 6.0            #: 两次 kick 之间的最小间隔
#: 横向保护：方向盘打得很大时，侧向加速度 v²/R 会顶到 5 m/s²。
#: 满舵半径约 3.61 m，4.5 m/s 时 a_lat ≈ 5.6（curve-r7 元素 3 环岛就是这么丢的 +10）。
RING_STEER_LIMIT = 0.50         #: |方向盘| 超过这个值就算「急转」
RING_LAT_VMAX_MS = 3.20         #: 急转时的最高车速（3.2²/3.61 ≈ 2.8 m/s² 侧向）

UPSHIFT_SPEED_THRESHOLD = 4.9   #: 一挡墙（metacar_race/config.py:56 同值）
RING_SPEED_CAP_MS = 6.0         #: 环岛横向策略给的速度上限（只在它接管纵向时用）

# ---- 环岛曲率限速（2026-10-25 r14 复盘后新增）--------------------------------
# 实测元素 3 = 497.6 m，半径中位 121 m，全段只有两处真急弯：
#   入口 S 弯 s≈1197（R≈7.4~23 m）、出口 s≈1565（R≈7.7 m）。
# r14 用 9 m/s 冲进入口：满舵 0.73 → 侧向 ≈5.6 m/s²（丢 +10），横向 −1.71 m（压线 −20）。
# 做法：v_cap = sqrt(a_lat_budget × R_local)，R 用前后 ±10 点最小二乘圆拟合；
# 减速点用滑行反推（不踩刹车，避免再产生加速度事件）。
RING_LAT_BUDGET_MS2 = 3.50      #: 环岛允许的侧向加速度预算（平台阈值 5.0，留 30% 余量）
RING_V_FLOOR_MS = 4.50          #: 环岛内最低目标车速（497.6/4.5 ≈ 111 s，远低于 200 s 限时）
RING_STEER_RATE = 0.02          #: 每帧方向盘变化上限
# 2026-10-25（m04149 复盘）：r19 把这里放到 0.05 之后，环岛横向最坏值从 r15 的
# |lat|=1.74 m 变成 2.10 m（`accel_window_probe\ring_lat_check.py` 跨跑次统计），
# 用户本轮依旧被判「压道路边线 −20」。所以回退到 r15/r16 拿 110 时的 0.02。
RING_PROFILE_COAST_MS2 = 0.30   #: 反推减速点用的滑行减速度（比实测 0.35 保守）

# ---- 直道纵向规划（用户 2026-10-25 要求）-------------------------------------
# 「前方存在较长直道时，油门从 0 逐渐加大，并根据预测减速到终点（终点速度不为 0）
#   反推松油滑行起点（松油滑行代替刹车）」
# 依据：本车油门是开关型的（>0.035 就 12~15 m/s²），能连续调净加速度的只有刹车；
# 而刹车会吃加速度分（13 局结论）。所以减速一律用「提前松油滑行」，不踩刹车。
COAST_DECEL_MS2 = 0.35          #: 松油后的实测减速度（保守取值，m/s²）
THROTTLE_RAMP_PER_S = 0.30      #: 油门从 0 爬升的斜率（每秒），避免阶跃丢 +10
ELEMENT_END_SPEED_MS = 6.00     #: 元素终点的目标车速（非零，不刹停）
COAST_MARGIN_MS = 0.30          #: 高于允许车速这么多就松油滑行
COAST_LOOKAHEAD_MIN_M = 8.0     #: 前方弯道入口距离小于这个值就按"已到弯"处理

#: 巡航段禁止踩刹车（2026-10-25，curve-r5 元素 1 的教训）
#: 平台判「加速度超限」看的是指令量，任何一次 brk > 0 都会吃掉该元素的 +10；
#: 而基线速度环在「指令还没爬上来」和「超调回落」两种情况下都会点刹。
#: 处理办法：速度环给出刹车时，若这一帧的目标速度并不低（不需要停车），改成松油滑行。
#: 只有目标速度低于本值（真要停车：红灯 / 前车 / 终点）才保留刹车。
BRAKE_ONLY_BELOW_MS = 1.20

#: 方向盘物理上限（本车 |steering| ≤ 1.0）；策略输出前统一夹一下
MAX_STEER = 1.00

# ---- 元素 7「匝道行驶」（683 m 长弯，2026-10-25 新增）-------------------------
# 依据：元素 7 从来没被单独验证过（elem6-r4 只跑到 s=2553.5），基线 9 m/s 纯跟踪
# 在长弯里没有任何曲率保护。这里只接管方向盘 + 曲率限速，纵向仍归基线速度环。
RAMP_LAT_BUDGET_MS2 = 3.00      #: 匝道允许侧向加速度（比环岛 3.5 保守，车速更高）
RAMP_V_FLOOR_MS = 6.00          #: 最低目标车速（683 m / 6.0 ≈ 114 s，限时 200 s）
RAMP_COAST_MS2 = 0.30           #: 反推减速点的滑行减速度
RAMP_LOOKAHEAD_M = 5.00         #: 纯跟踪前视距离（比基线短，弯里贴线）
RAMP_END_SLOW_M = 25.0          #: 终点前这么多里程按停车剖面收速（end_point 检查点）

# ---- 元素 8「车辆紧急停止」（46.4 m，2026-10-25 新增）------------------------
# 平台按「距 end_point 的近距检查点」推进（elem6-r1 实测 0.48~1.95 m），
# 所以必须停进 end_point 并保持，不能停在任意位置。
ESTOP_GAP_M = 0.40              #: 停车点距 end_point 的余量（旧口径，仅参数文档保留）
ESTOP_DECEL_MS2 = 1.50          #: 旧口径停车剖面减速度（已由下面的滑行模型取代）
ESTOP_OVER_MS = 0.30            #: 旧口径超速阈值（已由 COAST 段 + arrest 段取代）
ESTOP_BRAKE = 0.35              #: 收尾段刹车（加速度纪律允许的上限，只在最后 2 m 用）
ESTOP_STOP_M = 0.60             #: 旧口径停车窗口（保留兼容）
ESTOP_STOP_MS = 0.20            #: 旧口径「算停住」的速度
ESTOP_HOLD_BRAKE = 1.00         #: 停住后满刹车保持
ESTOP_HOLD_S = 3.00             #: 保持时间（与 EMERGENCY_STOP_HOLD_SECONDS 一致）
ESTOP_LOOKAHEAD_M = 4.00        #: 纯跟踪前视距离
# 元素 8 改「松油滑行」（2026-10-25 用户要求：减速回顾已有经验，用滑行不用刹车）
ESTOP_CLEARANCE_RATIO = 0.05    #: 停车点 = end_point − 0.05×车长（≈0.235 m），对齐 main 的 emergency_stop_run
ESTOP_COAST_MS2 = 0.25          #: 滑行模型的规划减速度（取实测 0.30 的保守值），allow=sqrt(2·a·d)
ESTOP_FINAL_M = 0.45            #: 距停车点 ≤ 此值 → 满刹保持
ESTOP_ARREST_M = 2.00           #: 距停车点 ≤ 此值且仍快于 ESTOP_ARREST_MS → 补一脚 0.35 刹车
ESTOP_ARREST_MS = 1.20          #: 收尾刹车触发的速度阈值

# ---- 元素 8 v2：端点坐标系停车律（2026-10-25 重写）---------------------------
# 实测依据（elem8-r3，得分 99.28 = 100 − 0.72）：
# * 扣分公式用 main 的两个失败点反解：`penalty = 10 × (|along| − 0.20)`，
#   `along` = 定位点沿终点法向到 end_point 的距离（正=在线前，负=越过），
#   理想 0、容差 ±0.20 m。本次 −0.72 ⇒ 实际越过 0.272 m。
# * 旧实现的根因：`d` 由整米网格的 `route_s` 算，`d > 0.45` 时返回的是**正的限速**
#   `sqrt(2·0.25·d)`（d=1.7 时 0.92 m/s）而不是 0，StallGuard 又补 0.09~0.27 的油门，
#   于是车以 ±0.4 m/s 抖着往前蹭了 2 m / 9 s，最后过线才满刹。
# * 新律离线试算：`accel_window_probe/scratch_estop_sim.py`，进场 5.04 m/s →
#   17 s 停住、最大刹车 0.03、落点 along ≈ 0.14 m（2/3/4 m/s 进场也落在 0.14~0.15）。
ESTOP_ALONG_TARGET_M = 0.12     #: 停车目标：end_point 前 0.12 m（容差 ±0.20 的中间偏安全侧）
ESTOP_APPROACH_MS2 = 0.25       #: 远段允许的规划减速度（= 滑行能力），allow = sqrt(2·a·gap)
ESTOP_FINE_ALONG_M = 8.0        #: 进入精细段的 along（此段用 a_need 反算刹车）
ESTOP_FINE_BRAKE_MAX = 0.28     #: 精细段刹车上限（低于 0.30，且全程不许阶跃）
ESTOP_BRAKE_GAIN_MS2 = 3.0      #: 刹车指令 → 减速度的换算斜率（1.0 brake ≈ 3.0 m/s²）
ESTOP_BRAKE_RAMP_PER_S = 0.60   #: 刹车指令爬升斜率（0.02/帧，避免把指令写成阶跃）
ESTOP_SELFCHECK_M = 8.0         #: 首帧自检：|along − (s_end − s_now)| > 此值 ⇒ end_point 不属于本元素
ESTOP_SETTLED_MS = 0.12         #: 算「停住」的速度
ESTOP_SETTLED_GAP_M = 0.12      #: 算「停住」的 along − target 窗口
ESTOP_SETTLED_ALONG_M = 0.06    #: 已经压到 end_point 前 0.06 m 也直接算停住
ESTOP_CREEP_THROTTLE = 0.03     #: 停早了时的最小蠕动油门（显式下发，压掉 stall 的 0.5）
ESTOP_CREEP_MAX_THROTTLE = 0.08  #: 蠕行 6 s 内线性加到的上限（怕 0.03 落在死区里推不动）
ESTOP_CREEP_TARGET_MS = 0.25    #: 蠕行的目标速度
ESTOP_CREEP_LIMIT_S = 6.0       #: 蠕行最长时间（超时就不再往前挪）
ESTOP_STUCK_ALONG_M = 0.30      #: 停在这里还没到点、也推不动：算停住（避免一直蠕行）
ESTOP_NEAR_ALONG_M = 3.0        #: 进入这个距离才记「近线平台元素名」（交棒判据的基线）
ESTOP_HOLD_RAMP_S = 1.0         #: 保持段刹车斜坡时间（零指令保持版本不再用，留作回退）
ESTOP_HOLD_IDLE_MS = 0.30       #: 保持段只在速度超过它时补一帧 0.10 轻刹（静止车不踩刹车）
ESTOP_RELEASE_MAX_S = 12.0      #: 兜底：最多保持这么久就交棒（平台元素名一直没变时）

# ---- 元素 8 第二轮：加速度 +10 与零刹车起跑（2026-10-25，elem89-r1 实测）-------
# 结算表（m00443）：元素 8 得 100，没有 +10。逐帧差分定位到唯一超限点——
# f13032 t=23.05 `v −0.00→−0.18 @ brk=1.00`（a=−5.16）、f13038 t=23.25
# `v 0.17→0.01 @ brk=1.00`（a=−5.04）：满刹 1.00 顶在一辆**静止**的车上，
# 速度逐帧抖 ±0.18 m/s。判据是「元素内 |a| ≤ 5 m/s²」（docs/08-continuous-scene.md），
# 元素 3 的 5.13 也拿了 +10，说明 5 附近有容差噪声——把驻车段压到 4 以下才稳。
# 同时元素 9 丢 +10 的元凶也是交棒瞬间：基线给了 2 帧 brk=0.35（安全闸，blk=6.82<7）
# + 0→0.56 油门阶跃（a=8.49）。⇒ 修法：保持 D 挡、刹车从 0.10 爬到 0.55；平台切元素后
# 自己零刹车起跑 24 m，再交棒。
# 2026-10-26 11:24 那一局（elem12-r1）就是这样拿到元素 8 = 110 的；当天下午改成
# 「零指令保持」后掉回 100（elem12-r2），已回退——别再往「刹车越少越好」方向改。
ESTOP_HOLD_BRAKE_MAX = 0.55     #: 保持段刹车斜坡的上限（不用 1.00：满刹在静止车上会抖出 −5 m/s²）
ESTOP_LAUNCH_M = 24.0           #: 零刹车起跑跑出这么远再交棒（元素 9 从 s_end+24.9 m 才开始）
ESTOP_LAUNCH_MS = 7.0           #: 起跑目标速度
ESTOP_LAUNCH_RAMP_PER_S = 0.10  #: 起跑油门爬升斜率（每帧 +0.003）
ESTOP_LAUNCH_THR_MAX = 0.30     #: 起跑油门上限（二挡 0.30 ≈ 7~8 m/s，够接上元素 9）
ESTOP_LAUNCH_MAX_S = 12.0       #: 起跑最长持续时间（兜底交棒）

# ---- 元素 9「辅路驶入、驶出」（1336.7 m，2026-10-25 新增）--------------------
# 依据 main 的 `templates/scenes/zyyxnjydhcg/fudao_run.py`（RouteFollower，单场景
# 443 m / 99 s / 110 分）：跟 `static.route` 走、不叠曲率限速、收尾轻刹到 0.05 m/s
# 以下再挂 P 保持 5 s（硬停一帧 4.8→0 就是 5 m/s² 的加速度事件，会丢 +10）。
# 连续场景的差别：元素 9 有 1336.7 m / 限时 200 s ⇒ 必须 ≥6.68 m/s；而元素 8
# 把车停住后变速箱退回一挡，driver 的 `_shifted_ok` 是全局一次、已经用掉（m03278）
# ⇒ 元素 9 必须自己补一次升挡 kick（元素 1 实测 0.86 油门 0.43 s 就能升挡）。
FUDAO_LOOKAHEAD_M = 5.00        #: 纯跟踪前视距离（与匝道一致，弯里贴线）
FUDAO_HANDOVER_M = 6.0          #: 距 s_end 这么多米就 return None，把元素 10 交给基线
FUDAO_KICK_ON = True            #: 是否允许自己补一次升挡 kick
FUDAO_KICK_THROTTLE = 0.88      #: kick 的油门（元素 1 实测 0.86 能触发升挡，留 0.02 余量）
FUDAO_KICK_RAMP_PER_S = 0.70    #: kick 油门爬升斜率（0.62 → 0.88 约 0.37 s）
FUDAO_KICK_MAX_S = 2.2          #: 单次 kick 最长持续时间
FUDAO_KICK_MIN_TAIL_M = 200.0   #: 剩余里程不足这么多就不 kick（别在收尾处顶油门）
FUDAO_KICK_MIN_GAP_S = 6.0      #: 两次 kick 的最小间隔

# ---- 元素 9 起跑缓升（2026-10-25 补，元素 8 交棒之后）-------------------------
# elem89-r1 的实测：元素 8 停车后由基线直接接过纵向，第一帧就 `thr=0.56` 把车从
# 0 顶到 1.5 s 9.14、2.5 s 11.27 m/s，|a| 峰值 8.49（元素 9 的 +10 就是这么丢的，
# 另有两帧安全闸 brk=0.35）。estop 的零刹车起跑只到 `s_end + ESTOP_LAUNCH_M(24 m)`，
# 而 span 在 3066.8 m（=s_end+24.9）就切给辅路，所以起跑速度只有 3 m/s 左右；
# 直接交回基线仍然是阶跃。这里由辅路策略自己把油门按 0.10/s 爬到 0.30、跑到
# 7 m/s 再交回基线（那时基线需要的油门也是 0.3 左右，交接没有阶跃）。
FUDAO_LAUNCH_MS = 7.00          #: 起跑缓升的目标速度，到这速度就交回基线纵向
FUDAO_LAUNCH_THR_MAX = 0.30     #: 缓升期间的油门上限（二挡 ~3.5 m/s²，进不了加速度事件）
FUDAO_LAUNCH_RAMP_PER_S = 0.10  #: 缓升斜率（0→0.30 要 3 s）
# 纵向自己接管（不再交回基线）。elem12-r2（2026-10-26 12:11）的元素 9 就是这一版：
# 起跑缓升到 7 m/s 后按巡航律继续给油，全程 0 帧刹车、|a| 峰值 4.11、vmax 9.63。
FUDAO_CRUISE_MS = 8.80          #: 巡航目标速度（1336.7 m / 200 s 需 ≥6.68，留余量）
FUDAO_CRUISE_UNDER_MS = 8.60    #: 低于这个速度就加油（0.02/帧）
FUDAO_CRUISE_OVER_MS = 9.20     #: 高于这个速度就收油（0.02/帧）
FUDAO_CRUISE_THR_STEP = 0.02    #: 每帧油门步长（0.02 ≈ 0.6 m/s²，进不了加速度事件）
FUDAO_CRUISE_THR_MAX = 0.20     #: 巡航油门上限（二挡 0.20 约 9 m/s）
FUDAO_HOLD_BACK_GAP_M = 25.0    #: 前方 25 m 内有车/灯时交回基线（不在辅路里自己跟停）

# ---- 元素 12「平行车道泊车」（2026-10-25 新增）--------------------------------
# 依据 main 的 `metacar_race/control/parallel_parking.py`（PR#19，平行泊车连续 5 次
# 210 分、停位离散 < 3 cm）与 `docs/parallel-parking-calibration.md` 的四个坑：
#   ① staging 必须与「远端那辆车」并排（远端车中心 +0.15 m），不能按车位中心加偏移；
#   ② 舵角必须斜坡打（1.2 s 到 0.75），低速先给连续小油门破静摩擦，否则车轮顶死；
#   ③ 收尾不能纯滑行（会多滚 1 m 把 dy 带偏），要 0.06 轻刹收速；
#   ④ 换向前先停稳，避免正负加速度叠加。
# 本站连续场景 4 是它的 180° 旋转（车位在行驶方向右侧、ego 沿 +y 开），所以
# `scene4/park.py` 把位姿与障碍物映射进「绕车位中心旋 180°」的规范坐标系，
# 原样调用该控制器，再把控制量映射回来（旋转不改手性，steering 符号不变）。
# 上一轮 elem89-r1 的教训：基线把车开到被挡的车屁股后停下（s=4716.4，距路线
# 终点 2.06 m），从未倒车入库 ⇒ 元素 12 整行消失 = 0 分（用户 m00443）。
PARK_LATERAL_M = 3.00           #: staging 侧距（= 两段等半径圆弧的总横移 L）
PARK_APPROACH_MS = 5.00         #: 接近段目标速度（从元素 11 的 6~9 m/s 收到这里）
PARK_APPROACH_DECEL_MS2 = 1.20  #: 接近段规划减速度（松油滑行即可，别踩大刹车）
PARK_APPROACH_GAIN_DEG_PER_M = 8.0   #: 侧向误差 1 m 换多少度航向偏置
PARK_APPROACH_YAW_MAX_DEG = 12.0     #: 航向偏置上限（再大就是甩尾了）
PARK_APPROACH_STEER_GAIN = 0.020     #: 航向误差 1° 换多少舵（0.02 → 12° 给 0.24）
PARK_APPROACH_STEER_MAX = 0.30       #: 接近段舵角上限
PARK_APPROACH_THR_MAX = 0.35         #: 接近段油门上限
PARK_APPROACH_THR_RAMP_PER_S = 0.10  #: 油门爬升斜率（避免 0→0.35 阶跃）
PARK_APPROACH_BRAKE = 0.06           #: 超速时的轻刹（|a|≈1.6 m/s²，不触发加速度事件）
PARK_HANDOVER_REM_M = 0.30      #: 距 staging 这么多米、且速度够低就交给控制器
PARK_HANDOVER_MS = 0.30         #: 交给控制器时的速度上限
PARK_HANDOVER_LAT_M = 0.45      #: 交给控制器时的侧向误差上限（超过就继续修）
PARK_HOLD_BRAKE = 0.55          #: 入库完成后的驻车刹车（同元素 8，不用 1.00）
PARK_ALIGN_MS = 6.00            #: 对齐段巡航上限（6.0：接近 70 m 从 22 s 缩到 ~13 s）
PARK_ALIGN_DECEL_MS2 = 1.00     #: 对齐段规划减速度（√(2·1.0·dist)，6 m/s 在 18 m 处开始收）
PARK_TOL_XY_M = 0.15            #: 交给控制器后的 FIT 容差（收紧到 15 cm 才允许收尾）
PARK_FIT_TIMEOUT_S = 20.0       #: FIT 超时（超时把容差放宽，先停进去再说）
PARK_TOL_RELAX_M = 0.35         #: 超时后放宽的容差（= 参考控制器原值）


@dataclass
class DriverConfig:
    """一次跑次的全部可调项（命令行 → 这里 → driver，不再临时改模块常量）。"""

    # 场景/流程
    skip_to_index: int = 0           #: 已停用（AGENTS.md 铁律 9）；非 0 会在 __post_init__ 报错
    vmax: float = 8.0                #: 速度上限
    max_seconds: float = 700.0       #: 墙钟超时
    end_hold: float = HOLD_END_S     #: 终点驻车保持秒数
    shots: float = 0.0               #: 每隔多少秒存一张前视相机图（0=不存）
    out: Path | None = None          #: 产物目录（None = <repo>/logs/park_cs4/<时间戳>）
    log_every: int = 30              #: 每多少帧打一行进度

    # 功能开关
    overtake: bool = True            #: 超车元素内启用并线状态机
    stanley: bool = True             #: 环岛段用 Stanley 兜底
    light: bool = True               #: 处理信号灯与停止线
    soft_start: bool = False         #: 油门上升率限幅
    accel_gov: bool = True           #: 加速度治理（收油）
    bonus: bool = False              #: +10 模式（油门 ≤0.62、刹车恒 0、|a_cmd| ≤4.5）

    # 覆盖项（None = 按 bonus 推导）
    thr_cap: float | None = None
    brk_cap: float | None = None
    a_cmd_limit: float | None = None
    a_cmd_slew: float | None = None

    # 直驱
    fixed_throttle: float = 0.0      #: >0 时绕过速度环，整段直发这个油门

    # 元素路由
    elements: bool = True            #: 元素级策略路由
    elements_only: tuple[str, ...] = ("起步直行", "车辆直行", "连续转弯", "环岛", "复杂路况超车",
                                      "匝道行驶", "车辆紧急停止")  #: 空元组 = 全部接管
    lane_element: str = ""           #: 非空时，名字含该字样的元素改走「真变道」（lanechange.py）

    # 由 __post_init__ 推导出来的实际预算
    caps: tuple[float, float, float, float] = field(default=(), init=False)

    def __post_init__(self) -> None:
        if int(self.skip_to_index) > 0:
            raise ValueError("跳关已停用（本地 AGENTS.md 铁律 9）：skip_to_index 只能为 0，"
                             "请用 elements_only 限定接管的元素")
        # 2026-10-25（curve-r7 之后）：加速度纪律改成**默认开启**——
        # 平台判加速度看的是指令量（油门 → 命令加速度 8·thr），所以默认就把
        # 油门压到 0.62（8×0.62 = 4.96 ≤ 5）、|a_cmd| 限 4.5、上升斜率限 1.5。
        # 刹车上限仍留 1.0：急刹（blk_ctrl 0.35）/终点驻车必须能刹得住。
        thr_cap = self.thr_cap if self.thr_cap is not None else THROTTLE_CAP_BONUS
        brk_cap = self.brk_cap if self.brk_cap is not None else 1.0
        a_limit = self.a_cmd_limit if self.a_cmd_limit is not None else ACCEL_CMD_LIMIT
        slew = self.a_cmd_slew if self.a_cmd_slew is not None else ACCEL_CMD_SLEW
        self.caps = (float(thr_cap), float(brk_cap), float(a_limit), float(slew))


__all__ = [name for name in dir() if not name.startswith("_")]
