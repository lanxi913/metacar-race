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
#: 2026-10-25 元素 6 实测：安全闸用 `brk=0.35` 一次就把指令加速度打到 −35（平台上限 5），
#: 515 帧 |a|>5、809 帧刹车全来自这一脚。改成"软刹 0.05（a_cmd=−5）+ 真急停才放 0.35"。
BRAKE_SOFT_CMD = 0.05           #: 软刹：a_cmd = −100×0.05 = −5，正好卡在预算线上
BRAKE_EMERG_DECEL_MS2 = 1.9     #: 要把停住所需减速度超过这个值（a_cmd 预算内做不到）才允许急刹
#: 2026-10-25 p1 元素 8 实测：跟车目标用 `0.45×(d−1.5)`，在 11.8 m 处要求 4.6 m/s
#: 而车速还有 7.4 m/s，剩下的距离根本减不下来 → 最后一脚 `brk=0.35`（a_cmd −35、
#: 实测 −17 m/s²）。改成「运动学安全车速」提前收：`v_safe = sqrt(2·a·(d−margin))`，
#: a 取软刹能做到的 1.3 m/s²，于是根本不会进入急刹分支。
BLOCK_DECEL = 1.50              #: 预判减速用的减速度（软刹 + 滑行阻力能做到的）
BLOCK_STOP_MARGIN_M = 1.5       #: 安全车速剖面里给前车留的余量
#: 2026-10-25 元素 8（车辆紧急停止）实测：`ObstacleScan` 的 `dist` 是**中心到中心**，
#: 前车长 4.89 m、自车 4.7 m，center 间距 4.9 m 时保险杠已经贴上（撞车 −10.59 分，
#: v=2.118 m/s）。所以"真正可用"的空隙必须两段余量一起扣：车身余量 + 停车余量。
BLOCK_STOP_GAP_M = BLOCK_MARGIN_M + BLOCK_STOP_MARGIN_M   # 6.3 m
#: 需要多大减速度才算"再不动手就顶上了"（软刹 a_cmd=−5 换算到车端约 1.3~1.6 m/s²）
BLOCK_DECEL_URGENT = 1.20
COAST_TRIM_STEP = 0.10          #: 收油治理：目标车速下调步长
COAST_TRIM_MAX = 2.00           #: 目标车速最多下调
COAST_THROTTLE_CUT = 0.30       #: 超限时本帧油门压到这个值以下

# ---- 实线余量保护（borders.py）---------------------------------------------
# 2026-10-25 p2 / p4 对比：环岛出口车身四角离实线的最近余量分别是 0.02 m 与
# 0.01 m（同在 (-3028,-889) 一带），平台只把 p4 判成「压道路边线 −20」。1 cm 的
# 差别离线无法复现、任何控制也保证不了，所以不追求贴着线走，而是把实线余量整体
# 抬到 `BORDER_SAFE_M` 以上：近侧在左就往右带舵，反之往左。
BORDER_SAFE_M = 0.55            #: 车身四角到实线的目标最小余量（米）
BORDER_SLOW_M = 0.30         #: 只有余量掉到这个值以下才降速（转向补偿另有 0.55 m 目标）
BORDER_STEER_K = 0.22           #: 每差 1 m 余量补多少舵（归一化舵角/米）
BORDER_STEER_MAX = 0.22         #: 单帧舵角补偿上限
BORDER_CELL_M = 6.0             #: 实线线段网格索引的格子边长

#: 2026-10-25 p4 元素 6（复杂路况超车）实测：并线起点 `v_hold=8.00` 是进并线那一刻
#: 的车速，车一边打舵一边掉到 5.78，速度误差 2.2 m/s 把油门顶到 0.56（a_cmd=4.5
#: 正好卡在限幅），换挡/扭矩台阶让平台连看 11 帧 8.48 m/s² → 丢 +10。改成并线期间
#: 目标速度夹到当前车速（只保持、不追），提速全部留到 PASS 直线段。
OVERTAKE_HOLD_NO_ACCEL = True   #: True = SHIFT_OUT / SHIFT_BACK 期间不加速

# ---- 低速油门限幅 ---------------------------------------------------------
# p4 元素 9 实测：进元素时车停着，油门 0.56 在 1.2 s 内把车从 0.14 甩到 8.42 m/s
# （实测 8.0~8.3 m/s²、41 帧超限）→ 丢 +10。`a_cmd = 8·thr` 只是折算值，一挡低速
# 扭矩放大后同样 0.56 的油门实测能到 8 m/s²，所以低速段必须按**实测速度**再压一手。
LOW_SPEED_THR_MS = 2.0          #: 低于这个车速
LOW_SPEED_THR_CAP = 0.28        #: 油门封顶（≈2.2 m/s² 指令值）
MID_SPEED_THR_MS = 3.5          #: 再往上一档
MID_SPEED_THR_CAP = 0.40        #: 油门封顶（≈3.2 m/s²）
BLOCK_STOP_EXTRA = 0.80         #: 跟停时在 BLOCK_STOP_GAP_M 之外再留的余量（够近直接要 0）

#: 2026-10-25 元素 8（车辆紧急停止，评分规则表 4）：要求「后车中心点停在前车中心点
#: **5 m** 处、保持 5 s、速度加速度均为 0」，位置误差 e 米扣 10×e；碰车再按 5×v 扣。
#: p4 实测停在 4.89 m（e=0.102、还以 0.424 m/s 蹭上）——因为通用跟停余量
#: BLOCK_STOP_GAP_M=6.3 是为了**不撞**，对这个元素太远；太近又会碰。所以这个元素
#: 单独给一套口径：目标 5.10 m（e≈0.10），只留 0.05 m 的"够近就要 0"余量。
EMERGENCY_STOP_GAP_M = 5.00     #: 元素 8 与前车中心的停车距离（规则要求 5 m）
#: 2026-10-25 「强制停车」改造后实测（离线模拟 hold 剖面）：自车从 8 m/s 按 1.5
#: m/s² 收速，会在距目标 0.1~0.2 m 处落到 `HOLD_LOCK_MS` 以下并锁定，所以目标直接
#: 取规则值 5.00，落点 ≈5.05~5.12（e≤0.12 ⇒ 扣 ≤1.2）。原来取 5.10 是因为旧逻辑
#: 靠"蹭到 4.9"碰巧接近 5.0（p5 实测 4.89 / e=0.11），那是运气不是设计。
EMERGENCY_STOP_EXTRA = 0.05     #: 元素 8 的"够近就要 0"余量

#: 2026-10-25 起步咬合助力。
#: 平台只豁免开局约 0.5 s 的加速度超限（上下坡驻车实测：0.20 s 咬合 → 210，
#: 0.60 s 咬合 → 200）。p1 跑次里元素 1 的超限帧落在 t=0.367~0.504 s、峰值
#: +20.88 m/s²，原因是油门从 0.010 慢慢爬到 0.161，把离合咬合拖到了 0.5 s 边界；
#: 而队友满分局是「0→4.83 m/s 只用 0.43 s」。所以车还没滚起来时直接给足咬合油门，
#: 让整段冲击落在豁免窗内（油门 0.15 的指令加速度只有 1.2，本身不违规）。
LAUNCH_THR = 0.25               #: 静止起步时的咬合油门（单帧到位；0.15 实测要 0.47 s 才咬合，超出平台 0.5 s 豁免窗口）
LAUNCH_BITE_MS = 0.50           #: 车速低于它就算"还没滚起来"
LAUNCH_DEMAND_MS = 1.00         #: 目标车速高于它才需要起步（否则是在停车/等灯）

#: 2026-10-25 「强制停车」指令（`scene4/holdctl.py`）。
#: 用户要求：必须停的时候一次停住、不要顿顿卡卡；要有前瞻性，前方有车挡就提前
#: 减速、停住，车走了再走。落地成显式状态机，只有它一处能决定"停"：
#:   run → decel（运动学剖面提前收速）→ hold（软刹 + P 挡锁死）→ release（一次咬合）。
#: 为什么刹车必须 ≤0.05：实测 brk=0.35 一帧 `a_cmd=8·0−100·0.35=−35`，车端 −13~−18
#: m/s²；brk=0.05 正好 a_cmd=−5，卡在预算线上（`BRAKE_SOFT_CMD` 同款口径）。
HOLD_BRAKE = 0.05               #: 锁定过程中的刹车（a_cmd=−5，不许再大）
HOLD_LOCK_MS = 0.12             #: 车速低于它才挂 P 挡（0.12/0.033≈3.6 m/s²，压得住）
HOLD_LOCK_EXTRA = 0.35          #: （旧）距"目标停车点"这么近就锁定，已改用预测滑行距离
HOLD_STOP_TOL = 0.10            #: 预测停车点落在目标点这么多米内就锁（1 挡滑行 1.5 m/s² 的精度）

# ---- 时钟（场景/元素时限）：都来自平台规则，不是某一局的成绩 ----------
SCENE_TIME_LIMIT_S = 810.0      #: 连续场景总时限（docs/SCENE4 与 AGENTS.md 记录的赛事规则）
ELEM_CLOCK_MARGIN = 1.02        #: 元素时钟律的余量（刚好赶上会因抖动错过，留 2%）
HOLD_LOCK_WINDOW = 1.20         #: 已经趴慢时，距目标点这么近就锁定
HOLD_RELEASE_M = 2.00           #: 前车拉开这么远才算"路让开了"
HOLD_RELEASE_S = 0.30           #: 并且要稳定这么久（防前车晃一下就当成放行）
SLOT_APPROACH_DECEL = 0.80      #: 接近车位时的规划减速度（只靠软刹+滑行就能做到）
SLOT_APPROACH_STOP_M = 6.00     #: 按这个余量算接近包络（车位中心到车）
SLOT_APPROACH_MIN_MS = 2.60     #: 接近车位的最低车速（再慢就没法并线了）
# 2026-10-25 p6 实测教训：hold 只要"靠近就锁"，在连续场景里会被慢车队列反复触发，
# 每次锁 = 1 s 干等 + 从 0 重新加速 ≈7 s，几十次吃掉 300+ s（p6 全程 1540 s 超时）。
# 改成测量驱动：**只有前车真的停住**（连续 HOLD_LEAD_STOP_S 内速度 < HOLD_LEAD_STOP_MS）
# 或者本段必须停（紧急停止 / 红灯 / 终点）才允许锁；前车在动就只跟速、永不停車。
HOLD_LEAD_STOP_MS = 0.50        #: 前车速度低于它算"停住"
HOLD_LEAD_STOP_S = 0.40         #: 并且要连续停这么久（防差分之噪声）
SHIFT_MAX_COUNT = 2             #: 一局最多升几次挡（每次升挡 ≈ 丢一个元素的 +10）
SHIFT_WALL_HOLD_S = 0.60        #: 贴一挡墙驻留这么久才算"真的被墙挡住"（二挡路过不算）
SHIFT_NEED_MS = 5.20            #: 剩余里程/剩余时间超过它就算"不升挡会超时"

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
    border_guard: bool = True        #: 实线余量保护（车身四角离实线 ≥ BORDER_SAFE_M）
    low_speed_cap: bool = True       #: 低速油门限幅（一挡低速扭矩放大，按实测车速压油门）
    scene_root: str = r"D:\无人算法\场景包\scenes"  #: 场景包根目录（按路线指纹认 mapd）

    # 覆盖项（None = 按 bonus 推导）
    thr_cap: float | None = None
    brk_cap: float | None = None
    a_cmd_limit: float | None = None
    a_cmd_slew: float | None = None

    # 直驱
    fixed_throttle: float = 0.0      #: >0 时绕过速度环，整段直发这个油门

    # 元素路由
    elements: bool = True            #: 元素级策略路由
    elements_only: tuple[str, ...] = ("起步直行", "车辆直行", "连续转弯", "环岛")  #: 空元组 = 全部接管
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
