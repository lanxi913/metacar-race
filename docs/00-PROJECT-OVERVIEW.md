# 项目总览（背景 · 目标 · 现状 · 交接）

> 这份文件给**新加入的人或 AI**看：读完它 + `docs/PROGRESS.md`，就能接手干活。
> 最后更新：2026-10-24

---

## 一、这是什么

**2026 智能无人系统算法挑战赛**（智能车 / 自动驾驶赛项）。

| 项 | 内容 |
|---|---|
| 报名截止 | 2026-10-25 |
| 正式比赛 | 2026-11-01 |
| 仿真平台 | 元动力 MetaCar（Windows 客户端 + Python SDK） |
| 仓库 | https://github.com/lanxi913/metacar-race （private） |
| 队伍 | 5 人，全员用 AI 协作，代码统一进 main |

**比赛形式**：在仿真里让车完成一个个「元素场景」（直行、转弯、泊车、路口通行…），
每个场景由平台自动打分。

---

## 二、评分规则（必须理解，否则白干）

| 类型 | 满分 | 说明 |
|---|---|---|
| 元素场景 | **110** | 基础 100 + 奖励 10 |
| 泊车 / 路口类 | **210** | 基础 200 + 奖励 10 |
| 连续场景 | 多个元素累加（如连续场景 1 = 630） | 元素拼装后一起跑 |

### 奖励分（+10）怎么拿

**全程不触发任何违规**才给。仿真右上角会实时打红字：

```
[加速度限制] 加速度高于允许上限
[加速度限制] 加速度低于允许下限
[压线检测] 压路边沿线扣5分
[压线检测] 压双黄实线扣5分
[位置误差] 停车位置误差扣5分
```

**出现任意一条，奖励分就没了**（110→100），压线还会额外扣分。

### 加速度红线（本项目最容易丢分的地方）

- 上限 ±5 m/s²，**起步、刹车、转弯都算**
- 横向加速度也计入（弯道压线时同时报加速度超限）
- **不许踩急刹**：减速度超限同样扣奖励分
- 反直觉的事实：这个仿真车**任何非零油门都会蠕行到 4.76 m/s**（低挡位特性），
  所以「慢」只能靠别的手段，不能靠轻踩油门

---

## 三、技术环境

| 项 | 值 |
|---|---|
| Python | 项目自带 `.venv`（**必须用它**，系统 python 会 `ModuleNotFoundError: metacar`） |
| SDK | `metacar 0.4.0`、`metacar_multiagent 0.1.0` |
| 通信 | `SceneAPI` 是**服务端**：先起脚本（阻塞在「等待场景连接」），再点客户端「启动场景」 |
| 端口 | 单场景 5061；多车场景 `ScenarioTaskAPI(port=9100)` |
| 代理 | git push 需要 `http://127.0.0.1:7897` |

### 坐标与符号约定（已验证，照抄不要重推）

```python
yaw = math.radians(-pose.ori_z)        # 场景 yaw 顺时针为正 → 数学航向取负
lx =  dx*cos(yaw) + dy*sin(yaw)
ly = -dx*sin(yaw) + dy*cos(yaw)
delta = math.atan2(2.0*ly, d)
steer = -delta / 0.6                   # ← 这个负号实测验证过
```

- 速度 `msg.main_vehicle.speed` 单位 **m/s**（4.76 m/s ≈ 17.2 km/h，低挡位上限）
- 满舵倒车半径 **R = 5.26 m**，前进满舵约 4.2 m，转向不对称
- 车位障碍物 `type` 是**字符串** `'PARKING_SLOT'`，不是数字 30
- 障碍物字段是 `pos_x / pos_y / ori_z / len / wid`（**没有 center**）

---

## 四、成绩现状（唯一权威表在 `docs/PROGRESS.md`）

### 已满分（13 个场景）

| 场景 | 分数 | 用哪份代码 |
|---|---|---|
| 车辆直行 | 110 | `main.py --track --cruise 0.5` |
| 车辆停止 | 110 | `tools/stop_run.py` |
| 直角转弯 | 110 | `tools/track_bonus.py` |
| u 型弯 | 110 | `tools/track_bonus.py --offset 0.8` |
| 连续转弯 | 110 | `metacar_race/control/` + 终点提前减速 |
| 环形转弯 | 110 | `control/ring_turn.py` |
| 环岛驶入、驶出 | 110 | 路线平滑 + Stanley |
| 限速行驶 | 110 | `speed/main.py`（参数现场读） |
| 车辆变道 | 110 | `lanechange/main.py`（S 弯 + Stanley） |
| 左侧车道超车 | 110 | 车道级循迹 + 换道避让 |
| 辅路驶入驶出 | 110 | 同上 |
| 车辆紧急停止 | 110 | `tools/emergency_stop_run.py` |
| 垂直车道泊车 | **210** | `tools/park_v4.py` ★**效果最好**（2026-10-25 复核 210.00，中心误差 0.14 m / 航向 1.5°）；同分备选 `tools/park_v8.py`（几何推导，无场景常数） |
| 平行车道泊车 | **210** | `control/parallel_parking.py` |
| 十字路口 | **210** | 读 SDK 停止线与信号灯 |
| 丁字路口 | **210** | 矢量加速度预算 |
| 无保护左转 | **210** | 自写让行判断 |

### 未完成

| 场景 | 状态 | 卡在哪 |
|---|---|---|
| 车辆跟随 | 100 | 1 次加速度违规，差奖励分 |
| **异形车道泊车** | 未跑通 | 路径没约束路面、倒车符号未标定 → 见 `docs/11-angled-parking-handoff.md` |
| **斜坡泊车** | 未做 | 平行车位 + 15° 坡，坡度让加速度更难压 |
| 连续场景 2~5 | 未做 | 元素拼装，可复用现有控制器 |
| VLA 场景（6 个）、港口迷雾 | 未碰 | 需要视觉/语言模型输入 |

---

## 五、代码地图

```
main.py                     车辆直行入口
metacar_race/control/       控制库（15 个模块）
  ├ tracking.py / route_follower.py / geo_follower.py   循迹
  ├ lane_follower.py       车道保持
  ├ parking.py / vertical_park.py / parallel_parking.py / reverse_park.py  泊车
  ├ ring_turn.py           环形转弯（564 行，最复杂）
  ├ follow.py / emergency_stop.py   跟车与紧急停止
  └ stop.py / pid.py / profile.py   基础件
crossroad/ lanechange/ leftturn/ parking/ roundabout/ speed/ straight_full/ tjunction/
                           各场景独立入口（main.py）
templates/                  可复用模板（stop / rightturn / parking / scenes）
tools/                      脚本工具
  ├ track_bonus.py          ★ 通用循迹（奖励分优化版，支持 --offset/--cscale/--nobrake）
  ├ park_v4.py              ★★ 垂直泊车【效果最好】（210 分，中心误差 0.14 m / 航向 1.5°，优先复用）
  ├ park_v8.py              ★ 垂直泊车（几何推导，210 分；同分备选）
  ├ park_cs_single_arc.py   垂直泊车（连续场景单圆弧移植，同属 park_v4 一系）
  ├ stop_run.py             ★ 车辆停止（3 cm 判据，110 分）
  ├ follow_run.py / emergency_stop_run.py   跟车与急停入口
  ├ park_angled*.py         异形泊车（未完成，v4 是当前版本）
  ├ night_run.ps1           通宵自动跑分（降温 + 清窗口 + 存图）
  ├ launch_scene.ps1        按场景名点「启动场景」
  ├ capture_window.ps1      按窗口句柄抓图（不受焦点影响）
  ├ probe_*.py              标定/探针（车位、转向、最大速度）
  └ download_scenes.py / fix_scene_dirs.py  场景下载与目录修复
docs/                       文档（见下）
```

---

## 六、文档索引

| 文件 | 什么时候读 |
|---|---|
| `docs/PROGRESS.md` | **每天第一份**：成绩总表、参数速查、排错 |
| `docs/11-angled-parking-handoff.md` | 做异形泊车时必读（场景数据 + 三条硬要求 + 下一步） |
| `docs/10-todo-verification.md` | 未完成项清单与验证顺序 |
| `docs/training-summary.md` | 循迹类参数细节、加速度经验 |
| `docs/05-parking-notes.md` / `07-parking-general.md` | 泊车几何推导 |
| `docs/06-results.md` / `results.csv` | 逐局跑分记录 |
| `AGENT_PLAYBOOK.md` | 参赛注意事项总纲（队友整理） |

---

## 七、跑一局的固定流程

```bat
:: 1) 清残留（端口占用会导致场景一直"等待算法连接"）
powershell -c "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | ? { $_.CommandLine -like '*track_*' } | % { Stop-Process -Id $_.ProcessId -Force }"

:: 2) 起脚本（阻塞在"等待场景连接"是正常的）
.venv\Scripts\python.exe -u tools\track_bonus.py --offset 0.8

:: 3) 再点「启动场景」（泊车类在第 2 页，先切页）
powershell -File tools\launch_scene.ps1 -Title "u型弯"

:: 4) 看仿真右上角有没有 [加速度限制] 红字；结算画面「总分」即成绩
```

**自动跑分**（通宵用）：

```bat
powershell -File tools\night_run.ps1 -RunTimeoutSec 200 -CooldownMinutes 90 -CooldownSleepMin 15
```

---

## 八、协作规矩

1. **一个场景只留一份代码**，不要新建目录；改动直接改现有文件
2. 改完**连跑 3 局**，三次同分才算稳定；跑一次不算数
3. 成绩写回 `docs/PROGRESS.md` 和 `docs/results.csv`
4. 一轮只改一处、只跑一次，把日志给人看，人来定下一步
5. 加速度以**仿真右上角提示**为准，不要只信自己算的差分
6. 提交前确认：`git pull --rebase` → 无冲突 → `push`

---

## 九、已知坑（排错先看这里）

| 现象 | 原因 | 解决 |
|---|---|---|
| `ModuleNotFoundError: metacar` | 用了系统 python | 用 `.venv\Scripts\python.exe` |
| 一直「等待算法连接」 | 旧脚本占着 5061 | 杀掉所有相关 python 进程 |
| 「场景文件不存在」 | 场景包多一层目录 | `python -u tools\fix_scene_dirs.py` |
| 点「启动场景」启错场景 | 列表滚动位置变了 | 先滚回列表顶部；泊车类要切第 2 页 |
| PowerShell 脚本中文乱码 | 文件丢 BOM | 存成 UTF-8 with BOM |
| 分数 100 不是 110 | 有 1 次加速度违规 | 调 `--nobrake` / `--offset`，见 PROGRESS |
| 车原地不动 | 脚本第一帧就崩了 | 看 `logs/*.err` |
| 车打转 | 转向符号反了 | 见第三节符号约定 |
| 倒车飙到 6 m/s | 恒定油门 | 改速度闭环，油门只用 0.10 / 0 |
