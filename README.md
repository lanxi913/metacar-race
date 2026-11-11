# metacar-race · 2026 智能无人系统算法挑战赛

> 元动力仿真平台（i-mylab）的自动驾驶 / VLA / 多车协同算法工程。
> 五人小队 · 十一天 · 407 次提交 · **决赛三等奖**

- 比赛：**2026-11-01**（报名 10-25 截止）
- 平台：<https://www.i-mylab.com/icv/>
- 本仓库：算法代码 + 逐场景实测记录 + 踩坑文档

---

## 一、奖项与战绩

**2026 智能无人系统算法挑战赛 · 决赛三等奖**

正式赛题 6 个，取各场景最好一局，平台 run-records 的真实记录：

| 场景 | 满分 | 最好成绩 | run | 说明 |
|---|---|---|---|---|
| 连续场景1 | 650 | **650.00** | 84014 | 5 个元素全拿加速度奖励，314 s |
| 连续场景4 | 1600 | **1400.00** | 84276 | 平行泊车元素没判完成 |
| VLA 文本导航与规范停车 | 1000 | **1000.00** | 84080 | 一把停进 PARKING_SLOT |
| 港口迷雾 | 1000 | **750.00** | 83954 | 8 项成果全成，扣 250 |
| 停车区违规检测 | 1000 | **750.00** | 84408 | 盲报全集，F1 = 0.75 |
| 功能区合规检测 | 1000 | **333.00** | 84415 | 盲报全集，F1 = 0.333 |
| **合计** | **6250** | **4883.33** | — | 缺口 1366.67 |

能把几何和控制律算清的，基本都拿到了；需要感知数据通道的，只能靠盲报；有一题从头到尾没有实现。
详细的逐元素拆解见 [docs/PROGRESS.md](docs/PROGRESS.md)。

---

## 二、心路历程

### 时间线

| 日期 | 提交 | 发生了什么 |
|---|---|---|
| 10-22 | 9 | 建仓。装环境、跑通第一个直行，把注意事项写成 `AGENTS.md` |
| 10-23 | 77 | 元素题批量爆破：直行、停止、限速、变道、环岛一次过 |
| 10-24 | 199 | 通宵。第 2 页十二个场景扫完，异形泊车 210 满分，写下打法总纲 |
| 10-26 | 9 | 反推出完整算分公式；直接跑队友 PR，一次复现连续场景4 的 1600 |
| 10-27 | 12 | VLA 文本导航 1000 满分；把起步瞬态定性成物理上没解 |
| 10-31 | 28 | 港口迷雾 8/8 定版；赛题场景预下载、启动通道打通 |
| 11-01 | 24 | 比赛日 09:30–11:30，六个赛题全部要在窗口内出分 |
| 11-11 | 3 | 仓库公开前的清理与收尾 |

### 三个真正改变打法的认知

**1. 最简控制反而赢。** 直行题用恒定油门 0.5 开环跑，110 连拿五次；
换成精心调过的速度闭环，只有 100。原因不是闭环写得差，而是闭环一定有油门动作，
有动作就可能碰到 `|a| ≤ 5 m/s²` 的门限，那 10 分奖励就没了。

**2. 加速度是矢量，而且平台按窗口平均判。** `√(a_lon² + a_lat²) ≤ 5`——
半径 1.85 m 的弯，4.78 m/s 会产生 12.3 m/s² 的向心加速度，必须降到 2.72 m/s。
更狠的是起步那一瞬：一帧里速度从 0 跳到 0.67 m/s，帧级加速度 +9.6 m/s²，
油门压到 0.004 也压不住，这是平台侧的物理瞬态，在当前规则下无解。

**3. 1600 分是架构差距，不是参数差距。** 自己的单套 PI 环加弯道刹车，天花板卡在 1500，
怎么调都上不去；把队友分支用 worktree 拉下来、原参数一跑就是 1600。
他每个元素都有专用控制器、纵向收油滑行、全程不踩刹车、只升一次挡。

### 没做到的部分

连续场景4 差 200 分：赛题场景在车位后方 1.1 m 处加了阻挡，倒车三次都卡住，元素判不了完成，
关窗前最后一局用 PR #50 原版参数仍是 1400。两个 VLA 检测题没有数据通道也没有实现，
只能按 F1 最优解盲报全集（0.75 / 0.333）。

后来查排行榜才发现：三个 VLA 场景都有人拿过 1000——
不是不能做，是把时间全投在了自己擅长的几何与控制上。

---

## 三、仓库结构

```
metacar-race/
├─ main.py                     单车入口：连接 → 读状态 → 发控制
├─ check_env.py                环境自检（三项全绿再跑场景）
├─ requirements.txt            metacar==0.4.0 / metacar-multiagent==0.1.0
├─ metacar_race/               可复用模块
│  ├─ config.py                全局常量（限速、端口、超时）
│  ├─ control/                 规划控制：纯跟踪、Stanley、PID、速度剖面、各类泊车
│  ├─ perception/              OCR 与 VLA 文本解析
│  └─ multiagent/              港口迷雾：多车状态机
├─ straight_full/              元素场景独立入口，一个场景一个目录、一个 main.py
│  parking/ crossroad/ tjunction/ leftturn/ roundabout/ lanechange/ speed/
├─ templates/                  按人归档的场景模板（谁负责的场景放谁的目录）
├─ tools/                      跑分、探针、批量脚本（见下表）
├─ docs/                       文档库：40 篇笔记 + 12 张实测证据图
│  ├─ PROGRESS.md              唯一入口：进度、分数、变更记录
│  ├─ 15-scene-playbook.md     从满分场景提炼的打法总纲
│  ├─ 14-angled-parking-findings.md  异形泊车 210 的完整推导
│  └─ images/                  12 张实测证据图
└─ .github/                    Issue 模板
```

**两条线，别看错分支：**

| 分支 | 内容 |
|---|---|
| `main`（默认） | 元素场景线 + 全部文档，就是你现在看到的这些 |
| `feat/competition-ready` | 比赛日实际跑的代码：连续场景1/4、VLA 文本导航、港口迷雾，以及一键跑分 `tools/run_scene.ps1`、查分 `tools/last_run.py` |

**tools/ 里最常用的几个**

| 脚本 | 用途 |
|---|---|
| `tools/run_scene.ps1` | 一键跑一局（清场 → 起脚本 → 点启动 → 等 → 结算等待），仅 `feat/competition-ready` |
| `tools/last_run.py` | 直接读平台 API 取分，别信自己打的日志 |
| `park_angled6.py` / `park_v8.py` | 异形泊车 / 垂直泊车的满分版 |
| `probe_*.py` | 各项标定探针：油门-加速度映射、转向、制动、循迹精度 |
| `capture_window.ps1` | 按窗口句柄抓图，不受前台焦点影响 |

**五人分工**（仓库按子包切，减少冲突）

| 角色 | 负责 | 主要产出 |
|---|---|---|
| A 工程/平台 | 环境、入口脚本、日志、打包 | `main.py`、`check_env.py`、可复现环境 |
| B 规划控制 | 循迹、速度环、泊车 | `metacar_race/control/` |
| C 感知/VLA | OCR、编号识别、三要素文本 | `metacar_race/perception/` |
| D 多车协同 | 港口迷雾状态机、路口与泊位 | `metacar_race/multiagent/` |
| E 测试/记录 | 成绩归档、复盘、规则核对 | 成绩表、`docs/` |

---

## 四、快速开始

```bat
git clone https://github.com/lanxi913/metacar-race.git
cd metacar-race

python -m venv .venv
.venv\Scripts\activate
python -m pip install -U pip
python -m pip install -r requirements.txt

.venv\Scripts\python.exe check_env.py
```

`check_env` 三项全绿后，按「**先脚本、后场景**」跑一局：

```bat
:: 终端 A：先跑脚本，它会卡在 connect 等待，这是正常的
.venv\Scripts\python.exe -u main.py

:: 然后在元动力客户端点「启动场景」，终端开始刷状态即成功
```

客户端安装、场景下载、目录搬到 D 盘、端口说明、报错对照表见
[docs/01-environment.md](docs/01-environment.md)。

想跑比赛日那六题，先切分支：

```bat
git switch feat/competition-ready
.venv\Scripts\python.exe tools\last_run.py ius_lianxu1 1 --det   # 查分
```

---

## 五、几条最容易丢分的坑

| 症状 | 原因 | 处理 |
|---|---|---|
| 场景一直「加载中」 | 顺序反了 | 先跑脚本（卡在 connect），再点启动 |
| `ModuleNotFoundError: metacar` | 解释器不对 | 用 `.venv\Scripts\python.exe`，别用系统 python |
| 速度卡在 **17.2 km/h** | 卡在低挡位 | 油门给到 1.0 顶到升挡，详见 [docs/01](docs/01-environment.md) |
| 连续场景被**拉回元素起点** | 元素超时自动重开（如连续转弯 857.8 m / 200 s） | 均速要够，详见 [docs/08](docs/08-continuous-scene.md) |
| 垂直泊车**突然 0 分** | 车身压出库边线（横向差 0.2 m 就翻车） | 看 `slot_dv` / `slot_margin`，详见 [docs/09](docs/09-continuous-scene-parking-accel.md) |
| 想消加速度尖峰 | 倒车速度剖面就是停车几何 | **别改倒车段**，详见 [docs/09](docs/09-continuous-scene-parking-accel.md) |
| 跑完没记分 | 脚本一退出就关场景，平台来不及结算 | 加 20~25 s 结算等待（`-SettleSeconds 25`） |

---

## 六、文档索引

| 文档 | 内容 |
|---|---|
| [docs/PROGRESS.md](docs/PROGRESS.md) | 唯一入口：进度、分数、逐日变更记录（含算分公式反推） |
| [docs/15-scene-playbook.md](docs/15-scene-playbook.md) | 打法总纲：速度层三种打法、横向三套控制、换挡与停车 |
| [docs/14-angled-parking-findings.md](docs/14-angled-parking-findings.md) | 异形车道泊车 210 的完整推导 |
| [docs/08-continuous-scene.md](docs/08-continuous-scene.md) | 连续场景1：元素、时限与跑法 |
| [docs/02-rules.md](docs/02-rules.md) | 赛题规则速查 |
| [docs/01-environment.md](docs/01-environment.md) | 环境搭建（可复现） |
| [docs/03-workflow.md](docs/03-workflow.md) | 协作流程：分支、PR、记分 |
| [AGENT_PLAYBOOK.md](AGENT_PLAYBOOK.md) | 给 AI 助手的第一读物：怎么跑、怎么记分、哪些坑不许再踩 |

---

## 七、许可

MIT License，见 [LICENSE](LICENSE)。代码可自由复用、修改、再分发，保留版权声明即可。
如果这几篇踩坑记录帮你省了几天，那这个仓库就值了。
