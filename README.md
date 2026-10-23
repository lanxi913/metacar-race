# metacar-race · 2026 智能无人系统算法挑战赛

元动力仿真平台的自动驾驶 / VLA / 多车协同算法工程。五人小队共用一套代码、一套环境、一个进度看板。

- 比赛时间：**2026-11-01**（报名 10-25 截止）
- 平台：<https://www.i-mylab.com/icv/>
- 本仓库：算法代码 + 环境文档 + 任务看板

---

## 1. 新队友 15 分钟上手

```bat
git clone https://github.com/lanxi913/metacar-race.git
cd metacar-race

python -m venv .venv
.venv\Scripts\activate
python -m pip install -U pip
python -m pip install -r requirements.txt

.venv\Scripts\python.exe check_env.py
```

check_env 三项全绿后，按「**先脚本、后场景**」跑一次：

```bat
:: 终端 A：先跑脚本，它会卡在 connect 等待
.venv\Scripts\python.exe -u main.py

:: 然后在客户端点「启动场景」，终端开始刷状态即成功
```

更细的步骤（含客户端安装、场景下载、目录搬到 D 盘、端口说明、报错对照表）见
[docs/01-environment.md](docs/01-environment.md)。

---

## 2. 目录结构

```
metacar-race/
├─ main.py                 单车入口（连接 → 读状态 → 发控制）
├─ check_env.py            环境自检
├─ requirements.txt        metacar==0.4.0 / metacar-multiagent==0.1.0
├─ metacar_race/           可复用模块（每人认领一个子包，减少冲突）
│  ├─ config.py            全局常量（限速、端口、超时）
│  ├─ control/             规划控制：纯跟踪、PID、泊车
│  ├─ perception/          感知与 VLA：OCR、编号识别、文本生成
│  └─ multiagent/          港口迷雾：四车状态机
├─ docs/                   环境、规则、协作、路线图
└─ tools/                  一键脚本
```

**分工原则**：谁负责哪块，就只改自己那个子包；`main.py` 只有一个人动。

---

## 3. 五人分工

| 角色 | 负责 | 主要产出 |
|---|---|---|
| A 工程/平台 | 环境、入口脚本、日志、打包、录制 | `main.py`、`check_env.py`、可复现环境 |
| B 规划控制 | 循迹、速度环、泊车 | `metacar_race/control/` |
| C 感知/VLA | OCR、编号识别、三要素文本 | `metacar_race/perception/` |
| D 多车协同 | 港口迷雾状态机、路口与泊位 | `metacar_race/multiagent/` |
| E 测试/记录 | 成绩归档、复盘、规则核对、文档 | 成绩表、`docs/` |

> 三人队：把 A 和 E 合并，C 并入 B。

---

## 4. 进度看板

任务全在 Issues，用 Label 过滤：

| 看板 | 链接 |
|---|---|
| 全部任务 | <https://github.com/lanxi913/metacar-race/issues> |
| P0 今天必做 | <https://github.com/lanxi913/metacar-race/issues?q=is%3Aissue+is%3Aopen+label%3AP0> |
| 我的任务 | <https://github.com/lanxi913/metacar-race/issues/assigned/lanxi913> |
| 环境类 | <https://github.com/lanxi913/metacar-race/issues?q=is%3Aissue+is%3Aopen+label%3Aenv> |

规则：

- 每人每天只认领 1 个 `P0`/`P1`，做完关掉再领
- 认领 = 把 Issue assign 给自己，避免撞车
- 每日站会三问：今天拿了几分？明天攻哪个？现在卡在哪？

---

## 5. 关键约定

1. **不直接推 main**：开 `feat/xxx` 分支 → PR → 至少一人 review → 合并。
   详见 [docs/03-workflow.md](docs/03-workflow.md)。
2. **不提交场景包、不提交 `.venv`**：已在 `.gitignore` 里。
3. **坐标、验证码、object_id 一律不许写死**：正式赛题是训练题的泛化版。
4. **提交前跑 `check_env.py` 和你自己的脚本**，能跑通再提 PR。

---

## 6. 正式场景模板

| 场景 | 入口 | 最近验收 |
|---|---|---|
| 车辆停止 | `templates/stop/chang-kou/run.py` | 连续 3 次 110 |
| 直角转弯 | `templates/rightturn/chang-kou/run.py` | 连续 3 次 110 |

两个入口都遵循“先运行脚本、后在客户端启动场景”。参数说明和标定记录见
[templates/README.md](templates/README.md)、[docs/stop-calibration.md](docs/stop-calibration.md)
和 [docs/turn-calibration.md](docs/turn-calibration.md)。

---

## 7. 实测跑分（车辆直行，满分 110）

| 跑法 | 用时 | 得分 | 说明 |
|---|---|---|---|
| 只刹停不动 | 100 s | 0 | 超时 |
| 恒油门 0.5 + 纯跟踪（`--cruise 0.5`） | 10.3 s | **110** | 100 到达 + 10 加速度加分 |
| 速度环闭环（默认 `--track`） | 11.0 s | 100 | 到达分拿到，加速度加分丢了 |
| 恒油门 1.0 | 2.7 s | 90 | 超速（109 km/h）扣 10 |

**结论**：元素题用 `--cruise 0.5`，最简控制反而拿满 110。速度环留给需要限速/长距离的场景。

**连续验证（2026-10-22 晚）**：同一套 --cruise 0.5 连跑 5 次，训练记录依次是

`
21:18:06  110.00
21:25:43  110.00
21:29:47  110.00
21:30:43  110.00
21:31:20  110.00
`

5/5 满分，每次约 10 秒（时限 100 秒）。批量跑用 	ools/one_run.ps1。

## 8. 常见坑（先看这里）

| 症状 | 原因 | 处理 |
|---|---|---|
| 场景一直「加载中」 | 顺序反了 | 先跑脚本（卡在 connect），再点启动 |
| 黑屏 1–2 秒退出 | 显卡不够 | 换独显机器 |
| `ModuleNotFoundError: metacar` | 解释器不对 | 用 `.venv\Scripts\python.exe`，别用系统 python |
| 速度卡在 **17.2 km/h** | 卡在低挡位 | 油门给到 1.0 顶到升挡；详见 docs/01 |
| 脚本跑了场景没反应 | 程序已异常退出 | 先看终端最后 20 行 |
| 连续场景被**拉回元素起点** | 元素超时自动重开（如连续转弯 857.8 m / 200 s） | 均速要够，详见 docs/08 |
| `used_time` 忽大忽小 | 它是元素内计时，切元素/重开都归零 | 全局时钟自己累加，详见 docs/08 |
| 垂直泊车**突然 0 分** | 车身压出库边线（横向差 0.2 m 就翻车） | 看 `slot_dv`/`slot_margin`，详见 docs/09 |
| 想消加速度尖峰 | 倒车速度剖面就是停车几何 | **别改倒车段**，详见 docs/09 |

---

## 9. 相关资料

- [环境搭建（可复现）](docs/01-environment.md)
- [赛题规则速查](docs/02-rules.md)
- [协作流程](docs/03-workflow.md)
- [10 天路线图](docs/04-roadmap.md)
- [连续场景1：元素、时限与跑法](docs/08-continuous-scene.md)
- [连续场景1：加速度奖励的实测与取舍（630 分怎么来的）](docs/09-continuous-scene-parking-accel.md)
- 参赛作战手册：`docs/2026智能无人系统算法挑战赛-参赛作战手册.docx`
