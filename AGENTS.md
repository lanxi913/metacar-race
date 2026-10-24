# AGENTS.md — 给 AI 助手的项目说明

> 本文件面向 AI 编码助手（Codex / Claude Code / Cursor / Copilot 等）。
> 人类队友看 [README.md](README.md) 和 `docs/`。
> 你在这个仓库里做任何改动前，先读完本文件。

---

## 1. 项目是什么

2026 智能无人系统算法挑战赛的参赛工程。用 Python 控制「元动力（MetaCar）」仿真平台里的车辆，
完成三类赛题：城市道路、VLA（读牌 + 文本提交）、港口迷雾（四车协同）。

- 比赛：2026-11-01，报名 2026-10-25 截止
- 平台：<https://www.i-mylab.com/icv/>
- 语言：**中文注释与文档**，代码标识符用英文
- Python：**3.12.4**（必须 ≥3.10，SDK 硬要求）

---

## 2. 环境（已存在，不要重建）

| 项 | 值 |
|---|---|
| 解释器 | `<repo>/.venv/Scripts/python.exe` |
| 依赖 | `metacar==0.4.0`、`metacar-multiagent==0.1.0`（PyPI 上有，勿改版本） |
| 场景包 | `D:\元动力场景`（用户目录下的 `car` 是指向它的 junction） |

**运行任何脚本都必须用 `.venv/Scripts/python.exe`**，不要用系统 `python`，不要 `pip install` 到全局。

```powershell
# 安装依赖（只在 .venv 缺失时做）
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt

# 自检
.venv\Scripts\python.exe check_env.py
```

---

## 3. 怎么跑（顺序是硬性的）

`metacar.SceneAPI` 是**服务端**：脚本监听 `127.0.0.1:5061`（JSON）和 `5063`（视频），
场景（Unity 进程）主动连上来。所以：

```powershell
# 1) 先在终端跑脚本，它会阻塞在 connect
.venv\Scripts\python.exe -u main.py

# 2) 再去客户端点「启动场景」
# 3) 终端开始刷 [xx.xs] pos=... v=... 即成功
```

反了就永远「加载中」。多车场景用 `ScenarioTaskAPI(port=9100)`。

---

## 4. 代码约定

- 入口 `main.py` 只由 A（工程）改；其他人改自己子包：
  - `metacar_race/control/` → B（循迹、PID、泊车）
  - `metacar_race/perception/` → C（OCR、编号识别、文本拆分）
  - `metacar_race/multiagent/` → D（四车状态机）
  - `docs/` → E
- 所有可调参数放 `metacar_race/config.py`，不要散落魔法数字。
- **禁止写死**场景坐标、验证码、`object_id`、泊位号：正式赛题是训练题的泛化版，
  这些必须从 `get_scene_static_data()` / `get_observation()` / `get_task_state()` 读。
- 每个控制循环都必须发控制指令（港口迷雾 1 秒心跳，超时安全停车）。
- **速度闭环不许用"油门/刹车反复开关"来凑**：需要频繁增减油门的地方，按 PID（带积分抗饱和 +
  输出变化率限制）或真实驾驶逻辑（先松油门滑行、再轻刹、最后补刹）设计，抑制加速度尖峰。
  平台对 `|a| ≤ 5 m/s²` 每元素加 10 分，而反复开关油门/刹车正是尖峰的来源
  （实测：3 m/s 门槛 + 0.15 刹车与加速段形成极限环，均速只有 2.8 m/s；加速度到 12 m/s² 时直接丢分）。
- 退出前 `brake=1.0, gear=PARKING` 刹停。
- 新增依赖要更新 `requirements.txt` 并在 PR 里说明理由。

---

## 5. 评分规则关键数字（写算法时会用到）

| 场景 | 限速 | 时限 | 要点 |
|---|---|---|---|
| 场景元素 | 60 km/h | 100 s / 200 s | 到达终点即满分；`|a|≤5 m/s²` 每题 +10；压线 −10/−20；碰撞 −5×v |
| 连续场景 | 60 km/h | 基准 180–810 s | 时间附加分 = `0.1×(基准−实际)`，权重很低，**别为抢时间压线** |
| VLA | 30 km/h | 100 s/任务 | 文本 400 + 完成 600；违规检测是 F1；标点用英文、不换行 |
| 港口迷雾 | 20 km/h | 600 s | 8 项成果 240+160+200+400；三条一票归零 |

港口迷雾硬阈值：观测 30 m / 前方 120°；读码 7 m；扫描 10 m；路口申请 12 m；
互斥区 6 m；泊位 3 m；观测凭证 30 s 有效。

**实测结论：油门 0.5 时速度卡在 17.2 km/h（低挡位），给到 1.0 才会升挡继续加速。**

---

## 6. 提交与协作

- 不直接改 `main`：建 `feat/*` 分支 → PR → 至少 1 人 review。
- 提交信息：`feat|fix|docs|chore(scope): 说明`。
- 不提交：`.venv/`、`__pycache__/`、场景包、`*.exe`、录屏（`.gitignore` 已挡）。
- 改完必须本地验证：`check_env.py` 通过 + 你的脚本能跑通。

---

## 7. 排错（AI 优先看这张表）

| 现象 | 原因 | 动作 |
|---|---|---|
| 一直「加载中」 | 先开场景后跑脚本 | 关场景 → 跑脚本 → 再开场景 |
| `ModuleNotFoundError: metacar` | 用了系统 python | 换 `.venv\Scripts\python.exe` |
| 端口 5061/5063 被占用 | 上次脚本没退干净 | 结束残留 `python.exe` |
| 速度卡 17.2 km/h | 低挡位 | 油门给 1.0 等升挡 |
| `granted=False` | 距离/角色/阶段不满足 | 读 `result.message`，对照阈值表 |
| 场景中途退出 | 超时或触发归零 | 查该场景时限与三条红线 |

---

## 8. 你不该做的事

- 不要重新发明环境（别装 conda 环境、别升级 SDK、别改 `requirements.txt` 版本）。
- 不要在无场景运行时反复重试 `connect()`——它本来就会一直阻塞。
- 不要提交二进制大文件。
- 不要把训练用的固定坐标/验证码写进代码。

---

## 进度看这里（AI 必读）

**docs/PROGRESS.md 是本项目唯一权威的进度文件**：成绩总表、每个场景该用哪份代码、奖励分（+10）的判定口径、代码地图、排错表。

改代码前先读它；跑完记得把成绩写回去。其他文档都是细节补充。
## 9. 参数抽象与情境自适应

- 新场景优先从 `get_scene_static_data()`、`get_observation()` 和
  `get_task_state()` 推导目标速度、距离、时间窗口和安全阈值。
- 减少只为单一训练场景成立的常量。场景特定数值只能用于探针或临时标定，
  验证后应改写成由相对距离、速度、车头距、剩余时间、道路宽度等状态推导。
- 一律不得写死障碍物 `object_id`、固定坐标、验证码或泊位号。
- 已通过验证的控制流程不因参数自适应而改变。调整时先保留原有状态机和控制顺序，
  只增加输入特征及参数映射；行为变化必须单独验证。
- 每个可调参数都必须有单位、物理含义、来源和适用情境。无法说明来源的参数不能进入正式实现。

---

## 10. 新元素与每轮仿真的工作流（硬性）

### 10.1 遇到新元素：先找已有实现，再考虑自己写

按这个顺序查，查到就用，不要重复实现：

```powershell
git log --all --oneline --grep="<元素名>"      # 谁做过
git branch -r --contains <commit>              # 在哪个分支（可能还没合并到 main）
git show <分支>:<文件>                          # 直接把实现取出来看
Select-String -Path metacar_race\control\*.py -Pattern "<关键词>"
```

现成可借的模块（截至 2026-10-24）：

| 元素 | 模块 | 实测 |
|---|---|---|
| 几何循迹 + 换道避障 | `metacar_race/control/geo_follower.py` | 复杂路况超车 **200** |
| 路线循迹（信号灯、停线） | `metacar_race/control/route_follower.py` | 丁字/十字/左转/行人 200 |
| 车道跟随 | `metacar_race/control/lane_follower.py` | 左侧超车 110、辅路 110 |
| 垂直泊车（倒车入库） | `metacar_race/control/vertical_park.py`、`reverse_park.py` | 连续场景1 泊车 200 |
| 平行泊车 | `metacar_race/control/parallel_parking.py` | 见 PR #19 |
| 跟随 / 紧急停止 / 环岛 / 破静摩擦 | `follow.py`、`emergency_stop.py`、`ring_turn.py`、`pid.py` | 见各 docs |

借用时在 PR 描述里写清「继承自哪个分支/文件」，并在对应 `docs/` 补一句来源。
只有确实没有可复用实现（或实测不达标）时，才新写，并在 PR 里说明为什么现有模块不能用。

### 10.2 每跑完一局仿真：先日志，再方案，批准后才改代码

1. **问题日志**——跑完立刻给，不要先动代码。按阶段（元素/环节）列：现象、证据
   （`sim_t` / 帧号 / 坐标 / 速度 / 油门 / 最近障碍距离）、影响（丢多少分、卡多久）。
2. **改正方案**——每条问题给一个改法：改哪个文件、改哪个判据、预期效果、风险。
3. **等确认**——用户明确同意后才改代码；改完先 `py_compile` + 冒烟测试，再准备下一轮仿真。
4. **一局只改一类问题**：多处改动塞进同一次调试，出了偏差就分不清是哪条改动的功劳。
