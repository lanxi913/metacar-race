# 模板总说明（templates/）

> 目的：**每个人一套能拿满分的基线**。跑完对比，选效果最好的进主线；不是分任务，是分方案。

## 目录约定

```
templates/
  <模块>/<你的名字>/run.py      ← 独立入口，互不覆盖
  <模块>/<你的名字>/README.md   ← 怎么跑、参数、踩过的坑、成绩记录
```

模块固定六个：`straight` `stop` `rightturn` `parking` `vla-text` `portfog`

## 一个模板算「成立」的三条硬标准

1. **连续 5 次同一满分**，`logs/bench.csv` 可查（用 `tools/bench.ps1 -Runs 5`）
2. **三条命令能复现**：装依赖 → 先跑脚本 → 点启动场景
3. README 里写清：怎么跑、每个参数什么含义、踩过什么坑

## 跑分

```bat
:: 跑一次（不读分，只看日志）
powershell -File tools\one_run.ps1 -PythonArgs "templates\straight\zyyxnjydhcg\run.py --throttle 0.5"

:: 连跑 5 次并自动读分（需要 Edge 先打开训练记录页）
powershell -File tools\bench.ps1 -Runs 5 -PythonArgs "templates\straight\zyyxnjydhcg\run.py"
```

`bench.ps1` 读分依赖 Edge 打开 `https://www.i-mylab.com/icv/history.html?id=<场景id>`，
场景 id：`straight` / `stop` / `rightangleturn` / `parking`（按实际改）。

## 成绩上报（贴到对应 Issue，一行）

```
<名字> | <模块> | 5 次分数 | 用时 | 入口命令
例：zyyxnjydhcg | 车辆直行 | 110/110/110/110/110 | 17s | python -u templates/straight/zyyxnjydhcg/run.py
```

## 硬规矩（和各人入口无关，所有人都要守）

- 不改根目录 `main.py`（那是选完方案后的主线入口）
- 不写死坐标 / 验证码 / object_id，全部从 `get_scene_static_data()` / `get_observation()` 读
- 先脚本、后场景；脚本别提前杀（启动场景后约 7 s 车才动）
- 提交前跑一次 `check_env.py`

## 已经验证过、别重复踩的结论

| 结论 | 数据 |
|---|---|
| 车辆直行用定油门 0.5，别用速度闭环 | 闭环 100 分，定油门 110 分 |
| 油门 <0.45 卡 17.2 km/h | 低挡位不升挡 |
| yaw 顺时针为正，steering 与纯跟踪反号 | 见 `metacar_race/control/tracking.py` 注释 |
| 直角转弯前视必须收到 3.0 m | 6 m 前视会切弯压线 |
| 车辆停止：油门 0.01 / 刹车 0.10 | 加速度 ≤5，三次 110 |
| 启动场景后约 7 s 车才动 | 脚本别提前杀 |

## 当前状态

| 模块 | 模板 | 状态 |
|---|---|---|
| straight | `straight/zyyxnjydhcg` | ✅ 可跑，待 5 次验证 |
| stop | `stop/zyyxnjydhcg` | ✅ 可跑，待 5 次验证 |
| rightturn | `rightturn/zyyxnjydhcg` | ✅ 可跑，待 5 次验证 |
| parking | `parking/zyyxnjydhcg` | 🚧 倒车入库骨架，参数未标定 |
| vla-text | `vla-text/zyyxnjydhcg` | 🚧 链路骨架，OCR 未接 |
| portfog | `portfog/zyyxnjydhcg` | 🚧 心跳+状态骨架，400 分状态机未接 |
