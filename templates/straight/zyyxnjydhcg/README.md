# 车辆直行 · 基线（目标 110）

## 怎么跑（三条命令）

```bat
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe -u templates\straight\zyyxnjydhcg\run.py
:: 然后去客户端点「启动场景」（顺序不能反）
```

跑分（自动读分，需要 Edge 先打开 `https://www.i-mylab.com/icv/history.html?id=straight`）：

```bat
powershell -File tools\bench.ps1 -Runs 5 -PythonArgs "templates\straight\zyyxnjydhcg\run.py"
```

## 参数

| 参数 | 默认 | 含义 |
|---|---:|---|
| `--throttle` | 0.5 | 定油门。**<0.45 会卡 17.2 km/h**（低挡位不升挡）；>0.6 起步加速度可能超 5 m/s² 丢 10 分 |
| `--lookahead` | 6.0 | 纯跟踪前视距离（m）。直行给大一点更稳，太小会左右摆 |
| `--max-seconds` | 100 | 保护上限，对应元素题时限 |

## 为什么这样写

- **定油门 0.5，不是速度闭环**：闭环要顶油门升挡，起飞段加速度会超 5 m/s²，实测只有 100 分；定油门稳定 110（任务 100 + 加速度 10）。
- 转向用仓库里的 `PurePursuit`：场景 yaw 顺时针为正，类里已经做了取反，别在外面再翻一次。

## 踩过的坑

1. 先开场景后跑脚本 → 一直「加载中」。必须先脚本、后场景。
2. 启动场景后约 7 s 车才动，脚本别提前杀。
3. 上一轮残留的 python 占着 5061/5063，新脚本会起不来 → 用 `tools/one_run.ps1` 或 `bench.ps1`，它们会先清场。

## 成绩

见 `logs/bench.csv`（`run,started,ended,score,log`）。
