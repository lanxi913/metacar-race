# 正式场景模板

本目录只收录已经定稿的场景入口。模板保留独立 `run.py` 和 `README.md`，
便于复现和后续回归。

| 场景 | 正式入口 | 最近验收 |
|---|---|---|
| 车辆停止 | `templates/stop/chang-kou/run.py` | 连续 3 次 110 |
| 直角转弯 | `templates/rightturn/chang-kou/run.py` | 连续 3 次 110 |

运行顺序固定为“先脚本、后场景”。

```powershell
# 车辆停止
.venv\Scripts\python.exe -u templates\stop\chang-kou\run.py

# 直角转弯
.venv\Scripts\python.exe -u templates\rightturn\chang-kou\run.py
```

运行脚本后再在客户端启动对应场景。不要提前结束脚本。
