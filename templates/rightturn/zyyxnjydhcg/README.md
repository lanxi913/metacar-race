# 直角转弯 · 基线（目标 110）

## 怎么跑

```bat
.venv\Scripts\python.exe -u templates\rightturn\zyyxnjydhcg\run.py
:: 然后客户端点「启动场景」（直角转弯）
```

跑分：`powershell -File tools\bench.ps1 -Runs 5 -PythonArgs "templates\rightturn\zyyxnjydhcg\run.py"`，
Edge 先开 `https://www.i-mylab.com/icv/history.html?id=rightangleturn`。

## 参数

| 参数 | 默认 | 含义 |
|---|---:|---|
| `--throttle` | 0.01 | 定油门，稳态约 4.78 m/s，加速度分保得住 |
| `--lookahead` | 3.0 | **关键参数**。题目的弯道曲率是 3 m，前视必须收到 3 m |

## 为什么前视必须 3 m

车辆停止题用 6 m 前视很稳，但拿到直角弯上会**切内道**：6 m 前视在 3 m 曲率下几乎等于直接抄近路，实测压边线 −10/−20。
收到 3.0 m 后三次跑完无压线、都是 110。

## 为什么不主动停车

在终点前主动驻车会让平台迟迟不触发「到达终点」判定（直角弯是「到达即满分」）。
所以这里跟 `stop` 模板相反：**一直保持前进**，交给平台收尾，只在最后 1.5 m 把方向盘回正。

## 踩过的坑

1. 前视给大了压线（见上）。
2. 终点前主动刹停 → 平台不判完成，白跑。
3. 和直行一样，先脚本后场景；场景起来后约 7 s 车才动。
