# 场景元素 18 · 十字路口通行（200 分） / 19 · 无保护左转（200 分）

日期：2026-10-24
实现：`metacar_race/control/route_follower.py`（路口场景用 `--no-avoid`）
入口：`templates/scenes/zyyxnjydhcg/shizi_run.py`、`zuozhuan_run.py`

## 一、平台成绩

| 场景 | 开始 | 结束 | 得分 |
|---|---|---|---|
| 十字路口通行 | 2026/10/24 11:21:02 | 11:21:23 | **200.00** |
| 无保护左转 | 2026/10/24 11:27:32 | 11:27:55 | **200.00** |

## 二、做法

两个场景都有一条完整的 `static.route`（含转向），直接做纯跟踪：

```
参考路径 = static.route
速度     = min(巡航, √(2a·剩余))
横向控制 = 纯跟踪，前视 1.5+0.45v
避让     = 关闭（路口横移会压线）
终点     = 到点轻刹 + 保持
```

无保护左转的路线长 79.9 m（81 点），十字路口路线也在同一量级，两次都是 20~23 s 跑完。

## 三、注意

- 记录页有**延迟渲染**：刚跑完时「得分」列是空的，刷新一次（F5）才出现，读分脚本要留 20 s 以上；
- 200 分档场景两次实测都是正好 200.00，没有 +10（与丁字路口、上下坡驻车一致）。

## 四、复现

```bat
.venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\shizi_run.py
.venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\zuozhuan_run.py
```
