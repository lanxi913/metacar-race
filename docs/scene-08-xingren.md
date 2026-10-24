# 场景元素 20 · 有行人右转（200 分）

日期：2026-10-24
实现：`metacar_race/control/route_follower.py`（新增"行人减速/停车"）
入口：`templates/scenes/zyyxnjydhcg/xingren_run.py`

## 一、平台成绩

| 开始 | 结束 | 得分 |
|---|---|---|
| 2026/10/24 11:33:42 | 11:34:00 | **200.00** |

## 二、场景要素

```
route 57 点，总长 55 m，终点 (-32.9, 5.4)
障碍物 5 个：PEDESTRIAN ×1 (-5.10, 18.14) + STATIC ×4（路口四个角的隔离桩）
自车 (-5.59, 39.27) yaw=90，沿 -Y 南下后右转向西
```

## 三、做法

1. 路线循迹 + 右转；
2. **行人处理**：`obstacles` 里 type=PEDESTRIAN 且在本车前方 12 m、横向 2 m 内 → 目标速度压到 `0.5*(间距-5)`，必要时停车等待；
3. 不做换道避让（路口横移会压线）。

## 四、复现

```bat
.venv\Scripts\python.exe -u templates\scenes\zyyxnjydhcg\xingren_run.py
```
