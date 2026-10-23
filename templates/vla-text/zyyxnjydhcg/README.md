# VLA 文本导航 · 基线（目标 1000）—— 链路骨架，OCR 未接

> 状态：**能力把「连上 → 取图 → 拆三要素 → 随控制提交」跑通**，但 `read_sign()` 还是空实现，
> 现在跑只会拿到「完成分」，文本分 0。

## 怎么跑

```bat
:: 安全模式：只打印，不提交
.venv\Scripts\python.exe -u templates\vla-text\zyyxnjydhcg\run.py --dry-run

:: 正式：接上 OCR 之后用这个
.venv\Scripts\python.exe -u templates\vla-text\zyyxnjydhcg\run.py
```

## 分数构成（规则 PDF）

| 项 | 分 | 说明 |
|---|---:|---|
| 完成 | 600 | 100 s 内进 8 m × 8 m 有效区并停稳 |
| OCR 文本 | 160 | 整句识别得对不对 |
| 时间 / 地点 / 动作 | 80 + 80 + 80 | 拆出来的三段，各自按归一化编辑距离 |

## 提交格式（直接决定 400 分里的 240）

- **不换行、不加空格**、英文标点、小写
- 系统只把中文逗号/冒号转英文；**换行和全角空格不会被清**，一出现就掉分
- 三要素是**每帧随控制一起**提交的，不是最后交文件：
  `api.set_vehicle_control(vc, VLAExtensionOutput(text_info=VLATextOutput(...)))`

## 待办

1. `read_sign()`：说明牌 OCR。建议 灰度 → 二值化 → 只留中间 1/2 区域 → 模板匹配/小模型。识别不到就返回空串，别乱猜。
2. 到达判定后要停稳（8 m×8 m 有效区），目前只给 0.3 油门直行。
3. 地点短语的切分口径要按规则 PDF 附录 A.4 的示例校准。

## 踩过的坑

- 不是 VLA 场景时 `static.vla_extension` 为 False，脚本会直接退出——别拿它跑直角弯。
- 乱填文本比不填扣得更多（编辑距离按全长算）。
