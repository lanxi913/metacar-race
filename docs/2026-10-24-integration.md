# 2026-10-24 下午：仓库整合结果（第二轮，全部 PR 已合并）

## 结论

**17 个 PR 全部合并进 main，0 个遗留。** `metacar_race/config.py` 冒烟测试通过（130 个常量可读）。

## 合并清单

| PR | 内容 | 成绩 |
|---|---|---|
| #10 #12 | 车辆直行（重复，已关闭） | 110 |
| #11 | 虚拟环境 AI 对话知识库 | 文档 |
| #15 | 垂直泊车（重复，已关闭） | 210 |
| #16 | 停止 / 直角转弯正式模板 + 标定文档 | 110 |
| #17 | 限速行驶（参数现场读） | 110 |
| #18 | 车辆变道（S 弯 + Stanley） | 110 |
| #19 | 平行车道泊车（几何推导） | 210 ×5 |
| #20 | 车辆跟随 + 紧急停止正式方案 | 待现场验 |
| #21 | 环岛驶入驶出（Stanley） | 110 ×2 |
| #22 | 第 2 页场景文档 + geo_follower | 文档/代码 |
| #23 | 丁字路口（矢量加速度预算） | 210 ×2 |
| #24 | 环形转弯自适应 | 110 |
| #25 | 连续转弯（终点提前减速 + 方向盘限速） | 110 |
| #26 | 连续场景 1 实战笔记 | 文档 |
| #27 | 十字路口（读 SDK 停止线与信号灯） | 210 ×3 |
| #28 | AGENT_PLAYBOOK 参赛总纲 | 文档 |
| #29 | 左侧车道超车 + 辅路驶入驶出 | 110 / 110 |
| #30 | 连续场景 1 成绩同步 | 630 |
| #31 | 无保护左转（自写让行判断） | 210 ×2 |

## 冲突处理记录

全部冲突都集中在两类文件：

| 文件 | 处理方式 |
|---|---|
| `metacar_race/config.py` | 两边都是追加常量块，**并集保留** |
| `docs/06-scoreboard.md`、`AGENTS.md`、`README.md` | **并集保留** |
| `speed/main.py` | 同一函数的两种等价写法，取 **已验证 110 的 PR 侧** |
| `tools/read_score.ps1` | add/add 冲突，保留 **main 侧**（工具类不轻易换） |

操作要点：`$env:GIT_EDITOR='true'` 否则 rebase 会卡在编辑器；解决后 `git add` → `rebase --continue` → `push -f` → `gh pr merge`。

## 最终结构

```
场景目录：crossroad/ lanechange/ leftturn/ parking/ roundabout/ speed/ straight_full/ tjunction/
控制库：  metacar_race/control/  15 个模块（follow/emergency_stop/geo_follower/lane_follower/
          parallel_parking/parking/reverse_park/ring_turn/route_follower/stop/vertical_park/tracking/pid/profile）
模板：    templates/{parking,rightturn,scenes,stop}
工具：    tools/（循迹 track_bonus、停止 stop_run、垂直泊车 park_v8、夜跑 night_run 等）
文档：    docs/PROGRESS.md（唯一进度入口）
```

## 已清理

- 关闭重复 PR：#10 #12（车辆直行）、#15（垂直泊车）
- 删除分支：`feat/control-tracking`、`feat/parking`，以及合并后自动删除的功能分支
- 删除 12 个被取代的实验脚本：`park_v5~v10`、`park_once`、`park_reverse`、`park_arc`、`plan_parking`、`track_run`、`track_coast`、`track_soft`

## 待办

1. `control/follow.py`、`control/emergency_stop.py` 尚未现场跑分验证（代码已合并）
2. `templates/scenes/` 下的连续场景跑法需要按最新 main 回归一遍
3. 异形泊车（车位斜 21.3°，需三点掉头）、斜坡泊车（平行车位 + 15° 坡，需 Reeds-Shepp）仍未做
