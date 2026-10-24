# 2026-10-24 下午：仓库整合结果

## 已合并进 main（新增）

| PR | 内容 | 成绩 |
|---|---|---|
| #18 | 车辆变道（S 弯 + Stanley） | 110 |
| #19 | 平行车道泊车（几何推导） | 210 ×5 |
| #30 | 连续场景 1 成绩 + 总表同步 | 630 |
| #17 #21 #23 #24 #25 #26 #27 #28 | 限速 / 环岛 / 丁字 / 环形 / 连续转弯 / 场景笔记 / 十字 / PLAYBOOK | 110 ~ 210 |

## 仍未合并（有冲突，需作者 rebase 到最新 main）

| PR | 内容 | 成绩 | 备注 |
|---|---|---|---|
| #31 | 无保护左转 | 210 ×2 | rebase 后仍冲突 |
| #29 | 左侧车道超车 + 辅路驶入驶出 | 110 / 110 | rebase 后仍冲突 |
| #22 | 场景分页 | — | rebase 后仍冲突 |
| #20 | 车辆跟随 + 紧急停止 | 待验 | AGENTS.md / config.py 冲突，已 rebase 一次仍冲突 |
| #16 | 停止 + 直角模板 | — | 待查 |
| #11 | 虚拟环境文档 | — | 待查 |

## 已清理

- 关闭重复 PR：#10 #12（车辆直行重复）、#15（垂直泊车重复）
- 删除已合并分支：`feat/control-tracking`、`feat/parking`
- 删除 12 个被取代的实验脚本：`park_v5~v10`、`park_once`、`park_reverse`、`park_arc`、`plan_parking`、`track_run`、`track_coast`、`track_soft`

## 处理冲突的经验（给下次用）

1. `metacar_race/config.py`：两边都是在追加常量块，**并集保留**即可
2. `AGENTS.md`、`docs/*.md`：同样并集
3. `speed/main.py` 这类同一函数的两种写法：取 PR 侧（功能等价，且是已验证 110 的那版）
4. rebase 卡住多半是编辑器：`$env:GIT_EDITOR='true'` 再 `git rebase --continue`
5. 解决完 `git add` 冲突文件 → `rebase --continue` → `push -f` → `gh pr merge`
