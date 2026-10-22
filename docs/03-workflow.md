# 协作流程

五人共用一套代码，靠这几条规矩避免互相踩脚。

---

## 1. 分支模型

只有 `main` 一条长期分支，它必须随时可跑。

```
main                 ← 永远能跑通 check_env + 车辆直行
 └─ feat/control-tracking     B 的循迹
 └─ feat/vla-ocr              C 的读牌
 └─ feat/multiagent-signal    D 的信号机
 └─ docs/env-setup            E 的文档
```

- 命名：`feat/` 新功能、`fix/` 修 bug、`docs/` 文档、`chore/` 杂项
- 分支活不过 3 天：做完就 PR，别攒大招

---

## 2. 日常流程

```bat
:: 1. 开工前同步
git checkout main
git pull

:: 2. 开分支
git checkout -b feat/control-tracking

:: 3. 写代码，本地跑通
.venv\Scripts\python.exe check_env.py
.venv\Scripts\python.exe -u main.py

:: 4. 提交（小步、频繁）
git add -A
git commit -m "feat(control): 纯跟踪循迹，直行场景跑通"

:: 5. 推送并开 PR
git push -u origin feat/control-tracking
gh pr create --fill
```

PR 要求：

- 标题写清楚「改了什么 + 验证方式」
- 至少 1 人 review 后合并（小改动可自己合，但要在群里说一声）
- 合并后删掉分支

---

## 3. 提交信息

```
<type>(<scope>): <subject>

feat(control): 增加纯跟踪循迹
fix(vla): 修复文本换行导致相似度下降
docs(env): 补充 17km/h 挡位问题
chore(deps): 升级 metacar 到 0.4.0
```

type：`feat` `fix` `docs` `refactor` `test` `chore`

---

## 4. 文件归属（减少冲突）

| 目录/文件 | 谁能改 |
|---|---|
| `metacar_race/control/` | B |
| `metacar_race/perception/` | C |
| `metacar_race/multiagent/` | D |
| `main.py`、`check_env.py`、`metacar_race/config.py` | A（别人要改先打招呼） |
| `docs/` | E 为主，其他人可补 |

---

## 5. Issue 与站会

- 所有任务先开 Issue，用 label 标 `P0/P1/P2` 和模块名
- 认领：把自己 assign 上去，避免两个人做同一件事
- 一条 Issue 对应一个 PR，PR 描述里写 `Closes #12`
- 每天 21:00 站会三问：**今天拿了几分 / 明天攻哪个 / 卡在哪**

---

## 6. 不能提交的东西

`.gitignore` 已经挡住：`.venv/`、`__pycache__/`、`*.pyc`、`out/`、`logs/`、`*.mp4`、
`scenes/`、`*.zip`、`*.exe`。

**场景包、客户端安装包、比赛录屏都不进仓库**——放 D 盘或群里。

---

## 7. 交接

任何人请假/掉线，他的分支和 Issue 必须能被人接手：

- 提交前在 Issue 里写「做到哪、下一步、卡在哪」
- 代码里的 TODO 写成 `# TODO(姓名): 具体要做什么`
