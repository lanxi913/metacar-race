# 贡献指南

详细流程见 [docs/03-workflow.md](docs/03-workflow.md)，这里只列最容易忘的几条。

1. 开工前 `git checkout main && git pull`，然后开 `feat/xxx` 分支。
2. 只改自己负责的目录（`control/` / `perception/` / `multiagent/`）。
3. 提交前必须本地跑通：
   ```bat
   .venv\Scripts\python.exe check_env.py
   .venv\Scripts\python.exe -u main.py
   ```
4. 提交信息用 `feat|fix|docs|chore(scope): 说明`。
5. PR 里写清楚「改了什么 / 怎么验证 / 关联 Issue」，至少一人 review。
6. 不提交 `.venv/`、场景包、录屏、exe。
