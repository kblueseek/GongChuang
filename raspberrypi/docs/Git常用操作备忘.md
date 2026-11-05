# Git 常用操作备忘

> 单人项目典型场景：笔记本 ↔ GitHub ↔ 树莓派 三方同步。
> 原则：**已经 push 到 GitHub 的东西用 `revert`（不改历史）；只有自己本地有的才用 `reset`（改写历史）。**

## 一、看历史、看状态

```bash
git log --oneline -10        # 最近 10 条提交（简写）
git log --oneline --graph --all -15   # 带分叉图，看分支关系
git status                   # 工作区状态（改了什么、有没有没提交的）
git diff <文件>               # 看某个文件具体改了哪几行
```

## 二、切换到旧版本

以回到 `952dd96` 为例，按目的三选一：

```bash
# ① 只是看看 / 试跑（分离头指针，别在里面改代码提交）
git checkout 952dd96
git checkout main            # 看完回最新

# ② 基于旧版本长期用（推荐：main 不动，新旧共存）
git switch -c old-stable 952dd96
git push -u origin old-stable          # 推到 GitHub
# 树莓派上切过去：
git fetch origin && git switch old-stable
git switch main                        # 随时切回来

# ③ main 本身彻底回到旧版本（改写历史）
git reset --hard 952dd96
git push --force-with-lease origin main
# 树莓派同步（⚠️ 丢弃 Pi 上未推送的改动）：
git fetch origin && git reset --hard origin/main
```

## 三、撤销改动

```bash
# 改动还没提交：丢弃工作区的修改
git restore .                          # 全部（⚠️ 不可恢复）
git restore 路径/文件                   # 只回退某一个文件

# 已经提交了，想撤销最后一次提交（保留改动在工作区）
git reset --soft HEAD~1

# 已经提交了，连改动一起丢掉（⚠️ 不可恢复）
git reset --hard HEAD~1

# 已经 push 了：生成"反向提交"，历史不动，最安全
git revert HEAD                        # 撤销最近一次
git revert HEAD~2..HEAD                # 一次撤销最近三次
git push                               # 正常推送即可
```

## 四、GitHub（远程仓库）回退到之前版本

| 场景 | 做法 |
|---|---|
| 只是撤掉某次改动 | 本地 `git revert <提交号>` 后正常 `git push` |
| 那次改动是 PR 合入的 | GitHub 网页 → 该 PR 页面 → 点 **Revert** 按钮，合并自动生成的回退 PR |
| 必须抹除历史（含密码/大文件） | `git reset --hard <旧提交>` + `git push --force-with-lease`，分支受保护时先去 Settings → Branches 删保护规则 |

## 五、三方同步常见坑

### `git pull` 报 divergent branches（两边都提交了）

```bash
git log --oneline main..origin/main     # 远程多了什么
git log --oneline origin/main..main     # 本地多了什么（确认没有要留的再动）
# 要保留本地提交：rebase（历史一条线，推荐单人用）
git config pull.rebase true && git pull
# 要本地和远程完全一样（⚠️ 丢弃本地未推送提交）：
git fetch origin && git reset --hard origin/main
```

### 换电脑/重装后同步

```bash
git clone <仓库地址>        # 第一次
git pull                  # 之后每次开工先拉
```

## 六、其他常用

```bash
git stash                          # 临时寄存未提交的改动
git stash pop                      # 取回寄存
git branch                         # 看本地分支
git switch 分支名                   # 切分支
git branch -d 分支名                # 删本地分支（合并过的）
```
