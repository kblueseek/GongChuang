# Git 分支协作速查

GongChuang 项目只有一个远程仓库，里面两条分支：

| 分支 | 内容 | 谁维护 | 本地文件夹 |
|---|---|---|---|
| `main` | 本项目（树莓派底盘 / 导航） | 我 | `~/GongChuang` |
| `vision` | 视觉部分 | 队友 | `~/vision`（单独 clone） |

远程地址：`https://github.com/kblueseek/GongChuang.git`

---

## 1. 我的分支 main（~/GongChuang）

### 拉取（把远程最新代码拉下来）

```bash
cd ~/GongChuang
git pull origin main
```

### 推送（把本地改动传上去）

```bash
cd ~/GongChuang
git status                  # 先看改了哪些文件
git add -A                  # 全部暂存（只加某文件: git add 路径）
git commit -m "改了什么"     # 提交
git push origin main        # 推到远程 main
```

---

## 2. 视觉分支 vision（单独文件夹）

> 现在 `~/vision_only` 还不是 git 仓库（只是散文件），不能直接 pull/push。
> 用下面一条命令把 vision 分支 clone 成一个独立文件夹 `~/vision`：

### 首次 clone（只做一次）

```bash
cd ~
git clone --branch vision https://github.com/kblueseek/GongChuang.git vision
```

### 拉取队友更新

```bash
cd ~/vision
git pull origin vision
```

### 推送改动

```bash
cd ~/vision
git status
git add -A
git commit -m "改了什么"
git push origin vision
```

---

## 3. 以后把视觉合并进本项目

```bash
cd ~/GongChuang
git fetch origin vision                 # 拿到最新 vision
git merge origin/vision                 # 合并进来（有冲突就解冲突）

# 或者只挑某几个文件/目录进来，不整体 merge：
git checkout origin/vision -- raspberrypi/vision
```

---

## 4. 常用排查

```bash
git status              # 看当前改了什么
git log --oneline -5    # 看最近提交
git diff                # 看未暂存的改动内容
git remote -v           # 看远程地址
git branch -a           # 看所有分支
```

### push 提示输入账号密码

HTTPS 推送要 GitHub 用户名 + Personal Access Token（不是登录密码）。
第一次会记住；想长期记住就执行 `git config credential.helper store`。

### pull 时冲突

改同一个文件会冲突。稳妥做法：

```bash
git stash          # 先把本地改动存起来
git pull           # 拉最新
git stash pop      # 恢复本地改动，手工解决冲突
```
