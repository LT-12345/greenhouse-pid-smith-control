# GitHub 上传命令

## 方式一：上传到新仓库

先在 GitHub 网页端新建一个空仓库，例如：

```text
greenhouse-pid-smith-control
```

然后在本地命令行进入本项目目录，执行：

```bash
git init
git add .
git commit -m "Initial commit: greenhouse cascade PID and Smith predictor simulation"
git branch -M main
git remote add origin https://github.com/你的用户名/greenhouse-pid-smith-control.git
git push -u origin main
```

## 方式二：上传到已有仓库

```bash
git clone https://github.com/你的用户名/你的仓库名.git
cd 你的仓库名
```

把本项目文件复制进去，然后执行：

```bash
git add .
git commit -m "Add greenhouse PID Smith predictor simulation code"
git push
```

## 注意

如果你没有配置 GitHub 登录，推荐使用 GitHub Desktop 上传，或者在 VSCode 中登录 GitHub 后推送。
