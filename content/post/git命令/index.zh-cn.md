---
title: git命令自用
date: 2025-12-06
categories:
    - 工具
tags: ['git', '命令速查']
---

## 基础配置
```bash
# 设置用户名和邮箱（全局）
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"

# 配置全局代理，端口设置为clash的端口
git config --global http.proxy http://127.0.0.1:7897

# 查看当前配置
git config --list
```
## 仓库初始化与克隆
```bash
# 在当前目录初始化仓库
git init

# 克隆远程仓库
git clone https://github.com/username/repo.git

# 克隆指定分支
git clone -b 分支名 https://github.com/username/repo.git
```
## 分支管理
```bash
# 查看本地分支
git branch
# 创建新分支
git branch -m dev
# 重命名分支
git branch -m old-branch new-branch
# 切换分支
git checkout dev
# 创建并切换到新分支
git checkout -b feature-branch
# 删除本地分支
git branch -d dev
git branch -D dev  # 强制删除

# 强制推送本地分支到远程分支
git push -f origin main
# 关联远程分支并推送
git push -u origin main   # origin为远程名，main为本地分支名
# 如果远程分支不存在，直接创建并关联
git push --set-upstream origin main



```
## 本地和远程操作
```bash
# 查看远程仓库
git remote -v
# 添加远程仓库
git remote add origin <url>
# 暂存文件到暂存区
git add 文件名
# 提交更改到本地仓库
git commit -m "提交信息"
# 撤销上一次的更改提交
git reset --soft HEAD^
# 推送到远程仓库
git push origin main
# 拉取远程仓库最新代码
git pull origin main
```
