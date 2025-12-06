---
title: git命令自用
description: 记录git的相关命令
date: 2025-12-06
categories:
    - git
---

## 基础配置
```bash
# 设置用户名和邮箱（全局）
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"

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
git branch dev

# 切换分支
git checkout dev

# 创建并切换到新分支
git checkout -b feature-x

# 删除本地分支
git branch -d dev
git branch -D dev  # 强制删除
```