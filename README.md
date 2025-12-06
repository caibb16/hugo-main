# caibb16 个人博客

基于 [Hugo](https://gohugo.io/) 构建的个人博客网站，使用 [hugo-theme-stack](https://github.com/CaiJimmy/hugo-theme-stack) 主题。

## 🌐 网站地址

[https://caibb16.github.io/](https://caibb16.github.io/)

## ✨ 特性

- 🌍 多语言支持（简体中文、English）
- 🎨 明暗主题切换
- 📝 Markdown 语法支持
- 📐 数学公式渲染（KaTeX）
- 💬 评论系统（Disqus）
- 🔍 全文搜索
- 📚 文章归档
- 🏷️ 标签与分类
- 📱 响应式设计

## 🚀 快速开始

### 环境要求

- [Hugo Extended](https://gohugo.io/installation/) (推荐 v0.100.0+)
- [Git](https://git-scm.com/)

### 本地运行

1. 克隆仓库

```bash
git clone https://github.com/caibb16/hugo-main.git
cd hugo-main
```

2. 初始化主题子模块

```bash
git submodule update --init --recursive
```

3. 启动本地服务器

```bash
hugo server -D
```

4. 在浏览器中访问 `http://localhost:1313`

### 构建静态网站

```bash
hugo
```

生成的静态文件将保存在 `public/` 目录中。

## 📁 项目结构

```
hugo-main/
├── archetypes/      # 文章模板
├── assets/          # 资源文件（SCSS、JS等）
├── content/         # 博客内容
│   ├── page/        # 独立页面（关于、归档、链接等）
│   └── post/        # 博客文章
├── layouts/         # 自定义布局
├── public/          # 构建输出目录
├── static/          # 静态资源
├── themes/          # Hugo 主题
└── hugo.yaml        # 站点配置文件
```

## ✍️ 写作

### 创建新文章

```bash
hugo new content/post/my-new-post/index.md
```

### Front Matter 示例

```yaml
---
title: "文章标题"
date: 2025-12-06
draft: false
description: "文章描述"
categories:
  - 分类名
tags:
  - 标签1
  - 标签2
image: cover.jpg
---
```

## ⚙️ 配置

主要配置文件为 `hugo.yaml`，可自定义：

- 网站基本信息
- 侧边栏设置
- 评论系统
- 社交链接
- 小部件配置
- 语言设置

## 📄 许可证

文章内容采用 [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) 许可协议。

## 🔗 相关链接

- [Hugo 官方文档](https://gohugo.io/documentation/)
- [Stack 主题文档](https://stack.jimmycai.com/)
- [GitHub](https://github.com/caibb16)
- [Twitter](https://x.com/caibb16)
