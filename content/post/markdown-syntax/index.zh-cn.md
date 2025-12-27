+++
author = "Hugo Authors"
title = "Markdown 语法指南"
date = "2019-03-11"
description = "展示基本 Markdown 语法和 HTML 元素格式的示例文章。"
tags = [
    "markdown",
    "语法",
]
categories = [
    "工具"
]
series = ["Themes Guide"]
aliases = ["migrate-from-jekyl"]
image = "pawel-czerwinski-8uZPynIu-rQ-unsplash.jpg"
+++

# Markdown 语法说明

Markdown 是一种轻量级标记语言，广泛用于编写文档、博客、说明书等。它语法简洁，易于阅读和编写，可快速转换为 HTML 网页。

## 标题
使用 `#` 号表示标题，支持六级标题：
```
# 一级标题
## 二级标题
### 三级标题
```

## 段落
直接输入文本即可形成段落。段落之间用空行分隔。

## 引用块
使用 `>` 表示引用内容：
```
> 这是引用内容
```

## 列表
- 无序列表：使用 `*`、`-` 或 `+` 作为标记
```
* 项目一
* 项目二
```
- 有序列表：使用数字加点
```
1. 第一项
2. 第二项
```
- 支持嵌套列表

## 代码块
- 行内代码：用反引号包裹，如 `print('Hello')`
- 多行代码块：用三个反引号包裹
````
```
import torch
print(torch.__version__)
```
````

## 表格
使用 `|` 和 `-` 创建表格：
```
| 姓名 | 年龄 |
| ---- | ---- |
| 张三 | 27   |
| 李四 | 23   |
```

## 链接与图片
- 链接：`[描述](网址)`
- 图片：`![描述](图片地址)`

## 强调
- *斜体*：用一个星号或下划线包裹
- **粗体**：用两个星号或下划线包裹
- ~~删除线~~：用两个波浪线包裹

## 其他元素
- 分割线：`---` 或 `***`
- 脚注：`[^1]`，在文档底部添加脚注内容
- 行内 HTML：可直接嵌入 HTML 标签
- 特殊格式：如 `<kbd>Ctrl</kbd>`、`<mark>高亮</mark>`、`<sub>下标</sub>`、`<sup>上标</sup>`

## 视频与多媒体
- 图片：`![图片描述](图片地址)`
- 视频：`<video src="xxx.mp4" controls width="700"></video>`

## 数学公式
支持 LaTeX 语法：
- 行内公式：`$公式$`
- 块级公式：`$$公式$$`  
- 多行公式：`$$$公式$$$`

常用 LaTeX 命令：
```
\alpha, \beta, \gamma, \delta, \epsilon # 希腊字母
\sum_{下标}^{上标}, \prod_{下标}^{上标}, \int_{下标}^{上标} # 求和、乘积、积分
\frac{分子}{分母}  # 分数
\sqrt{表达式}     # 平方根
\sqrt[n]{表达式}  # n 次根
\begin{matrix} ... \end{matrix} # 矩阵
```

---

Markdown 语法简单易学，适合快速编写结构化文档。更多高级用法可参考 [Markdown 官方文档](https://markdown.com.cn/)  。