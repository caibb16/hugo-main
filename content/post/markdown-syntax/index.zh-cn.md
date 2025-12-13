+++
author = "Hugo Authors"
title = "Markdown 语法指南"
date = "2019-03-11"
description = "展示基本 Markdown 语法和 HTML 元素格式的示例文章。"
tags = [
    "markdown",
    "css",
    "html",
    "themes",
]
categories = [
    "themes",
    "syntax",
]
series = ["Themes Guide"]
aliases = ["migrate-from-jekyl"]
image = "pawel-czerwinski-8uZPynIu-rQ-unsplash.jpg"
+++

本文提供了可在 Hugo 内容文件中使用的基本 Markdown 语法示例，同时展示了 Hugo 主题中 CSS 对基本 HTML 元素的装饰效果。
<!--more-->

## 标题

以下 HTML `<h1>`—`<h6>` 元素代表六个级别的章节标题。`<h1>` 是最高级别，而 `<h6>` 是最低级别。

# H1
## H2
### H3
#### H4
##### H5
###### H6

## 段落

这是一段示例文本。段落是文档中最基本的文本单元，用于组织和呈现相关的思想和信息。在 Markdown 中，段落由一个或多个连续的文本行组成，通过一个或多个空白行与其他段落分隔。

这是另一段示例文本。通过将文本分成段落，可以提高文档的可读性，使读者更容易理解和吸收信息。良好的段落结构是清晰写作的关键。

## 引用块

引用块元素表示从其他来源引用的内容，可选择性地包含引文（必须在 `footer` 或 `cite` 元素中），以及可选的行内修改（如注释和缩写）。

#### 无署名引用

> 这是一段示例引用文本。
> **注意**你可以在引用块中使用 *Markdown 语法*。

#### 有署名引用

> 不要通过共享内存来通信，而应该通过通信来共享内存。<br>
> — <cite>Rob Pike[^1]</cite>

[^1]: 以上引用摘自 Rob Pike 在 2015 年 11 月 18 日 Gopherfest 期间的[演讲](https://www.youtube.com/watch?v=PAAkCSZUG1c)。

## 表格

表格不是核心 Markdown 规范的一部分，但 Hugo 开箱即用地支持它们。

   姓名 | 年龄
--------|------
    张三 | 27
    李四 | 23

#### 表格内的行内 Markdown

| 斜体   | 粗体     | 代码   |
| --------  | -------- | ------ |
| *斜体* | **粗体** | `代码` |

| A                                                        | B                                                                                                             | C                                                                                                                                    | D                                                 | E                                                          | F                                                                    |
|----------------------------------------------------------|---------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------|------------------------------------------------------------|----------------------------------------------------------------------|
| Lorem ipsum dolor sit amet, consectetur adipiscing elit. | Phasellus ultricies, sapien non euismod aliquam, dui ligula tincidunt odio, at accumsan nulla sapien eget ex. | Proin eleifend dictum ipsum, non euismod ipsum pulvinar et. Vivamus sollicitudin, quam in pulvinar aliquam, metus elit pretium purus | Proin sit amet velit nec enim imperdiet vehicula. | Ut bibendum vestibulum quam, eu egestas turpis gravida nec | Sed scelerisque nec turpis vel viverra. Vivamus vitae pretium sapien |

## 代码块

#### 使用反引号的代码块

```html
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Example HTML5 Document</title>
</head>
<body>
  <p>Test</p>
</body>
</html>
```

#### 使用四个空格缩进的代码块

    <!doctype html>
    <html lang="en">
    <head>
      <meta charset="utf-8">
      <title>Example HTML5 Document</title>
    </head>
    <body>
      <p>Test</p>
    </body>
    </html>

#### 使用 Hugo 内置高亮短代码的代码块
{{< highlight html >}}
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Example HTML5 Document</title>
</head>
<body>
  <p>Test</p>
</body>
</html>
{{< /highlight >}}

#### Diff 差异代码块

```diff
[dependencies.bevy]
git = "https://github.com/bevyengine/bevy"
rev = "11f52b8c72fc3a568e8bb4a4cd1f3eb025ac2e13"
- features = ["dynamic"]
+ features = ["jpeg", "dynamic"]
```

## 列表类型

#### 有序列表

1. 第一项
2. 第二项
3. 第三项

#### 无序列表

* 列表项
* 另一项
* 再一项

#### 嵌套列表

* 水果
  * 苹果
  * 橙子
  * 香蕉
* 乳制品
  * 牛奶
  * 奶酪

## 其他元素 — abbr, sub, sup, kbd, mark

<abbr title="Graphics Interchange Format">GIF</abbr> 是一种位图图像格式。

H<sub>2</sub>O

X<sup>n</sup> + Y<sup>n</sup> = Z<sup>n</sup>

按 <kbd>CTRL</kbd> + <kbd>ALT</kbd> + <kbd>Delete</kbd> 键结束会话。

大多数 <mark>蝾螈</mark> 是夜行性的，捕食昆虫、蠕虫和其他小生物。

## 超链接图片

[![Google](https://www.google.com/images/branding/googlelogo/1x/googlelogo_light_color_272x92dp.png)](https://google.com)