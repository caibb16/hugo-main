---
title: 'VLM阅读笔记'
date: 2026-03-18T10:59:00+08:00
draft: false
description: ""
categories:
  - 笔记
tags: [VLM]
math: true 
---

## LLaVA
`prepare_inputs_labels_for_multimodal()` 函数解读  
[源代码](https://github.com/haotian-liu/LLaVA/blob/main/llava/model/llava_arch.py)  
该函数用于将输入文本和图像进行拼接，并生成对应的标签，是视觉语言融合的核心函数，  
事例如下：

1. 首先对原始文本进行分词
```python
"USER: <image>\n What is the color of the car? \n
ASSISTANT: "

↓ encode

124, 31, -200, 83, ..., 91  (-200 为图片索引)
```
2. 提取文本并嵌入

```python
124, 31, -200, 83, ..., 91 

↓ 提取文本

[124, 31, 83, ..., 91]

↓ 嵌入并分割

[[], []], [[], ..., []]
```
![](image-1.png)

3. 插入图像特征

```python
[[], []], [[], ..., []]

↓ 插入图像

[[], [], [image_feature], ..., [], ..., []]  
```
![](image-2.png)

图像特征获取如下： 

![](image.png)

4. 插入图像特征后的序列被送入LLM进行处理，具体见[llava_llama.py](https://github.com/haotian-liu/LLaVA/blob/main/llava/model/language_model/llava_llama.py)中的`forward()`函数。

### 三、lora微调
[源代码](https://github.com/haotian-liu/LLaVA/blob/main/llava/train/train.py)  

使用[scripts/v1_5/finetune_lora.sh](https://github.com/haotian-liu/LLaVA/blob/main/scripts/v1_5/finetune_lora.sh)脚本进行微调，可以看到 lora 参数如下：

```bash
--lora_enable True --lora_r 128 --lora_alpha 256 --mm_projector_lr 2e-5
```
- lora_enable：启用lora微调
- lora_r：lora秩，控制低秩矩阵的维度
- lora_alpha：lora缩放因子，控制lora更新的幅度
- mm_projector_lr：多模态投影器的学习率

配置LoRA参数并添加到模型
![alt text](image-3.png)
其中`get_peft_model()`函数把 LoRA 注入到目标层，返回一个 PEFT 包装后的模型，之后训练时只更新 LoRA 参数，基础模型权重保持冻结，具体可参考[PEFT文档](https://hugging-face.cn/docs/peft/quicktour) 。  
`注意`：原项目中的pyproject.toml文件中没有指定peft库的版本，安装时会默认安装最新版本，实测目前peft最新版本与项目指定的transformers库存在兼容性问题，建议使用peft 0.4.0版本 。
