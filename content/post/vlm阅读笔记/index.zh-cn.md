---
title: 'VLM阅读笔记'
date: 2026-03-18T10:59:00+08:00
draft: false
description: ""
categories:
  - 笔记
tags: []
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

# VLA
## OpenVLA
`class ActionTokenizer`类解读  
[源代码](https://github.com/openvla/openvla/blob/main/prismatic/vla/action_tokenizer.py)

1. 将连续动作区间均匀离散化

      bins: $\underbrace{[-1.0, ..., 1.0]}_{256个边界点}$  

      $\downarrow 取区间中心 $

      bin_centers: $\underbrace{[-0.99609375, ..., 0.9960784]}_{255个中心点}$
```python
# Create Uniform Bins + Compute Bin Centers
self.bins = np.linspace(min_action, max_action, self.n_bins)
self.bin_centers = (self.bins[:-1] + self.bins[1:]) / 2.0
```
2. 动作编码  
      action $\xrightarrow{clip}$ action $\in [-1.0, 1.0] \xrightarrow{digitize}$ discretized_action $\in [1, 256]$
      $\xrightarrow{decode}$ token_id
```python
action = np.clip(action, a_min=float(self.min_action), a_max=float(self.max_action))
discretized_action = np.digitize(action, self.bins)
# 映射到词表的末尾一段token_id（对于BPE分词法，最不常用的token在词表末尾）
return self.tokenizer.decode(list(self.tokenizer.vocab_size - discretized_action))
```
3. 动作解码  
      token_id $\xrightarrow{decode}$ discretized_action $\in [1, 256] \xrightarrow{clip}$ decretized_action $\in [0, 254]$
      $\xrightarrow{取值}$ bin_centers[discretized_actions]
```python
discretized_actions = self.tokenizer.vocab_size - action_token_ids
discretized_actions = np.clip(discretized_actions - 1, a_min=0, a_max=self.bin_centers.shape[0] - 1)
return self.bin_centers[discretized_actions]
```

