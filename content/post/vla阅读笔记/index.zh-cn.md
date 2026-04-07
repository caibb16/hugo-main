---
title: 'VLA阅读笔记'
date: 2026-04-07T08:45:20+08:00
draft: false
description: ""
categories:
  - 
tags: []
---

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
