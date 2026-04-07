---
title: 'LLM阅读笔记'
date: 2026-04-07T08:52:43+08:00
draft: false
description: ""
categories:
  - 
tags: []
---

## nanoGPT
### LayerNorm  
   - 计算均值：$\mu = \frac{1}{d} \sum_{i=1}^{d} x_i$  
   - 计算方差：$\sigma^2 = \frac{1}{d} \sum_{i=1}^{d} (x_i - \mu)^2$  
   - 归一化：$y_i = \frac{x_i - \mu}{\sqrt{\sigma^2 + \epsilon}} \cdot \alpha_i + \beta_i$  
   - 其中$\alpha$和$\beta$是可学习参数，$\epsilon$是防止除零的小常数
### MHA  
   - 计算QKV矩阵：$Q = XW_Q, K = XW_K, V = XW_V$  
   - 计算注意力权重：$A = \text{softmax}(\frac{QK^T}{\sqrt{d_k}})$  
   - 输出：$O = AV$  
   - 多头机制：将输入分成多个头，每个头独立计算注意力，最后拼接输出
### 位置编码  
   ```
   "wpe": nn.Embedding(config.block_size, config.n_embd)
   pos_emb = wpe(pos)
   ```
### `generate`函数  
   - 输入序列裁剪
   ```
   idx_cond = idx if idx.size(1) <= self.config.block_size else idx[:, -self.config.block_size:]
   ```
   - 输出token拼接至输入序列
   ```
   idx = torch.cat((idx_cond, idx_next), dim=1)
   ```
## minimind
### RMSNorm
1. 计算公式
  - 计算均方根：$rms = \sqrt{\frac{1}{d} \sum_{i=1}^{d} x_i^2}$  
  - 归一化：$y_i = \frac{x_i}{rms + \epsilon} \cdot \gamma_i$  
  - 其中$\gamma$是可学习参数，$\epsilon$是防止除零的小常数
2. 与LayerNorm的区别
  - LayerNorm计算均值和方差，而RMSNorm只计算均方根
  - RMSNorm没有偏置项$\beta$，只有缩放参数$\gamma$
### RoPE位置编码  
1. 计算公式
  - 位置编码：$f(x_m,m) = x_m*e^{im\theta_i} = x_m \cdot \cos(m\theta_i) + i*x_m \cdot \sin(m\theta_i)$ ,其中m为位置索引，$\theta_i$为频率，$x_m$表示位置m的输入向量
  - 两两分组：将向量分为前半部分和后半部分并一一配对(与RoPE原论文的配对方式略有不同，但最终效果相同)  
  $x_m = [x_{m,0}, x_{m,1}, ..., x_{m,d/2-1}, x_{m,d/2}, ..., x_{m,d-1}]$
  rotate_half $(x_m) = [-x_{m,d/2}, ..., -x_{m,d-1}, x_{m,0}, ..., x_{m,d/2-1}]$  
  表示为复数形式：$x_m = [x_{m,0} + i*x_{m,d/2}, x_{m,1} + i*x_{m,d/2+1}, ..., x_{m,d/2-1} + i*x_{m,d-1}]$  
  rotate_half $(x_m) = [-x_{m,d/2} + i*x_{m,0}, -x_{m,d/2+1} + i*x_{m,1}, ..., -x_{m,d-1} + i*x_{m,d/2-1}]$  
  则$i*x_m$表示为：$i*x_m =$ rotate_half $(x_m)$
  - 最终位置编码：$f(x_m,m) = x_m \cdot \cos(m\theta_i) +$ rotate_half $(x_m) \cdot \sin(m\theta_i)$
2. 代码如下：
  - $\theta_i = base^{-2i/d}$
   ```python
   freqs = 1.0 / (rope_base ** (torch.arange(0, dim, 2)[: (dim // 2)].float() / dim))
   ```
  - 位置嵌入
   ```python
   q_embed = ((q * cos.unsqueeze(unsqueeze_dim)) + (rotate_half(q) * sin.unsqueeze(unsqueeze_dim))).to(q.dtype)
   k_embed = ((k * cos.unsqueeze(unsqueeze_dim)) + (rotate_half(k) * sin.unsqueeze(unsqueeze_dim))).to(k.dtype)
   ```
   编码后的`q_embed`和`k_embed`做内积包含了相对位置信息 m-n
### GQA
1. 概念：(Grouped Query Attention)基于MHA，减少k,v头的数量，让多个q头共享同一组k,v头，从而降低计算复杂度，减少KV_cache
2. 具体实现：进行前向传播时，将k,v矩阵在头数量维度进行复制，使其能够与q头进行计算
### MoE
1. 概念：(Mixture of Experts)将FFN层替换为多个专家网络，每个专家网络是一个标准的FFN层，在前向传播时每个token动态选择其中k个专家进行计算，从而提高模型的表达能力和效率
2. 具体实现：使用一个门控网络（gating network）来计算每个token选择哪个专家，门控网络输出一个权重分布，表示每个专家的选择概率，然后根据这个分布选择k个专家进行计算，并将它们的输出加权求和作为最终的FFN输出

    