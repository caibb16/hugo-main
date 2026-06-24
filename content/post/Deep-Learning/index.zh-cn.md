---
title: Deep Learning
date: 2026-06-24T20:41:55+08:00
draft: false
description: 记录深度学习相关
categories:
  - 笔记
tags:
  - LLM
math: true
---
## 分词方法
1. word（词）粒度
中文句子：我喜欢看电影和读书。
分词结果：我 | 喜欢 | 看 | 电影 | 和 | 读书。
英文句子：I enjoy watching movies and reading books.
分词结果：I | enjoy | watching | movies | and | reading | books.
2. char（字符）粒度
中文句子：我喜欢看电影和读书。
分词结果：我 | 喜 | 欢 | 看 | 电 | 影 | 和 | 读 | 书 | 。
英文句子：I enjoy watching movies and reading books.
分词结果：I |   | e | n | j | o | y |   | w | a | t | c | h | i | n | g |   | m | o | v | i | e | s |   | a | n | d |   | r | e | a | d | i | n | g |   | b | o | o | k | s | .
3.   subword（子词）粒度
	1. WordPiece
		- 核心思想：单词拆分成多个前缀符号最小单元，再通过子词合并规则将最小单元进行合并为子词级别
		- 计算合并分数
		score=(freq_of_pair)/(freq_of_first_element×freq_of_second_element)
		分数 = 合并pair候选的频率 / (第一个元素的频率 × 第二个元素的频率)
		- 将分数最高的进行合并然后开始循环迭代
	2. Byte-Pair Encoding (BPE)
			- 核心思想：逐步合并出现频率最高的子词对，当词元总数达到设定的阈值后就停止合并，从而构建出一个词表，实现数据压缩
			- 分词阶段：在进行模型训练前，会预先训练好一个词表，存储token和对应 ID，分词时的一个常见方法是不断尝试合并当前相邻的字符，只要合并后的新字符串存在于词表中，就执行合并
## Token Embedding
- 分词后得到索引 ID
$$tokens = [t_1, t_2,..., t_n],  t_i \in {0, 1,..., |V|-1}$$
- 构建嵌入矩阵
$$W_E \in R^{|V|\times d_{emd}}，|V|为词表大小$$
- 词嵌入：对于单个 token id  𝑡，其 embedding：
$$x=W_E[t] \in R^{d_{emb}}$$
## 位置编码
1. 固定的正/余弦编码
$$PE_{pos,2i}=sin(\frac{pos}{10000^{2i/d_{model}}})$$
$$PE_{pos,2i+1}=cos(\frac{pos}{10000^{2i/d_{model}}})$$
	- 模型可以通过向量运算直接感知所有 token 的相对距离。
2. 可学习的位置嵌入
	- 为每个位置分配一个可训练向量，就像词向量（Token Embedding）从嵌入矩阵中取向量，位置本身作为索引
	- 位置嵌入矩阵：
$$P \in R^{L_{max} \times d}，L_{max}为序列最大长度$$
	- 位置pos的位置向量为：$$ PE(pos)=P[pos]$$
	- 和Token Embedding结合：$$X_{input}(pos)=TokenEmbedding(word)+PositionEmbedding(pos)$$
## FNN
- 输入层，隐藏层，输出层
- 结构：$FFN(x)=W_2\sigma(W_1x+b_1)+b_2$
- 第一层线性层（升维） → 激活函数 → 第二层线性层（降维）
- FFN 负责 对每个 token 独立地进行非线性特征变换
## RNN
- 严格按照序列顺序处理每一个 token，输入序列有多少个 token，就会进行多少个时间步的计算
- 核心递推公式：$h_t=f(Wx_t+Uh_{t-1})$
- 隐藏状态ht取决于当前时间步$x_t$和前一个时间步$h_{t-1}$
- nn.RNN的输出包括 两个张量:
	- output 是 每个时间步的隐藏状态序列$$output.shape=(seq\_len,batch_size,hidden\_size)$$
	- h_n 是 最后一个时间步的隐藏状态$$h\_n.shape=(num\_layers,batch\_size,hidden\_size)$$
## CNN
- 局部连接：每个神经元只连接前一层的局部神经元
- 权值共享：通过卷积核在输入数据上滑动，不同神经元共享同一个卷积核的权重
	- 卷积核的深度与输入矩阵深度一致，其尺寸只需指定前两个维度
	- 卷积核的数量就是卷积层的输出深度
	- 输出尺寸=(W+2P-F)/S+1	padding表示在输入特征图周围填充0
	- 通过调整P的大小，可保持卷积后尺寸大小不变
- 池化层：利用图片特征的局部不变形进行下采样，降低网络的空间尺寸
	- 方法：Max Pooling和Mean Pooling

## 多头自注意力层
- 输入和输出的形状完全相同。
	- 输入：每个token的表示是独立的，只包含该token自身的信息
	- 输出：每个token的表示是上下文感知的，融合了所有其他token的信息
- 注意力分数：计算每个token对其他token的关注程度$$score = Q * K_T / \surd d_k$$$$attention=softmax(score)$$
- 加权求和得到上下文向量：输出序列的每个token都融合了上下文信息
$$context=attention * V$$
## KV cache
- LLM推理过程
	- Prefill阶段：输入序列输入模型，并行计算所有token的q,k,v，并保存kv cache
	- Decode阶段：在生成下一个token后，模型只输入最新一个token,计算它的q,k,v，读取历史 KV cache，用新token的 Q 对历史所有 K 做 attention，得到当前输出，再把新的 K/V append 到 cache
## LoRA
- 理论基础：大模型微调过程中的参数矩阵改变量具有低秩性