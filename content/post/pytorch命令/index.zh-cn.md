---
date: '2025-12-08T16:18:37+08:00'
draft: false
title: 'PyTorch 张量操作命令速查'
description: '记录 PyTorch 常用张量操作命令'
categories: ['PyTorch', 'Deep Learning']
tags: ['PyTorch', '张量', '深度学习', '命令速查']
---

> 记录常用 PyTorch 张量操作命令，偏"查表备忘"。

## 环境与基础

```python
import torch

# 查看版本
print(torch.__version__)

# 判断是否有 GPU
torch.cuda.is_available()

# 当前 GPU 数量
torch.cuda.device_count()
```

---

## 1. 张量创建

### 1.1 从 Python 数据创建

```python
x = torch.tensor([1, 2, 3])          # 1D
x = torch.tensor([[1., 2.], [3., 4.]])  # 2D，浮点
x = torch.FloatTensor([1, 2, 3])     # 指定类型
x = torch.LongTensor([1, 2, 3])      # int64
```

### 1.2 常见初始化

```python
torch.zeros(3, 4)        # 全 0
torch.ones(2, 3)         # 全 1
torch.full((2, 3), 7)    # 全 7

torch.eye(3)             # 单位矩阵
torch.arange(0, 10, 2)   # [0, 2, 4, 6, 8]
torch.linspace(0, 1, 5)  # [0., 0.25, 0.5, 0.75, 1.]
```

### 1.3 随机张量

```python
torch.rand(2, 3)         # [0, 1) 均匀分布
torch.randn(2, 3)        # N(0, 1) 正态分布
torch.randint(0, 10, (2, 3))  # [0, 10) 整数
```

### 1.4 与已有张量同形状

```python
a = torch.randn(2, 3)
torch.zeros_like(a)
torch.ones_like(a)
torch.rand_like(a)
```

---

## 2. 张量属性

```python
x = torch.randn(2, 3, 4)

x.shape      # 或 x.size()
x.ndim       # 维度数
x.dtype      # 数据类型
x.device     # 设备 (cpu / cuda:0)
x.numel()    # 元素总数
```

---

## 3. 设备与类型转换

```python
x = torch.randn(3, 4)

# 类型
x.float()          # 转 float32
x.double()         # 转 float64
x.long()           # 转 int64
x.int()            # 转 int32

# 设备
x_cuda = x.to('cuda')       # 或 .cuda()
x_cpu = x_cuda.to('cpu')    # 或 .cpu()

# 拷贝到同设备同类型
y = torch.zeros_like(x)     # 形状/类型一致
```

---

## 4. 形状操作（view / reshape / squeeze 等）

```python
x = torch.randn(1, 3, 4)

# 展平
x_flat = x.view(-1)              # 或 x.reshape(-1)

# 改变形状
y = x.view(2, 12)                # 2 x 12
z = x.reshape(3, 8)              # 3 x 8（更通用）

# 插入维度
x_unsq = x.unsqueeze(0)          # 在 dim=0 插一维: (1, 2, 3, 4)
x_unsq = x.unsqueeze(2)          # (2, 3, 1, 4)

# 去掉大小为1的维度
x_sq = x_unsq.squeeze()          # 去掉所有 =1 的维度
x_sq = x_unsq.squeeze(0)         # 只去掉 dim=0

# 交换维度
x_perm = x.permute(0, 2, 1)      # (2, 4, 3)

# 扩展维度
x_exp = x.expand(2, -1, -1)      # 维度变为 (2, 3, 4)，-1表示该维度不变

# 转置（2D 专用）
m = torch.randn(3, 4)
m_t = m.t()                      # (4, 3)
```

---

## 5. 基本算术运算

```python
a = torch.randn(2, 3)
b = torch.randn(2, 3)

a + b
a - b
a * b          # 逐元素乘
a / b

torch.add(a, b)
torch.mul(a, b)

# 标量运算
a + 2
a * 10
```

### 5.1 矩阵运算

```python
A = torch.randn(2, 3)
B = torch.randn(3, 4)

C = A @ B              # 矩阵乘法
C = torch.matmul(A, B)

# 批量矩阵乘法
X = torch.randn(10, 2, 3)
Y = torch.randn(10, 3, 4)
Z = torch.matmul(X, Y) # (10, 2, 4)

# 转置 + 乘法
A_T = A.t()
torch.mm(A, B)         # 2D 矩阵乘
```

---

## 6. 维度上的聚合操作

```python
x = torch.randn(2, 3, 4)

x.sum()                        # 所有元素和（标量）
x.sum(dim=0)                   # 在 dim=0 上求和
x.mean(dim=1)                  # 在 dim=1 上求均值
x.max()                        # 全局最大值
x.max(dim=1)                   # 返回 (values, indices)
x.min(dim=2)
x.prod(dim=0)                  # 连乘

# 保留维度
x.sum(dim=1, keepdim=True)     # shape 中 dim=1 仍保留，大小为1
```

---

## 7. 索引与切片

```python
x = torch.arange(0, 12).view(3, 4)
# x:
# [[0, 1, 2, 3],
#  [4, 5, 6, 7],
#  [8, 9,10,11]]

x[0, 0]        # 标量
x[1]           # 第2行
x[:, 1]        # 第2列
x[0:2, 1:3]    # 行0~1, 列1~2

# 使用布尔索引
mask = x > 5
x[mask]

# 高级索引
idx = torch.tensor([0, 2])
x[idx]         # 选第 0、2 行
```

---

## 8. 拼接与分割

```python
a = torch.randn(2, 3)
b = torch.randn(2, 3)

# 沿行拼接（行数相同，列加长）
torch.cat([a, b], dim=0)   # (4, 3)

# 沿列拼接（列数相同，行加长）
torch.cat([a, b], dim=1)   # (2, 6)

# 堆叠（新增一维）
torch.stack([a, b], dim=0) # (2, 2, 3)
```

---

## 9. 广播（Broadcasting）

```python
x = torch.randn(2, 3)
b = torch.randn(3)

# b 自动扩展为 (2, 3)
y = x + b

# 手动调整维度后广播
w = torch.randn(2, 1)   # (2,1) -> (2,3)
z = x + w
```

---

## 10. 自动求导基础（梯度相关）

```python
x = torch.randn(3, 4, requires_grad=True)  # 开启梯度

y = x * 2 + 1
loss = y.mean()

loss.backward()        # 反向传播

x.grad                 # ∂loss/∂x
x.grad.zero_()         # 清空梯度
```

---

## 11. 常用张量实用函数

```python
x = torch.tensor([-1.0, 0.5, 2.0])

torch.clamp(x, min=0.0, max=1.0)  # 截断
torch.abs(x)                      # 绝对值
torch.relu(x)                     # ReLU

torch.argmax(x)                   # 最大值索引
torch.argmin(x)                   # 最小值索引

torch.topk(x, k=2)                # 取前 k 大
```

---

## 12. 与 NumPy 互转

```python
import numpy as np

# Tensor -> NumPy
x = torch.randn(3, 4)
np_arr = x.numpy()                # 注意：共享内存（在 CPU 上）

# NumPy -> Tensor
y = torch.from_numpy(np_arr)

# 如果在 GPU 上，需要先转回 CPU
x_cpu = x.to('cpu').numpy()
```

---

> 后续如果有更多 PyTorch 命令（如优化器、DataLoader、nn.Module 等），可以在本文下继续补充新章节。
