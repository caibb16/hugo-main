+++
date = '2025-12-29T12:32:33+08:00'
draft = false
title = '数值分析笔记'
description = '期末救急用，记录一些考点公式'
categories = [
    "笔记"
]
tags = [
    "数值分析"
]
math = true
+++

## 绪论
### 误差
绝对误差： $e = x^* - x$  
$|e| = |x^* - x| \leq \varepsilon$，$\varepsilon$ 为近似值 $x$ 的绝对误差限  
相对误差： $e_r = \frac{e}{x}$  
$|e_r| = \frac{|e|}{|x|} \leq \varepsilon_r$，$\varepsilon_r$ 为近似值 $x$ 的相对误差限  
有效数：如果 $|x^* - x| \leq \frac{1}{2} \times 10^{-t}$，则近似值 $x$ 的有效数字为 $n$ 位，$n = t + $ 小数点前的有效数字位数  
数据误差对函数的影响：  
若 $y = f(x_1, x_2)$，则误差传播可用泰勒展开近似表示，即

$$
e(y) \approx \frac{\partial f(x_1, x_2)}{\partial x_1} e(x_1) + \frac{\partial f(x_1, x_2)}{\partial x_2} e(x_2)
$$

$$
e_r(y) = \frac{e(y)}{y} \approx \frac{\partial f}{\partial x_1} \frac{x_1}{f(x_1, x_2)} e_r(x_1) + \frac{\partial f}{\partial x_2} \frac{x_2}{f(x_1, x_2)} e_r(x_2)
$$

补充：
二元函数泰勒展开：

$$
f(x_1 + \Delta x_1, x_2 + \Delta x_2) = f(x_1, x_2) + \frac{\partial f}{\partial x_1} \Delta x_1 + \frac{\partial f}{\partial x_2} \Delta x_2 \newline + \frac{1}{2!} \left( \frac{\partial^2 f}{\partial x_1^2} (\Delta x_1)^2 + 2 \frac{\partial^2 f}{\partial x_1 \partial x_2} \Delta x_1 \Delta x_2 + \frac{\partial^2 f}{\partial x_2^2} (\Delta x_2)^2 \right) + \cdots
$$

## 方程求根 $f(x) = 0$
### 迭代法的收敛性
设迭代格式为 $x_{k+1} = \varphi(x_k)$，若存在 $x^ *$ 使得 $x^ *= \varphi(x^ * )$，则 $x^ *$ 为方程 $f(x) = 0$ 的一个根。

迭代法收敛定理：

* 设 $\varphi(x)$ 在 $[a, b]$ 上存在一阶连续导数，且满足：
  1. 对任意 $x \in [a, b]$，有 $\varphi(x) \in [a, b]$
  2. 存在 $0 < L < 1$，使得对任意 $x \in [a, b]$，有 $\max\limits_{a \leq x \leq b} |\varphi'(x)| \leq L < 1$

  则
  1. $x = \varphi(x)$ 在 $[a, b]$ 上有唯一实根 $x^ *$
  2. 对任意初值 $x_0 \in [a, b]$，迭代格式 $x_{k+1} = \varphi(x_k)$ 收敛于 $x^ *$，且有

$$
|x_k - x^*| \leq \frac{L}{1-L} |x_k - x_{k-1}|,\quad k = 1, 2, \cdots;
$$

$$
|x_k - x^*| \leq \frac{L^k}{1-L} |x_1 - x_0|,\quad k = 1, 2, \cdots;
$$

$$
\lim_{k \to \infty} \frac{x^* - x_{k+1}}{x^* - x_k} = \varphi'(x^*)
$$

* 设方程 $x = \varphi(x)$ 在区间 $[a, b]$ 上有根，且 $\min\limits_{a \leq x \leq b} |\varphi'(x)| \geq 1$，则对任意 $x_0 \in [a, b]$，且 $x_0 \neq x^*$，迭代格式 $x_{k+1} = \varphi(x_k)$ 发散。

### Newton 迭代法
设 $f(x)$ 在区间 $[a, b]$ 上连续，且 $f(a) f(b) < 0$，则存在 \(x^* \in (a, b)\) 使得 $f(x^*) = 0$。
取初值 $x_0 \in [a, b]$，则迭代公式为

$$
x_{k+1} = x_k - \frac{f(x_k)}{f'(x_k)}
$$

### Newton 迭代的局部收敛性
若 $\varphi(x)$ 在 \(x^*\) 附近的某个邻域内有 $p (\geq 1)$ 阶连续导数，且

$$
\varphi'(x^*) = \varphi''(x^*) = \cdots = \varphi^{(p-1)}(x^*) = 0, \quad \varphi^{(p)}(x^*) \neq 0
$$

则迭代格式在 \(x^*\) 附近为 $p$ 阶局部收敛，且有

$$
\lim_{k \to \infty} \frac{x^* - x_{k+1}}{(x^* - x_k)^p} = (-1)^{p-1} \frac{\varphi^{(p)}(x^*)}{p!}
$$

设 \(x^*\) 是方程 $f(x) = 0$ 的 $m$ 重根，则
$$
\varphi'(x^*) = 1 - \frac{1}{m} 
$$
* 当 $m = 1$，即 \(x^*\) 为方程单根时，$\varphi'(x^*) = 0$，Newton 迭代至少二阶局部收敛。
* 当 $m \geq 2$，即 \(x^*\) 为方程 $m (m \geq 2)$ 重根时，$|\varphi'(x^*)| < 1$，Newton 迭代一阶（线性）局部收敛。

补充：
$$
\lim_{k \to \infty} \frac{x^* - x_{k+1}}{(x^* - x_k)^2} = -\frac{f''(x^*)}{2 f'(x^*)}
$$
### 重根的 Newton 迭代法
设 \(x^*\) 是方程 $f(x) = 0$ 的 $m$ 重根
* 若 $m$ 已知，迭代改为：

$$
x_{k+1} = x_k - m \frac{f(x_k)}{f'(x_k)},\quad k = 0, 1, \cdots.
$$

* 若 $m$ 未知，记 $u(x) = \frac{f(x)}{f'(x)}$，此时 \(x^*\) 是方程 $u(x) = 0$ 的单根，迭代改为：

$$
x_{k+1} = x_k - \frac{u(x_k)}{u'(x_k)},\quad k = 0, 1, \cdots.
$$

## 线性方程组的数值解法
### 三角形方程组的回代法
设线性方程组 $Ax = b$ 已化为上三角形形式：
$$
\begin{cases}
a_{11} x_1 + a_{12} x_2 + \cdots + a_{1n} x_n = b_1 \\
\quad \quad \quad \vdots \\
0 \quad + 0 \quad + \cdots + a_{nn} x_n = b_n
\end{cases}
$$
则回代法求解步骤为：
1. 计算 $x_n = \frac{b_n}{a_{nn}}$
2. 对 $k = n-1, n-2, \cdots, 1$，依次计算
$$
x_k = \frac{b_k - \sum\limits_{j=k+1}^{n} a_{kj} x_j}{a_{kk}}
$$
### 列主元高斯消去法
1. 在第 k 步消元之前, 从第 k 列位于对角线以下的元素中选绝对值最大者作为主元,然后进行消元。
2. 使用回代法求解上三角形方程组 $Ux = y$，其中 $U$ 为消元后的系数矩阵，$y$ 为更新后的常数向量。
### 三对角方程组的追赶法
设三对角方程组为：
$$
\begin{bmatrix}
b_1 & c_1 & 0 & \cdots & 0 \\
a_2 & b_2 & c_2 & \cdots & 0 \\
0 & a_3 & b_3 & \cdots & 0 \\
0 & a_3 & b_3 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & a_n & b_n
\end{bmatrix}
\begin{bmatrix}
x_1 \\
x_2 \\
x_3 \\
\vdots \\
x_n
\end{bmatrix}=
\begin{bmatrix}
d_1 \\
d_2 \\
d_3 \\
\vdots \\
d_n
\end{bmatrix}
$$
则追赶法求解步骤为：
1. 消元过程：
$$
\begin{cases}
\beta_1 = b_1, y_1 = d_1 \\
l_i = \frac{a_i}{\beta_{i-1}}, \quad \beta_i = b_i - l_i c_{i-1}, \quad y_i = d_i - l_i y_{i-1}  \quad i = 2, 3, \cdots, n 
\end{cases}
$$
2. 回代过程：  
得到同解三角方程组为
$$
\begin{bmatrix}
\beta_1 & c_1 & 0 & \cdots & 0 \\
0 & \beta_2 & c_2 & \cdots & 0 \\
0 & 0 & \beta_3 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & 0 & \beta_n
\end{bmatrix}
\begin{bmatrix}
x_1 \\
x_2 \\
x_3 \\
\vdots \\
x_n
\end{bmatrix}=
\begin{bmatrix}
y_1 \\
y_2 \\
y_3 \\
\vdots \\
y_n
\end{bmatrix}
$$
计算
$$ 
\begin{cases}
x_n = \frac{y_n}{\beta_n} \\
x_i = \frac{y_i - c_i x_{i+1}}{\beta_i} \quad i = n-1, n-2, \cdots, 1
\end{cases} 
$$
### 向量范数
1. 定义：非负性、齐次性、三角不等式
2. 常用向量范数：
    * $L_1$ 范数：$||x||_ 1 = \sum\limits_{i=1}^{n} |x_i|$
    * $L_{\infty}$ 范数：$||x||_ {\infty} = \max\limits_{1 \leq i \leq n} |x_i|$
    * $L_2$ 范数：$||x||_ 2 =\sqrt{ \sum\limits_{i=1}^{n} |x_i|^2 }$
    
### 矩阵范数
1. 算子范数：$||A|| = \max\limits_{x \neq 0} \frac{||Ax||}{||x||}$
2. 谱半径：$\rho(A) = \max\limits_{1 \leq i \leq n} |\lambda_i|$，其中 $\lambda_i$ 为矩阵 $A$ 的特征值
3. 常用矩阵范数：
    * $L_1$ 范数：$||A||_ 1 = \max\limits_{1 \leq j \leq n} \sum\limits_{i=1}^{n} |a_{ij}|$
    * $L_{\infty}$ 范数：$||A||_ {\infty} = \max\limits_{1 \leq i \leq n} \sum\limits_{j=1}^{n} |a_{ij}|$
    * $L_2$ 范数：$||A||_ 2 = \sqrt{\rho(A^T A)}$
4. 谱半径与2范数的关系：$\rho(A) \leq ||A||_ 2$  
如果 $A$ 是对称矩阵，则 $\rho(A) = ||A||_ 2$
  