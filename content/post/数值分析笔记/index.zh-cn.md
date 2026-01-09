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
    * $L_1$ 范数：$||A||_ 1 = \max\limits_{1 \leq j \leq n} \sum\limits_{i=1}^{n} |a_{ij}| \quad$ 按列求和的最大值
    * $L_{\infty}$ 范数：$||A||_ {\infty} = \max\limits_{1 \leq i \leq n} \sum\limits_{j=1}^{n} |a_{ij}| \quad$ 按行求和的最大值
    * $L_2$ 范数：$||A||_ 2 = \sqrt{\rho(A^T A)}$  

    条件数：$cond(A) = ||A|| \cdot ||A^{-1}||$
4. 谱半径与2范数的关系：$\rho(A) \leq ||A||_ 2$  
如果 $A$ 是对称矩阵，则 $\rho(A) = ||A||_ 2$

### 迭代法求解线性方程组
将线性方程组 Ax = b 写为分量形式：
$$
\begin{cases}
a_{11} x_1 + a_{12} x_2 + \cdots + a_{1n} x_n = b_1 \\
a_{21} x_1 + a_{22} x_2 + \cdots + a_{2n} x_n = b_2 \\
\quad \quad \quad \vdots \\
a_{n1} x_1 + a_{n2} x_2 + \cdots + a_{nn} x_n = b_n
\end{cases}
$$
从第 $i$ 个方程中解出 $x_i$，得到如下同解方程组：
$$
\begin{cases}
x_1 = (b_1-a_{12}x_2-a_{13} x_3 \cdots - a_{1n} x_n)/a_{11} \\
x_2 = (b_2 - a_{21} x_1 - a_{23} x_3 - \cdots - a_{2n} x_n)/a_{22} \\
\quad \quad \quad \vdots \\
x_n = (b_n - a_{n1} x_1 - a_{n2} x_2 - \cdots - a_{n,n-1} x_{n-1})/a_{nn}
\end{cases}
$$
1. Jacobi 迭代格式：
$$
\begin{cases}
x_1^{(k+1)} = (b_1 - a_{12} x_2^{(k)} - a_{13} x_3^{(k)} - \cdots - a_{1n} x_n^{(k)})/a_{11} \\
x_2^{(k+1)} = (b_2 - a_{21} x_1^{(k)} - a_{23} x_3^{(k)} - \cdots - a_{2n} x_n^{(k)})/a_{22} \\
\quad \quad \quad \vdots \\
x_n^{(k+1)} = (b_n - a_{n1} x_1^{(k)} - a_{n2} x_2^{(k)} - \cdots - a_{n,n-1} x_{n-1}^{(k)})/a_{nn}
\end{cases}
$$
2. Gauss-Seidel 迭代格式：  
用新分量替换旧分量
$$
\begin{cases}
x_1^{(k+1)} = (b_1 - a_{12} x_2^{(k)} - a_{13} x_3^{(k)} - \cdots - a_{1n} x_n^{(k)})/a_{11} \\
x_2^{(k+1)} = (b_2 - a_{21} x_1^{(k+1)} - a_{23} x_3^{(k)} - \cdots - a_{2n} x_n^{(k)})/a_{22} \\
\quad \quad \quad \vdots \\
x_n^{(k+1)} = (b_n - a_{n1} x_1^{(k+1)} - a_{n2} x_2^{(k+1)} - \cdots - a_{n,n-1} x_{n-1}^{(k+1)})/a_{nn}
\end{cases}
$$
3. SOR 迭代格式：  
第k+1次迭代近似值和第k次迭代近似值的加权平均
$$
\begin{cases}
x_1^{(k+1)} = (1-\omega) x_1^{(k)} + \omega (b_1 - a_{12} x_2^{(k)} - a_{13} x_3^{(k)} - \cdots - a_{1n} x_n^{(k)}) /a_{11} \\
x_2^{(k+1)} = (1-\omega) x_2^{(k)} + \omega (b_2 - a_{21} x_1^{(k+1)} - a_{23} x_3^{(k)} - \cdots - a_{2n} x_n^{(k)}) /a_{22} \\
\quad \quad \quad \vdots \\
x_n^{(k+1)} = (1-\omega) x_n^{(k)} + \omega (b_n - a_{n1} x_1^{(k+1)} - a_{n2} x_2^{(k+1)} - \cdots - a_{n,n-1} x_{n-1}^{(k+1)}) /a_{nn}
\end{cases}
$$
### 迭代格式的收敛性
1. 迭代格式收敛的充分必要条件：迭代矩阵 $B$ 的谱半径 $\rho(B) < 1$
2. Jacobi 迭代格式的收敛性:
    * 谱半径判断：迭代矩阵特征方程为 $|\lambda D+L+U| = 0$ (系数矩阵的对角线乘 $\lambda$ )
    * 充分条件判断：如果系数矩阵 $A$ 严格对角占优，则 Jacobi 迭代格式收敛。
3. Gauss-Seidel 迭代格式的收敛性:
    * 谱半径判断：迭代矩阵特征方程为 $|\lambda (D+L)+U| = 0$ (系数矩阵的下三角乘 $\lambda$ )
    * 充分条件判断：如果系数矩阵 $A$ 严格对角占优，则 Gauss-Seidel 迭代格式收敛。
4. SOR 迭代格式的收敛性:
    * 收敛的必要条件：$0 < \omega < 2$ 
## 多项式插值
### 拉格朗日插值多项式
1. 定义：设插值节点为 $x_0, x_1, \cdots, x_n$，对应的函数值为 $f(x_0), f(x_1), \cdots, f(x_n)$，若存在一个次数不超过 $n$ 的多项式 $p_n(x)$，使得
$$p_n(x_i) = f(x_i), \quad i = 0, 1, \cdots, n$$
成立，则称 $p_n(x)$ 为函数 $f(x)$ 的 $n$ 次插值多项式。

2. 定理：满足上述条件的 $n$ 次多项式 $p_n(x)$ 存在且唯一。
3. 基本插值多项式：定义 $n+1$ 个基本插值多项式 $l_k(x)$，满足
$$
l_k(x_j) = \begin{cases}1, & j = k \\ 0, & j \neq k \end{cases}
$$
则 $l_k(x)$ 可表示为：
$$
l_k(x) = \prod\limits_{ \substack{i=0  \\  i \neq k}}^{n} \frac{x - x_i}{x_k - x_i}
$$
4. 利用基本插值多项式， $n$ 次 Lagrange 插值多项式为：
$$
L_n(x) = \sum\limits_{k=0}^{n} f(x_k) l_k(x)= \sum\limits_{k=0}^{n} f(x_k) \prod\limits_{ \substack{i=0  \\  i \neq k}}^{n} \frac{x - x_i}{x_k - x_i}
$$
$l_0(x), l_1(x), \cdots, l_n(x)$ 线性无关，称为 $n$ 次 Lagrange 插值基函数。  
5. 插值余项及误差估计：若函数 $f(x)$ 在区间 $[a, b]$ 上具有 $n+1$ 阶连续导数，则对任意 $x \in [a, b]$，存在 $\xi \in (a, b)$，使得
$$
R_n(x)=f(x) - L_n(x) = \frac{f^{(n+1)}(\xi)}{(n+1)!} \omega_{n+1}(x)
$$
其中 $\omega_{n+1}(x) = (x - x_0)(x - x_1) \cdots (x - x_n)$。
### Newton 插值多项式
1. 差商的定义：
$$
f[x_i] = f(x_i)
$$
$$
f[x_i, x_{i+1}, \cdots, x_{i+k}] = \frac{f[x_{i+1}, \cdots, x_{i+k}] - f[x_i, \cdots, x_{i+k-1}]}{x_{i+k} - x_i}
$$
2. $n$ 次 Newton 插值多项式定义为：
$$
L_n(x) = f[x_0] + f[x_0, x_1](x - x_0) + f[x_0, x_1, x_2](x - x_0)(x - x_1) \\+ \cdots + f[x_0, x_1, \cdots, x_n](x - x_0)(x - x_1) \cdots (x - x_{n-1})
$$
3. $k$ 阶差商与 $k$ 阶导数的关系：
$$
f[x_0, x_1, \cdots, x_k] = \frac{f^{(k)}(\eta)}{k!}, \quad \eta \in (\min(x_0, \cdots, x_k), \max(x_0, \cdots, x_k))
$$
### Hermite 插值多项式
1. 定义：给定$n+1$ 个互异节点 $x_0, x_1, \cdots, x_n$上的函数值和直到$m_i$阶的导数值，令 $m = \sum \limits_0^n(m_i+1)-1$，
若存在一个次数不超过 $m$ 的多项式 $H_m(x)$，使得
$$
\begin{cases}
H_m^{(j)}(x_i) = f^{(j)}(x_i), \quad j = 0, 1, \cdots, m_i \\
i = 0, 1, \cdots, n
\end{cases}
$$
则称 $H_m(x)$ 为函数 $f(x)$ 的 Hermite 插值多项式。  

2. 重节点插值：将上述插值问题看成是在 $m + 1$ 不同节点上的 Newton 插值, 然后取极限就成为 $n + 1$ 不同节点上的 Hermite 插值, 称之为重节点插值。  

$m$次 Hermite 插值多项式为：
$$
H_m(x) = f[x_0] + f[x_0, x_0](x - x_0) + f[x_0, x_0, x_1](x - x_0)^2 \\+
 \cdots + f[\underbrace{x_0, x_0, \cdots, x_0}_{m_0+1}, \underbrace{x_1, x_1, \cdots, x_1}_{m_1+1}, \cdots, \underbrace{x_n, x_n, \cdots, x_n}_{m_n+1}](x - x_0)^{m_0+1}(x - x_1)^{m_1+1} \cdots (x - x_n)^{m_n}
$$
重节点差商公式：
$$
f[\underbrace{x_i, x_i, \cdots, x_i}_{k+1}] = \frac{f^{(k)}(x_i)}{k!} \\
f[x_0, x_0,x_1] = \frac{f[x_0, x_1] - f[x_0, x_0]}{x_1 - x_0} 
$$
插值余项为：
$$
R_m(x) = f(x) - H_m(x) = \frac{f^{(m+1)}(\xi)}{(m+1)!} (x - x_0)^{m_0+1}(x - x_1)^{m_1+1} \cdots (x - x_n)^{m_n+1}
$$
### 3次样条插值函数
1. 定义：每段小区间上用3次多项式进行插值，且在区间上具有连续二阶导数，称为3次样条插值函数。  
要求3次样条插值函数 $S(x)$，每个小区间要确定4个参数，共要确定 $4n$ 个参数。  
在每个节点处满足下面的连续性条件:  
$S(x_j-0) = S(x_j+0),\quad S'(x_j-0) = S'(x_j+0),\quad S''(x_j-0) = S''(x_j+0), \quad j = 1, 2, \cdots, n-1$  
共有 $3(n-1)$ 个条件，加上$n+1$ 个插值条件，共有 $4n - 2$ 个条件，还要加上2个条件，才能确定 $4n$ 个参数。  
常用的附加条件有：
* (边界条件) 已知两端点的一阶导数, 即 $S'(x_0) = f'(x_0), \quad S'(x_n) = f'(x_n)$
* (边界条件) 已知两端点的二阶导数, 即 $S''(x_0) = f''(x_0), \quad S''(x_n) = f''(x_n)$
若令 $S''(x_0) = 0, \quad S''(x_n) = 0$，则称为自然边界条件。
* (连接条件) 要求 $S'''(x)$ 在 $x_1$ 和 $x_{n-1}$ 处连续，即 $S'''(x_1-0) = S'''(x_1+0), \quad S'''(x_{n-1}-0) = S'''(x_{n-1}+0)$
2. 求法：  

$S(x)=y_j+\{f[x_j,x_{j+1}]-(\frac{1}{3}M_j+\frac{1}{6}M_{j+1})h_j\}(x-x_j)+ \frac{M_j}{2}(x - x_j)^2 + \frac{M_{j+1} - M_j}{6h_j}(x - x_j)^3, \quad x \in [x_j, x_{j+1}]，j=0,1,⋯,n-1$

其中 $h_j = x_{j+1} - x_j$，$M_j = S''(x_j)$

对于边界条件 $S'(x_0) = f'(x_0), \quad S'(x_n) = f'(x_n)$，可列出如下方程组求解 $M_j$：
$$\begin{bmatrix}
2 & 1 \\
\mu_1 & 2 & \lambda_1 \\
& \mu_2 & 2 & \lambda_2 \\
& & \ddots & \ddots & \ddots \\
& & & \mu_{n-1} & 2 & \lambda_{n-1} \\
& & & & 1 & 2
\end{bmatrix}
\begin{bmatrix}
M_0 \\
M_1 \\
M_2 \\
\vdots \\
M_{n-1} \\
M_n
\end{bmatrix} =
\begin{bmatrix}
d_0 \\
d_1 \\
d_2 \\
\vdots \\
d_{n-1} \\
d_n
\end{bmatrix}
$$
对于边界条件 $S''(x_0) = f''(x_0), \quad S''(x_n) = f''(x_n)$，可列出如下方程组求解 $M_j$：
$$\begin{bmatrix}
2 & \lambda_1 \\
\mu_2 & 2 & \lambda_2 \\
& \mu_3 & 2 & \lambda_3 \\
& & \ddots & \ddots & \ddots \\
& & & \mu_{n-2} & 2 & \lambda_{n-2} \\
& & & & \mu_{n-1} & 2
\end{bmatrix}
\begin{bmatrix}
M_1 \\
M_2 \\
M_3 \\
\vdots \\
M_{n-2} \\ 
M_{n-1}
\end{bmatrix} =
\begin{bmatrix}
d_1-\mu_1 f''(x_0) \\
d_2 \\
d_3 \\
\vdots \\
d_{n-2} \\
d_{n-1}-\lambda_{n-1} f''(x_n)
\end{bmatrix}
$$
其中
$$\mu_j = \frac{h_{j-1}}{h_{j-1} + h_j}, \quad \lambda_j = \frac{h_j}{h_{j-1} + h_j}=1-\mu_j, \quad j = 1, 2, \cdots, n-1$$
$$d_0 = 6f[x_0,x_0,x_1], \quad d_n = 6f[x_{n-1},x_n,x_n]$$
$$d_j = 6 \left( f[x_{j-1}, x_j, x_{j+1}] \right), \quad j = 1, 2, \cdots, n-1$$
## 多项式逼近
### 最佳一致逼近
* 连续函数的范数：设$f\in C[a,b]$，记
$$
||f||_1 = \int_a^b |f(x)| dx,\quad
||f||_{\infty} = \max_{a \leq x \leq b} |f(x)|,\quad
||f||_2 = \sqrt{ \int_a^b |f(x)|^2 dx } $$
* 最佳一致逼近多项式  
定义：设 $f(x) \in C[a,b]$，$M_n$ 为次数不超过 $n$ 的任意多项式集合，若$\exists p_n\in M_n$，使得对任意 $q_n \in M_n$，有
$$||f - p_n||_{\infty} \leq ||f - q_n||_{\infty}$$
则称 $p_n(x)$ 为函数 $f(x)$ 在区间 $[a,b]$ 上的$n$次最佳一致逼近多项式。
* 一次最佳一致逼近多项式求法：  
设 $p_1(x) = c_0 + c_1 x$，若 $f''(x)$ 在 $(a, b)$ 内存在且保号，则 $f(x) − p_1 (x)$ 在 [a, b] 内有 3 个交错偏差点 $a, x_1 , b$, 于是可得
$$\begin{cases}
f(a) - p_1(a) = -[f(x_1)-p_1(x_1)] = f(b)-p_1(b) \\
f'(x_1) - p_1'(x_1) = 0
\end{cases}$$
### 最佳平方逼近
* 最佳平方逼近多项式  
定义：设$X$为内积空间， $f \in X$，$M$ 为$X$的子空间，$\varphi_0,\varphi_1,\cdots,\varphi_m$为$M$的一组基，若$\exists \varphi\in M_n$，使得对任意 $\psi \in M$，有
$$||f - \varphi|| \leq ||f - \psi||$$
则称 $ \varphi$ 是 $f$ 在 $M$ 中的最佳平方逼近元
* 最佳平方逼近多项式求法：  
如果$\varphi_i(x)=x^i(i=0,1,\cdots,m)$，设 $p(x) = \sum\limits_{i=0}^{m} c_i \varphi_i(x)$，则 $p(x)$ 称为 $f(x)$ 在 $[a, b]$ 上的 $m$ 次最佳平方逼近多项式  
$c_0 , c_1 , \cdots , c_m$ 是下面的 (正规) 方程组的解:
$$\begin{bmatrix}
(\varphi_0, \varphi_0) & (\varphi_0, \varphi_1) & \cdots & (\varphi_0, \varphi_m) \\
(\varphi_1, \varphi_0) & (\varphi_1, \varphi_1) & \cdots & (\varphi_1, \varphi_m) \\
\vdots & \vdots & \ddots & \vdots \\
(\varphi_m, \varphi_0) & (\varphi_m, \varphi_1) & \cdots & (\varphi_m, \varphi_m)
\end{bmatrix}
\begin{bmatrix}
c_0 \\
c_1 \\
\vdots \\
c_m
\end{bmatrix} =
\begin{bmatrix}
(f, \varphi_0) \\
(f, \varphi_1) \\
\vdots \\
(f, \varphi_m)
\end{bmatrix}$$
其中 $(f, g) = \int_a^b f(x) g(x) dx$ 为内积。
* 超定方程组的最小二乘解：  
设超定方程组为 $Ax = b$，其中 $A$ 为 $m \times n$ 矩阵，$m > n$，则其最小二乘解 $x^*$ 满足正规方程组
$$A^T A x = A^T b$$
* 离散数据的最佳平方逼近多项式：  
给定离散数据点 $(x_i, y_i), i = 0, 1, \cdots, n$，设 $p(x) = \sum\limits_{i=0}^{m} c_i \varphi_i(x)$，如果$\varphi_k(x)=x^k$，则 $p(x)$ 称为数据点的 $m$ 次最小二乘拟合多项式。
记
$$
\mathbf{\varphi}_k =
\begin{bmatrix}
\varphi_k(x_0) \\
\varphi_k(x_1) \\
\vdots \\
\varphi_k(x_n)
\end{bmatrix}, \quad
\mathbf{y} =
\begin{bmatrix}
y_0 \\
y_1 \\
\vdots \\
y_n
\end{bmatrix}
$$  
$c_0 , c_1 , \cdots , c_n$ 是下面的 (正规) 方程组的解:
$$\begin{bmatrix}
(\varphi_0, \varphi_0) & (\varphi_0, \varphi_1) & \cdots & (\varphi_0, \varphi_m) \\
(\varphi_1, \varphi_0) & (\varphi_1, \varphi_1) & \cdots & (\varphi_1, \varphi_m) \\
\vdots & \vdots & \ddots & \vdots \\
(\varphi_m, \varphi_0) & (\varphi_m, \varphi_1) & \cdots & (\varphi_m, \varphi_m)
\end{bmatrix}
\begin{bmatrix}
c_0 \\
c_1 \\
\vdots \\
c_m
\end{bmatrix} =
\begin{bmatrix}
(y, \varphi_0) \\
(y, \varphi_1) \\
\vdots \\
(y, \varphi_m)
\end{bmatrix}$$

## 数值积分与数值微分
### 插值型求积公式
1. 定义  
设有计算定积分 $I(f) = \int_a^b f(x) dx$ 的求积公式
$$I_n(f) = \sum\limits_{k=0}^{n} A_k f(x_k)$$
如果求积系数 $A_k=\int_a^b l_k(x) dx$，其中 $l_k(x)$ 为插值节点上的 Lagrange 基函数，则称 $I_n(f)$ 为插值型求积公式。  
截断误差为：
$$R_n(f) = I(f) - I_n(f) = \int_a^b \frac{f^{(n+1)}(\xi)}{(n+1)!}  \prod_{i=0}^n (x - x_i) dx, \quad \xi \in (a, b)$$
若函数 $f(x)$ 在

如果求积节点 $x_k$ 为等距节点，即
$$x_k = a + k h, \quad h = \frac{b-a}{n}, \quad k = 0, 1, \cdots, n$$
则称为 Newton-Cotes 求积公式，可写为：
$$I_n(f) = (b-a) \sum\limits_{k=0}^{n} C_{n,k} f(x_k)$$
2. 常用的等距节点插值型求积公式  
    * 梯形公式（n=1）：$$T(f) = \frac{b-a}{2} [f(a) + f(b)]$$
    * Simpson 公式（n=2）：$$S(f) = \frac{b-a}{6} [f(a) + 4f\left(\frac{a+b}{2}\right) + f(b)]$$
3. 代数精度  
* 定义：设求积公式 $I_n(f)$ 对任意次数不超过 $m$ 的多项式均精确成立，即
$$I(f) = I_n(f), \quad \forall f \in P_m$$
而至少对一个 $m+1$ 次多项式不精确成立，则称求积公式 $I_n(f)$ 的代数精度为 $m$
* 定理：
    * 求积公式 $I_n(f)$ 的代数精度至少为 $n$ $\Leftrightarrow$
该求积公式是插值型求积公式  
    * 求积公式 $I_n(f)$ 的代数精度为$m$ $\Leftrightarrow$
求积公式对 $1,x,x^2,\cdots,x^m$ 均精确成立，而对 $x^{m+1}$ 不精确成立  

4. 截断误差表达式推导
    * 梯形公式：$$R_T(f) = -\frac{(b-a)^3}{12} f''(\xi), \quad \xi \in (a, b)$$
    * Simpson 公式：$$R_S(f) = -\frac{(b-a)^5}{2880} f^{(4)}(\xi), \quad \xi \in (a, b)$$
### 复化求积公式
1. 定义  
将区间 $[a, b]$ 等分为 $n$ 个小区间，每个小区间上使用相同的求积公式进行积分近似，称为复化求积公式。  
记 $h = \frac{b-a}{n},\quad x_k = a + k h,\quad k=0,1,\cdots,n$，则复化求积公式可写为：
$$I(f) = \sum\limits_{k=0}^{n-1} \int_{x_k}^{x_{k+1}} f(x) dx$$
2. 复化梯形公式
$$T_n(f) = \sum\limits_{k=0}^{n-1} \frac{h}{2} [f(x_k) + f(x_{k+1})]$$
截断误差
$$I(f)-T_n(f) = -\frac{b-a}{12}h^2 f''(\eta), \quad \eta \in (a, b)$$
$$I(f)-T_{2n}(f) \approx \frac{1}{3} [T_{2n}(f) - T_n(f)]$$
递推关系式
$$T_{2n}(f) = \frac{1}{2} [T_n(f) + h \sum\limits_{k=0}^{n-1} f\left(x_{k+\frac{1}{2}}\right)]$$
其中$x_{k+\frac{1}{2}} = \frac{x_k + x_{k+1}}{2}$为新增的n个节点, $h=\frac{b-a}{n}$  
3. 复化 Simpson 公式
$$S_n(f) = \sum\limits_{k=0}^{n-1} \frac{h}{6} [f(x_{k}) + 4f(x_{k+\frac{1}{2}}) + f(x_{k+1})]$$
截断误差
$$I(f)-S_n(f) = -\frac{b-a}{180}\left(\frac{h}{2}\right)^4 f^{(4)}(\eta), \quad \eta \in (a, b)$$
$$I(f)-S_{2n}(f) \approx \frac{1}{15} [S_{2n}(f) - S_n(f)]$$
4. 复化求积公式的阶数
$$h\rightarrow 0,\quad I(f)-I_n(f) = O(h^p)$$
则称复化求积公式的阶数为 $p$，由上文可知复化梯形公式为 2 阶, 复化 Simpson 公式为 4 阶
### Romberg 求积方法
先利用复化梯形公式计算出一系列 $T_{2^k}(f)$，然后利用 
$$S_n(f) \approx \frac{4}{3} T_{2n}(f) - \frac{1}{3} T_n(f)$$
$$C_n(f) \approx \frac{16}{15} S_{2n}(f) - \frac{1}{15} S_n(f)$$
$$R_n(f) \approx \frac{64}{63} C_{2n}(f) - \frac{1}{63} C_n(f)$$
依次递推，直到满足精度
$$I(f) - R_n(f) \approx \frac{1}{255} [R_{2n}(f) - R_{n}(f)] < \varepsilon$$
### Gauss 求积公式
1. 定义：适当选择求积节点 $x_k$，使得求积公式的代数精度达到 $(2n+1)$，称为 Gauss 求积公式，对应的求积点 $x_k$ 称为 Gauss 点。
2. 区间[-1,1] 上的 Gauss 求积公式
当 $n=0$ 时，1 点 Gauss 求积公式为：
$$G_0(f) = 2 f(0)$$
当 $n=1$ 时，2 点 Gauss 求积公式为：
$$G_1(f) = f\left(-\frac{1}{\sqrt{3}}\right) + f\left(\frac{1}{\sqrt{3}}\right)$$
当 $n=2$ 时，3 点 Gauss 求积公式为：
$$G_2(f) = \frac{5}{9} f\left(-\sqrt{\frac{3}{5}}\right) + \frac{8}{9} f(0) + \frac{5}{9} f\left(\sqrt{\frac{3}{5}}\right)$$
3. 区间[a,b] 上的 Gauss 求积公式  
考虑区间 $[a, b]$ 上的积分
$$\int_a^b f(x) dx$$
通过变量代换
$x = \frac{a+b}{2} + \frac{b-a}{2} t$
将区间 $[a, b]$ 上的积分转化为区间 $[-1, 1]$ 上的积分，得到
$$\int_a^b f(x) dx = \frac{b-a}{2} \int_{-1}^{1} f\left(\frac{a+b}{2} + \frac{b-a}{2} t\right) dt$$
得到 Gauss 求积公式：
$$I_n(f) =  \sum\limits_{k=0}^{n}\frac{b-a}{2} \tilde{A_k} f\left(\frac{a+b}{2} + \frac{b-a}{2} t_k\right)$$
再令$$x_k = \frac{a+b}{2} + \frac{b-a}{2} t_k,\quad A_k = \frac{b-a}{2} \tilde{A_k}$$
则有
$$I_n(f) =  \sum\limits_{k=0}^{n} A_k f(x_k)$$
4. 截断误差表达式  
Gauss 求积公式的截断误差为：
$$R_n(f) = \frac{f^{(2n+2)}(\xi)}{(2n+2)!} \int_a^b \left[ \prod_{k=0}^{n} (x - x_k) \right]^2 dx, \quad \xi \in (a, b)$$
### 数值微分
1. 差商代替导数值
$$向前差商：f'(x_0) \approx \frac{f(x_0 + h) - f(x_0)}{h}$$
$$向后差商：f'(x_0) \approx \frac{f(x_0) - f(x_0 - h)}{h}$$
$$中心差商：f'(x_0) \approx \frac{f(x_0 + h) - f(x_0 - h)}{2h}$$

2. 截断误差  
将 $f(x_0 + h), f(x_0 − h)$ 在 $x_0$ 点 Taylor 展开, 可以得
$$f'(x_0) - \frac{f(x_0 + h) - f(x_0)}{h} = -\frac{h}{2} f''(x_0)+O(h^2)$$
$$f'(x_0) - \frac{f(x_0) - f(x_0 - h)}{h} = \frac{h}{2} f''(x_0)+O(h^2)$$
$$f'(x_0) - \frac{f(x_0 + h) - f(x_0 - h)}{2h} = -\frac{h^2}{6} f^{(3)}(x_0)+O(h^3)$$
3. 插值型求导公式  
建立插值多项式 $p_n(x)$作为函数 $f(x)$ 的近似，取其导数 $p_n'(x)$ 作为 $f'(x)$ 的近似，称为插值型求导公式。
## 常微分方程数值解
### Euler 方法
1. Euler 公式：
$$y_{i+1} = y_i + h f(x_i, y_i), \quad i=0,1,\cdots,n-1$$
其中 $h$ 为步长，$y_i$ 为 $y(x_i)$ 的近似值。  

2. 梯形公式：
$$y_{i+1} = y_i + \frac{h}{2} [f(x_i, y_i) + f(x_{i+1}, y_{i+1})], \quad i=0,1,\cdots,n-1$$
3. 预测校正公式(改进的Euler公式)：
$$\begin{cases}
预测：y^{(p)}_{i+1} = y_i + h f(x_i, y_i) \\
校正：y_{i+1} = y_i + \frac{h}{2} [f(x_i, y_i) + f(x_{i+1}, y^{(p)}_{i+1})]
\end{cases}$$
局部截断误差：$R_{i+1} = y(x_{i+1}) - y(x_i) - \frac{h}{2} [f(x_i, y(x_i)) + f(x_{i+1}, y(x_i)+hf(x_i,y(x_i))]$  

求法：分别求出预测公式的截断误差 $R^{（p）}_ {i+1}$ 和校正公式的截断误差 $R^{（c）}_ {i+1}$ ，则$R_{i+1}$可以通过加项减项凑出$R^{（p）}_ {i+1}$和$R^{（c）}_ {i+1}$  
若$R_{i+1}=O(h^{p+1})$，则称该公式为 $p$ 阶的。Euler 公式为 1 阶，梯形公式和预测校正公式为 2 阶。  

4. 整体截断误差：$E(h)= \max\limits_{0 \leq i \leq n} |y(x_i) - y_i^{[h]}|$
### Runge-Kutta 方法
1. r级 Runge-Kutta 公式：
$$\begin{cases}
y_{i+1} = y_i + h \sum\limits_{j=1}^{r} \alpha_j k_j \\
k_1 = f(x_i, y_i) \\
k_2 = f(x_i + \lambda_2 h, y_i + h (\mu_{21} k_1)) \\
k_3 = f(x_i + \lambda_3 h, y_i + h (\mu_{31} k_1 + \mu_{32} k_2)) \\
\quad \vdots \\
k_r = f(x_i + \lambda_j h, y_i + h (\mu_{r1} k_1 + \mu_{r2} k_2 + \cdots + \mu_{r,r-1} k_{r-1}))
\end{cases}$$
2. 2阶 Runge-Kutta 公式推导：
当 $r=2$ 时，有
$$\begin{cases}
y_{i+1} = y_i + h (\alpha_1 k_1 + \alpha_2 k_2) \\
k_1 = f(x_i, y_i) \\
k_2 = f(x_i + \lambda_2 h, y_i + h \mu_{21} k_1)
\end{cases}$$
其局部截断误差为
$$\begin{cases}
R_{i+1} = y(x_{i+1}) - y(x_i) - h (\alpha_1 K_1 + \alpha_2 K_2) \\
K_1 = f(x_i, y(x_i)) \\
K_2 = f(x_i + \lambda_2 h, y(x_i) + h \mu_{21} K_1)
\end{cases}$$

将 $y(x_{i+1})$ 在 $x_i$ 点 Taylor 展开, 将 $K_2$ 在 $(x_i, y_i)$ 点 Taylor 展开  
代入使公式具有二阶精度，即$R_{i+1}=O(h^{3})$，得
$$\begin{cases}
\alpha_1 + \alpha_2 = 1 \\
\alpha_2 \lambda_2 = \frac{1}{2} \\
\alpha_2 \mu_{21} = \frac{1}{2}
\end{cases}$$

## 偏微分方程数值解
### 抛物型方程的差分解法
1. 网格剖分
将求解区域 $[0, l] \times [0, T]$ 作等距网格剖分，设空间步长为 $h = \frac{l}{M}$，时间步长为 $\tau = \frac{T}{N}$，则网格点为
$$(x_i, t_k) = (i h, k \tau), \quad 0 \leq i \leq M, 0 \leq k \leq N$$
记 $u_i^k$ 为 $u(x_i, t_k)$ 的近似值。$r = \frac{a \tau}{h^2}$ 为步长比。  

2. 差分格式推导  
考虑下面的定解问题：
$$\frac{\partial u}{\partial t} - a \frac{\partial^2 u}{\partial x^2}=f(x,t), \quad a > 0$$
$$u(x,0) = \varphi(x), \quad x \in [0, l]$$
$$u(0,t) = \alpha(t), \quad u(l,t) = \beta(t), \quad t \in [0, T]$$
差分格式推导常用公式：
$$g'(x_0)= \frac{1}{h}[g(x_0+h)-g(x_0)] - \frac{h}{2} g''(\xi),\quad x_0 < \xi < x_0 + h;$$
$$g'(x_0)= \frac{1}{h}[g(x_0)-g(x_0-h)] + \frac{h}{2} g''(\xi),\quad x_0 - h < \xi < x_0;$$
$$g'(x_0)= \frac{1}{2h}[g(x_0+h)-g(x_0-h)] - \frac{h^2}{6} g^{(3)}(\xi),\quad x_0 - h < \xi < x_0 + h;$$
$$g''(x_0)= \frac{1}{h^2}[g(x_0+h)-2g(x_0)+g(x_0-h)] - \frac{h^2}{12} g^{(4)}(\xi),\quad x_0 - h < \xi < x_0 + h;$$
$$g(x_0)= \frac{1}{2}[g(x_0+h)+g(x_0-h)] - \frac{h^2}{2} g''(\xi),\quad x_0 - h < \xi < x_0 + h.$$

