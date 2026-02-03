+++
date = '2026-02-03T14:51:02+08:00'
draft = true
title = 'Leetcode笔记'
categories = ['笔记']
tags = ['leetcode', '算法题']
+++

这篇笔记专注于：**LeetCode 刷题中 Java 常用集合与数据结构的“选型 + 常用 API + 典型写法”**。

## 1. Java 集合框架快速定位

刷题时常用的接口层级：

- `Collection`：`List` / `Set` / `Queue` 的父接口
- `Map`：键值对容器（不属于 `Collection`）

常用实现类速查：

| 需求 | 推荐实现 | 复杂度/特点（常见） |
| --- | --- | --- |
| 顺序表、随机访问 | `ArrayList` | `get/set` $O(1)$，尾部 `add` 均摊 $O(1)$ |
| 频繁头删/头插、双端队列 | `ArrayDeque` | 头尾 `add/remove` 均摊 $O(1)$，比 `LinkedList` 更快 |
| 需要队列/双端队列且要 `null` 也行 | `LinkedList` | 额外指针开销大；`Deque`/`Queue` 都能用 |
| 自动最小/最大值（堆） | `PriorityQueue` | `offer/poll` $O(\log n)$ |
| 有序集合/有序映射（平衡树） | `TreeSet` / `TreeMap` | 红黑树：`add/remove/contains` $O(\log n)$ |
| 去重、查找快 | `HashSet` / `HashMap` | 平均 $O(1)$，注意哈希与扩容 |

## 2. 单端队列（Queue）的特点与 Java 用法

**特点**：FIFO（先进先出），常用于 BFS、滑动窗口（配合双端队列）、生产者消费者模型等。

**推荐实现**：

- 一般队列：`ArrayDeque`（不允许 `null`）
- 也可：`LinkedList`（允许 `null`，但性能通常不如 `ArrayDeque`）

**常用方法（Queue 接口）**：

- 入队：`offer(e)`（失败返回 `false`，不抛异常） / `add(e)`（失败抛异常）
- 出队：`poll()`（空返回 `null`） / `remove()`（空抛异常）
- 查看队头：`peek()`（空返回 `null`） / `element()`（空抛异常）

示例：

```java
Queue<Integer> q = new ArrayDeque<>();
q.offer(1);
q.offer(2);
int head = q.peek();   // 1
int x = q.poll();      // 1
boolean empty = q.isEmpty();
```

## 3. 双端队列（Deque）的特点与 Java 用法

**特点**：两端都能入/出，既能当队列也能当栈；刷题中经常用于：

- 单调队列（滑动窗口最大/最小）
- 0-1 BFS（配合 `addFirst/addLast`）
- 作为栈（替代 `Stack`）

**推荐实现**：`ArrayDeque`。

**常用方法（Deque 接口）**：

- 队头：`addFirst(e)` / `offerFirst(e)` / `removeFirst()` / `pollFirst()` / `peekFirst()`
- 队尾：`addLast(e)` / `offerLast(e)` / `removeLast()` / `pollLast()` / `peekLast()`

示例（单调队列：维护递减，队头最大）：

```java
Deque<Integer> dq = new ArrayDeque<>(); // 存索引
int[] a = {1, 3, -1, -3, 5, 3, 6, 7};
int k = 3;
for (int i = 0; i < a.length; i++) {
	while (!dq.isEmpty() && a[dq.peekLast()] <= a[i]) dq.pollLast();
	dq.offerLast(i);
	if (dq.peekFirst() <= i - k) dq.pollFirst();
	if (i >= k - 1) {
		int max = a[dq.peekFirst()];
		// use max
	}
}
```

## 4. 优先队列（PriorityQueue/堆）的特点与 Java 用法

**特点**：每次能取出“最小/最大”的元素（堆结构），适用于 TopK、合并 K 路、Dijkstra、贪心等。

**Java 实现**：`PriorityQueue<E>`（默认小根堆）。

**常用方法**：

- 入堆：`offer(e)` / `add(e)`
- 出堆：`poll()`
- 查看堆顶：`peek()`
- 建堆：构造时传集合，或逐个 `offer`

小根堆示例：

```java
PriorityQueue<Integer> pq = new PriorityQueue<>();
pq.offer(3);
pq.offer(1);
pq.offer(2);
int min = pq.poll(); // 1
```

大根堆（传比较器）：

```java
PriorityQueue<Integer> maxHeap = new PriorityQueue<>((x, y) -> Integer.compare(y, x));
maxHeap.offer(3);
maxHeap.offer(1);
int max = maxHeap.peek(); // 3
```

存数组/对象时（注意比较器写法与溢出）：

```java
int[][] points = {{1, 2}, {3, 4}};
PriorityQueue<int[]> pq2 = new PriorityQueue<>((p1, p2) -> Integer.compare(p1[0], p2[0]));
pq2.offer(points[0]);
```

## 5. 栈（Stack）的特点与 Java 用法

**特点**：LIFO（后进先出），用于括号匹配、表达式求值、DFS（显式栈）、单调栈（下一个更大元素）等。

### 5.1 不推荐：`Stack`

`Stack` 是早期类（继承 `Vector`），线程安全但常带来不必要的开销；刷题更推荐用 `Deque`。

### 5.2 推荐：`ArrayDeque` 作为栈

用法（把队头当栈顶）：

```java
Deque<Integer> st = new ArrayDeque<>();
st.push(1);      // 等价 addFirst
st.push(2);
int top = st.peek(); // 2
int v = st.pop();    // 2
```

单调栈示例（下一个更大元素的经典模板之一）：

```java
int[] a = {2, 1, 2, 4, 3};
int n = a.length;
int[] nextGreater = new int[n];
Arrays.fill(nextGreater, -1);

Deque<Integer> st = new ArrayDeque<>(); // 存索引，维护递减
for (int i = 0; i < n; i++) {
	while (!st.isEmpty() && a[st.peek()] < a[i]) {
		nextGreater[st.pop()] = a[i];
	}
	st.push(i);
}
```

## 6. 二叉树（Binary Tree）的特点与 Java 表达

**特点**：每个节点最多 2 个子节点；常见题型：遍历、路径和、最近公共祖先、层序、树的直径等。

**Java 中没有通用的“二叉树标准库节点类”**，LeetCode 通常提供 `TreeNode`；自己写题时可定义：

```java
static class TreeNode {
	int val;
	TreeNode left;
	TreeNode right;
	TreeNode(int val) { this.val = val; }
}
```

### 6.1 常见遍历

- 前序：根-左-右
- 中序：左-根-右（BST 中序为有序序列）
- 后序：左-右-根
- 层序：BFS

层序遍历模板（队列）：

```java
List<List<Integer>> levelOrder(TreeNode root) {
	List<List<Integer>> res = new ArrayList<>();
	if (root == null) return res;
	Queue<TreeNode> q = new ArrayDeque<>();
	q.offer(root);
	while (!q.isEmpty()) {
		int size = q.size();
		List<Integer> level = new ArrayList<>(size);
		for (int i = 0; i < size; i++) {
			TreeNode cur = q.poll();
			level.add(cur.val);
			if (cur.left != null) q.offer(cur.left);
			if (cur.right != null) q.offer(cur.right);
		}
		res.add(level);
	}
	return res;
}
```

## 7. 二叉搜索树（BST）的特点与 Java 对应结构

**BST 特点**：

- 对任意节点：左子树所有值 < 节点值 < 右子树所有值（或按题意允许等号）
- 中序遍历结果为递增序列
- 理想情况下操作 $O(\log n)$，但普通 BST 可能退化为链表

### 7.1 LeetCode 的 BST（节点形式）

题目给 `TreeNode` 的场景下，BST 的插入/查找通常递归/迭代完成；复杂度取决于树高。

### 7.2 Java 标准库里的“平衡 BST”：`TreeMap` / `TreeSet`

刷题里需要“动态有序 + 能找前驱/后继”的能力时，往往直接用：

- `TreeSet<E>`：有序去重集合
- `TreeMap<K,V>`：有序键值映射

它们底层是**红黑树（自平衡）**，保证 `add/remove/contains` 近似 $O(\log n)$，并提供很多“有序查询”API：

```java
TreeSet<Integer> set = new TreeSet<>();
set.add(3);
set.add(1);
set.add(5);

Integer floor = set.floor(4);   // <=4 的最大值 -> 3
Integer ceil  = set.ceiling(4); // >=4 的最小值 -> 5
Integer lower = set.lower(3);   // <3 的最大值 -> 1
Integer higher= set.higher(3);  // >3 的最小值 -> 5
Integer first = set.first();
Integer last  = set.last();
```

`TreeMap` 常用：

```java
TreeMap<Integer, Integer> map = new TreeMap<>();
map.put(2, 10);
map.put(5, 20);
Map.Entry<Integer, Integer> e1 = map.floorEntry(4);   // key=2
Map.Entry<Integer, Integer> e2 = map.ceilingEntry(4); // key=5
```

## 8. 刷题中常见“坑”与建议

### 8.1 `ArrayDeque` 不允许 `null`

因此 `peek/poll` 返回 `null` 可以安全用来判断“空队列/空栈”；但不能把 `null` 当合法元素塞进去。

### 8.2 比较器不要写 `a - b`

可能溢出，推荐用：

- `Integer.compare(a, b)`
- `Long.compare(a, b)`

### 8.3 `PriorityQueue` 里元素被修改

堆依赖比较结果。如果你把已经入堆的对象字段改了，堆内顺序不会自动重建；一般做法是**不要原地修改入堆元素**，需要变化就重新 `offer`。

### 8.4 选择 Map/Set 的经验法则

- 只关心“存在性/计数”，优先 `HashMap/HashSet`
- 需要“有序 + 前驱/后继/范围”，用 `TreeMap/TreeSet`
- 需要“插入顺序稳定迭代”，用 `LinkedHashMap/LinkedHashSet`

---

后续我会把这篇笔记拆成更可检索的小节（例如“单调栈模板/0-1 BFS/TopK”），并在每节末尾挂上对应题号列表。
