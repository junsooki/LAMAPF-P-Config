# src/cpp/dinic.h

## 作用
声明 Dinic 最大流结构与 Edge 数据结构。

## 数据结构

### struct Edge
- 字段：
  - `int to`：终点
  - `int rev`：反向边索引
  - `int cap`：当前残量容量
  - `int original_cap`：初始容量（仅正向边非 0，用于路径分解）

### class Dinic
- 构造：`Dinic(int n)`
  - 作用：创建包含 `n` 个节点的网络
- `void add_edge(int u, int v, int cap)`
  - 作用：添加一条容量为 `cap` 的有向边（并自动添加反向边）
- `int max_flow(int s, int t)`
  - 作用：返回从 `s` 到 `t` 的最大流
- `std::vector<std::vector<Edge>>& graph()`
  - 作用：访问内部邻接表（用于路径分解时读取/消耗流）
- `const std::vector<std::vector<Edge>>& graph() const`
  - 作用：只读访问内部邻接表

## 约束/约定
- 使用 `int` 容量；默认适配单位容量场景。
- `original_cap` 仅用于正向边，反向边为 0。
