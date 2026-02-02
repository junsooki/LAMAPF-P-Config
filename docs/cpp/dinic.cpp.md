# src/cpp/dinic.cpp

## 作用
实现 Dinic 最大流算法，对应 `dinic.h` 的接口。

## 函数定义与作用
- `Dinic::Dinic(int n)`：初始化内部图结构与层级数组。
- `void Dinic::add_edge(int u, int v, int cap)`：加入正向/反向边并记录初始容量。
- `int Dinic::max_flow(int s, int t)`：计算最大流。
- `std::vector<std::vector<Edge>>& Dinic::graph()`：返回可修改的邻接表。
- `const std::vector<std::vector<Edge>>& Dinic::graph() const`：返回只读邻接表。
