# src/cpp/hlpp.h

## 作用
声明 HLPP（Highest Label Preflow Push）最大流求解器接口。

## 主要接口

### class HLPP
- `HLPP(int n)`：创建包含 `n` 个节点的残量网络。
- `add_edge(int u, int v, int cap)`：添加有向边（带反向边）。
- `int max_flow(int s, int t)`：计算从 `s` 到 `t` 的最大流。
- `graph()`：返回内部图（用于路径提取等后处理）。

## 约束/约定
- 使用 `Edge` 结构与 Dinic 相同的字段（cap/original_cap/rev）。
- 与 `flow_planner.cpp` 的模板路径提取兼容。
