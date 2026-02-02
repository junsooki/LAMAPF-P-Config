# src/cpp/hlpp.cpp

## 作用
实现 HLPP（Highest Label Preflow Push）最大流算法，用于替代 Dinic。

## 核心要点
- 维护高度 `height_` 与超额流 `excess_`。
- 使用 bucket（按高度分组）选择当前最高标号活跃点。
- 支持 gap heuristic：当某高度层为空时将更高层直接设为无穷高度。
- 使用一次 `global_relabel` 初始化高度（从汇点反向 BFS）。

## 与系统的交互
- 被 `flow_planner.cpp` 通过模板参数调用。
- 在 Python 侧通过 `plan_flow(..., method="hlpp")` 或 `plan_flow_sync(..., method="hlpp")` 选择。
