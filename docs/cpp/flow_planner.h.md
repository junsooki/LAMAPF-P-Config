# src/cpp/flow_planner.h

## 作用
声明时间展开网络规划接口与结果结构。

## 主要接口

### struct PlanResult
- 字段：
  - `bool feasible`：是否达到最大流 == 起点数量
  - `std::vector<std::vector<std::pair<int,int>>> paths`：每个机器人路径（按输入 starts 顺序）

### PlanResult plan_flow(...)
```cpp
PlanResult plan_flow(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int,int>>& starts,
    const std::vector<std::pair<int,int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int,int,int>>& reserved,
    const std::vector<std::tuple<int,int,int,int,int>>& reserved_edges
);
```
- 作用：在时间窗口 `T` 内构建时间展开图并求解最大流
- 目标点采用按时间吸收机制（每个时间层容量限制）；详情见 `flow_planner.cpp.md`

### PlanResult plan_flow_sync(...)
```cpp
PlanResult plan_flow_sync(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int,int>>& starts,
    const std::vector<std::pair<int,int>>& pickups,
    const std::vector<std::pair<int,int>>& drops,
    const std::vector<int>& drop_caps,
    int T,
    int tau
);
```
- 作用：同步两段模型的 max-flow，可选 `tau` 作为强制取货时刻
