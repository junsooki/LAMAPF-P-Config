# tests/test_flow_cpp.py

## 作用
验证 C++ `plan_flow` 的基础可行性与路径格式。

## 主要测试
- `test_single_agent_reaches_target`：单机器人在无障碍格子内到达目标。
- `test_two_agents_no_conflict`：两机器人在小网格内同时规划且无点冲突。
- `test_same_target_different_times`：两机器人可在不同时间到达同一目标点。
- `test_unreachable_target_infeasible`：不可达目标必须返回不可行。
- `test_hlpp_solver_feasible`：HLPP 求解器可在简单场景下得到可行解。

## 断言点
- `feasible == True`
- 路径起点与终点合法
- 路径长度 <= T+1
- 无点冲突
- 边冲突由 `test_edge_conflict.py` 覆盖

## 备注
依赖 `flow_planner_cpp` 扩展模块；若未构建会直接抛出 ImportError（不跳过）。
