# tests/test_sync_two_stage.py

## 作用
覆盖同步两段模型的基础可行性与不可行性判定。

## 覆盖点
- `test_sync_two_stage_feasible`：二维 2x2 网格，`tau=1` 时刻所有机器人在取货点，`T=2` 回到卸货点，要求可行。
- `test_sync_two_stage_infeasible_when_pickups_too_few`：取货点数量少于机器人数量时，要求不可行。
- `test_sync_tau_too_small_infeasible`：`tau` 小于最短到达取货点时间时，要求不可行。
- `test_sync_hlpp_solver_feasible`：HLPP 求解器在同步两段模型下可行。

## 备注
依赖 `flow_planner_cpp` 扩展模块；若未构建会直接抛出 ImportError（不跳过）。
