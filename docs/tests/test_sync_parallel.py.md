# tests/test_sync_parallel.py

## 作用
验证同步两段模型在并行搜索下的结果一致性与不可行情形。

## 覆盖点
- `test_parallel_T_matches_serial`：并行 T 搜索与串行结果一致（T=2, tau=1）。
- `test_parallel_tau_matches_serial`：并行 tau 搜索与串行结果一致（T=2, tau=1）。
- `test_parallel_search_infeasible`：不可行场景在并行搜索下仍返回 `None`。

## 备注
依赖 `flow_planner_cpp` 扩展模块与 `planner.search_min_T_sync`。
