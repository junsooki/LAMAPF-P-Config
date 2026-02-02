# tests/test_small_cases.py

## 作用
验证 Python 规划器（两次流 + T 搜索）在小案例中的可行性。

## 主要测试
- `test_plan_round_mixed`：混合 Empty/Loaded 的最小可行窗口返回非空路径。
- `test_plan_round_all_empty`：全 Empty 情况下可行规划并无点冲突。

## 断言点
- `T` 非空且路径字典不空
- 路径合法、无点冲突
- 路径终点属于对应目标集合

## 备注
依赖 `flow_planner_cpp` 扩展模块；若未构建会直接抛出 ImportError（不跳过）。
