# tests/test_sync_planner_guard.py

## 作用
验证同步模型在 `goals < agents` 时会直接抛错，而不是进入搜索。

## 主要测试
- `test_sync_requires_goals_ge_agents`：当卸货点数量少于 agent 数时，`plan_round_sync` 必须抛出 `RuntimeError`。
