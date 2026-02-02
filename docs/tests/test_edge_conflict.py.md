# tests/test_edge_conflict.py

## 作用
验证边冲突（对向交换）会被碰撞检测捕获。

## 主要测试
- `test_edge_swap_forbidden`：构造对向交换轨迹，期望触发碰撞检测错误。
