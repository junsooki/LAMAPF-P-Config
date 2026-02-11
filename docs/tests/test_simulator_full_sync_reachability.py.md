# tests/test_simulator_full_sync_reachability.py

## 作用
验证 `reachable_cells` 会被墙阻隔，避免不可达区域被当作可达起点。

## 测试

### test_reachable_cells_respects_walls
```python
def test_reachable_cells_respects_walls():
    ...
```
- 构造被墙隔开的区域，确保只返回与货架连通的可达格。
