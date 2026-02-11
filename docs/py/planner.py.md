# src/py/planner.py

## 作用
封装两次流（Loaded/Empty）与窗口 T 搜索，提供单轮 event-driven 规划接口。

## 主要函数

### plan_round(...)
```python
def plan_round(grid, robots, pickup_points, drop_points, drop_caps, T_max, method="dinic"):
    """返回单轮规划结果（最小可行 T），并给出各机器人路径。"""
```
- 输出：`(T, paths_by_id)`；若不可行返回 `(None, {})`
 - 约定：返回的路径会补齐到长度 `T+1`

### search_min_T(...)
```python
def search_min_T(...):
    """指数扩张找到上界后在区间内二分，返回最小可行 T 与路径。"""
```

### build_reserved_vertices(paths)
```python
def build_reserved_vertices(paths):
    """由路径集合构造 (x,y,t) 占用集合。"""
```

### build_reserved_edges(paths)
```python
def build_reserved_edges(paths):
    """由路径集合构造 (x1,y1,x2,y2,t) 边占用集合。"""
```

### search_min_T_sync(...)
```python
def search_min_T_sync(...):
    """同步两段模型：先指数扩张 T 找到可行上界，再在线性区间内枚举 (T, tau)。"""
```
- 参数：
  - `verbose`：打印搜索进度
  - `progress_every`：每隔多少个 `tau` 打印一次
  - `method`：最大流求解器（`dinic`/`hlpp`）
  - `parallel_workers`：总线程预算（同时用于 `T` 与 `tau`）
  - `parallel_T_workers`：并行搜索 `T` 的最大线程数（>1 时启用）

### plan_round_sync(...)
```python
def plan_round_sync(..., method="dinic", parallel_workers=1, parallel_T_workers=1):
    """同步模型：返回 (T, tau, paths)，强制所有机器人在 tau 取货、在 T 卸货。"""
```

## 说明
- Empty 阶段会锁定 Loaded 阶段的占用顶点与边，避免跨阶段点/边冲突。
- Loaded/Empty 各自阶段会先求本阶段最小可行 T，减少过度占用时间窗。
- 卸货点使用“按时间吸收”语义（不再是总容量 gate）。
- 同步模型要求同一时刻取货与卸货，因此需要 `|goals| >= agent_count` 才可能可行。
- 同步搜索会用 BFS 距离剪枝：\n  - `tau >= max_i dist(start_i, pickup)`\n  - `T - tau >= k-th smallest dist(pickup, drop)`（k=agent 数）
- BFS 使用网格邻接缓存与距离缓存（对同一地图/目标集合重复调用更快）。
- `verbose=True` 时会打印同步搜索的 (T, tau) 进度。

### explain_infeasible(...)
```python
def explain_infeasible(...):
    """返回 loaded-first / empty-first 两种顺序的不可行原因。"""
```

## 约束/约定
- 路径按机器人 id 输出，时间从 t0=0 开始。
- 模块会尝试从 `build/` 目录加载 `flow_planner_cpp` 扩展。
- `method` 透传到 C++ 最大流实现（`dinic` 或 `hlpp`）。
- 当 `parallel_T_workers > 1` 时，会先并行 `T`；剩余线程预算按当前并行的 `T` 数量均分给 `tau` 搜索（若均分后 ≤1，则 `tau` 仍按串行执行）。
