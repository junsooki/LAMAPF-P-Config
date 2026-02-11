# src/py/simulator_full_sync.py

## 作用
使用“同步两段模型”进行离线仿真：每一轮强制所有机器人在同一时刻到达取货点（tau），并在时刻 T 到达卸货点，然后进入下一轮。

## 主要函数

### run_simulation(map_path, agent_count, max_timestep, output_path, seed, solver="dinic", workers=1, t_workers=1, debug=False, debug_every=25)
```python
def run_simulation(map_path: str, agent_count: int, max_timestep: int, output_path: str, seed: int, solver: str = "dinic", workers: int = 1, t_workers: int = 1, debug: bool = False, debug_every: int = 25) -> None:
    """运行同步两段仿真并保存结果 JSON。"""
```
- 输入：地图路径、agent 数、最大 timestep、输出路径
- 输出：保存 JSON（agent 轨迹 + 任务生成/取走/送达时间）
- `seed` 用于可复现随机生成
- `solver` 选择最大流求解器（`dinic`/`hlpp`）
- `workers` 为总线程预算（同时用于 `T` 与 `tau`）
- `t_workers` 为并行搜索 `T` 的最大线程数（>1 时启用，剩余预算均分给 `tau`）

### ensure_tasks(...)
```python
def ensure_tasks(tasks, shelf_cells, current_timestep, agent_count, next_task_id) -> int:
    """若可用任务数小于 max(agent_count, 20%货架数)，随机补足到该下限。返回更新后的 next_task_id。"""
```

## 约束/约定
- 忽略地图里预设的 agent，仅读取障碍、货架、卸货点。
- 货架格可通行，墙不可通行。
- 初始 agent 位置仅从“可到达任一货架”的可通行格中采样，避免生成不可达起点。
- 任务带 `spawn_time`，仅当 `spawn_time <= current_timestep` 可被分配。
- 每轮用 `plan_round_sync` 得到 `(T, tau)`，并执行完整轮次（不做事件触发重规划）。
- `tau` 时刻标记任务被取走，`T` 时刻标记送达。
- 同步模型要求 `|goals| >= agent_count`，否则直接不可行。
- 当下一轮会跨过 `max_timestep` 时，仍执行该轮并结束模拟。
- 每轮会校验同步规划结果（路径长度、取货/送达时刻、移动合法性、无点/边冲突）。
- 输出前会校验轨迹无点冲突与边冲突，发现冲突直接报错。
- 输出包含统计信息 `stats`：吞吐量、平均等待/送达时间、任务积压、空转比例等。
- 卸货点按时间层吸收；同步模型中 `drop_caps` 设为 1。
- 支持 debug 输出：打印重规划时刻与 `(T, tau)` 搜索进度。
- 输出 JSON 记录 `solver`、`solver_workers`（总线程预算）与 `solver_t_workers`（T 并行上限）字段。
