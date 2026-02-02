# src/py/simulator_full.py

## 作用
在离线模式下模拟完整流程：读取地图、随机生成 agent、按事件驱动重规划，直到超过最大 timestep，输出轨迹与任务生命周期。

## 主要函数

### run_simulation(map_path, agent_count, max_timestep, output_path, seed, solver="dinic", debug=False)
```python
def run_simulation(map_path: str, agent_count: int, max_timestep: int, output_path: str, seed: int, solver: str = "dinic", debug: bool = False) -> None:
    """运行仿真并保存结果 JSON。"""
```
- 输入：地图路径、agent 数、最大 timestep、输出路径
- 输出：保存 JSON（agent 轨迹 + 任务生成/取走/送达时间）
 - `seed` 用于可复现随机生成
 - `solver` 选择最大流求解器（`dinic`/`hlpp`）

### ensure_tasks(...)
```python
def ensure_tasks(tasks, shelf_cells, current_timestep, agent_count, next_task_id) -> int:
    """若可用任务数小于 max(agent_count, 20%货架数)，随机补足到该下限。返回更新后的 next_task_id。"""
```

## 约束/约定
- 忽略地图里预设的 agent，仅读取障碍、货架、卸货点。
- 货架格可通行，墙不可通行。
- 任务带 `spawn_time`，仅当 `spawn_time <= current_timestep` 可被分配。
- 当第一个 agent 到达目标（取货或卸货）时触发重规划。
- 当下一次事件会跨过 `max_timestep` 时，仍执行该事件并结束模拟。
- 若规划失败会抛出错误，并包含 empty/loaded 数量、pickup 点数量、goal 数量及阶段/单独可行性诊断信息。
- 任务分配使用唯一货架位置，避免同一位置重复任务导致不可行。
- 卸货点按时间层吸收（容量为“同一时刻最多 1 人”）；模拟中 `drop_caps` 设为 1。
- 输出前会校验轨迹无点冲突与边冲突，发现冲突直接报错。
- 支持 debug 输出：打印每轮的 agent 状态与规划窗口。
- 输出 JSON 记录 `solver` 字段。
