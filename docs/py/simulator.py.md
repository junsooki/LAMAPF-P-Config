# src/py/simulator.py

## 作用
事件驱动仿真循环：执行路径到下一事件，更新机器人状态并触发重规划。

## 主要函数

### run_simulation(...)
```python
def run_simulation(initial_state, planner, max_steps):
    """运行事件驱动仿真，返回轨迹或最终状态。"""
```
 - 事件时间由机器人路径中首次到达目标集合的位置决定。

### apply_paths(...)
```python
def apply_paths(state, paths, steps):
    """执行若干步动作并更新位置。"""
```

### update_states_on_event(...)
```python
def update_states_on_event(state, pickup_points, drop_points):
    """在事件发生点更新机器人 Empty/Loaded 状态。"""
```
