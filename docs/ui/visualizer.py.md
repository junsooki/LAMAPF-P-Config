# src/py/ui/visualizer.py

## 作用
Tkinter 可视化播放器：绘制网格与路径动画。

## 主要类型

### class PlanPlayer
- `load(paths, agents, start_timestep, end_timestep)`：加载路径与播放时间窗
- `on_finish`：播放结束回调（可用于恢复 UI 显示）
- `on_step`：每步播放回调（用于更新 timestep）
- `play() / pause() / step()`：播放控制
- `current_positions()`：返回当前帧的 agent 坐标
- `draw()`：重绘当前帧

## 约束/约定
- 路径为 `Dict[int, List[(x,y)]]`
- 播放时会覆盖静态 agent 标记
- 绘制每帧前会清理 `agent` 与 `agent_static` 图层
