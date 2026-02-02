# src/py/ui/sim_player.py

## 作用
读取模拟输出 JSON，并提供基于 timestep 的播放与回退控制。

## 主要函数

### load_sim_output(path)
```python
def load_sim_output(path: str) -> dict:
    """加载模拟输出 JSON，返回字典结构。"""
```

## 约束/约定
- 输出 JSON 格式与 `simulator_full.py` 一致。
- 必须包含 `max_timestep` 与 `agents` 轨迹信息。
