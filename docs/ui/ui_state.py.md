# src/py/ui/ui_state.py

## 作用
定义编辑器与可视化共享的 UI 状态数据结构。

## 主要类型

### class UIState
```python
@dataclass
class UIState:
    width: int
    height: int
    cells: List[List[int]]
    agents: List[Tuple[int,int]]
    goals: List[Tuple[int,int]]
```

### empty_state(width, height)
```python
def empty_state(width: int, height: int) -> UIState:
    """创建指定尺寸的空白地图状态。"""
```

## 约束/约定
- `cells` 取值：0=free, 1=wall, 2=shelf
