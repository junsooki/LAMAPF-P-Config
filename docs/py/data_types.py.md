# src/py/data_types.py

## 作用
定义核心数据结构与枚举。

## 主要类型

### class RobotState
```python
@dataclass
class RobotState:
    id: int
    pos: Tuple[int,int]
    state: str  # "Empty" or "Loaded"
```

### class PlannerState
```python
@dataclass
class PlannerState:
    grid: List[List[int]]
    robots: List[RobotState]
    pickup_points: List[Tuple[int,int]]
    drop_points: List[Tuple[int,int]]
    drop_caps: Dict[Tuple[int,int], int]
```
