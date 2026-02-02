# src/py/map_loader.py

## 作用
加载二维格子地图（文本或数组格式），输出标准 grid 表示。

## 主要函数

### load_grid_from_txt(path)
```python
def load_grid_from_txt(path):
    """读取文本地图，输出 grid (0=free, 1=blocked)。"""
```

### normalize_grid(grid)
```python
def normalize_grid(grid):
    """将输入转为 0/1 网格，保证矩形与数据合法性。"""
```
