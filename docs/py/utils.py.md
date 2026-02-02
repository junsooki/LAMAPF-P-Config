# src/py/utils.py

## 作用
提供通用工具函数（路径处理、校验）。

## 主要函数

### pad_path(path, T)
```python
def pad_path(path, T):
    """用最后位置补齐到长度 T+1。"""
```

### validate_paths(paths, grid)
```python
def validate_paths(paths, grid):
    """检查路径是否越界、是否在障碍上、是否有点冲突。"""
```

### has_edge_conflict(paths)
```python
def has_edge_conflict(paths):
    """检查是否存在对向交换的边冲突。"""
```
