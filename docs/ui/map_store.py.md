# src/py/ui/map_store.py

## 作用
管理地图文件的存储与加载，提供地图列表与读写接口。

## 主要函数

### list_maps(maps_dir)
```python
def list_maps(maps_dir: str) -> list:
    """返回 maps_dir 下的地图文件名列表（按名字排序）。"""
```

### load_map(maps_dir, name)
```python
def load_map(maps_dir: str, name: str) -> dict:
    """读取地图文件，返回包含 grid/agents/goals 的字典。"""
```

### save_map(maps_dir, name, data)
```python
def save_map(maps_dir: str, name: str, data: dict) -> str:
    """保存地图并返回最终文件名（自动补 .json）。"""
```

## 约束/约定
- 文件格式为 JSON。
- `data` 至少包含：`width`, `height`, `cells`, `agents`, `goals`。
- 读入后应将 `agents/goals` 坐标转换为 tuple 以便哈希使用。
