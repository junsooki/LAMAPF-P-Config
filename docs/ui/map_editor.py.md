# src/py/ui/map_editor.py

## 作用
Tkinter 地图编辑器与模拟回放（sim_output.json）播放器。

## 主要函数

### run_editor(maps_dir)
```python
def run_editor(maps_dir: str) -> None:
    """启动编辑器，提供地图列表、编辑、保存与模拟回放。"""
```

## 主要交互
- 地图列表选择
- 工具选择：none/墙/货架/agent/goal/擦除（默认 none）
- 支持鼠标按住拖拽批量绘制（墙/货架/目标/擦除）
- 单击默认不进行绘制或删除
- 支持鼠标滚轮/触控板缩放画布网格（以光标为中心）
- none 模式下左键拖拽平移视图
- 按住空格可临时进入平移模式（左键拖拽）
- 左侧提供缩放滑条（以画布中心为缩放锚点）
- Load Sim：读取模拟输出 JSON 并进入回放模式
- 回放模式支持进度条、步进与回退

## 约束/约定
- 编辑器仅包含地图编辑与回放功能（无规划按钮）。
- 回放 mode 依据任务 spawn/picked 时间与当前 timestep 显示货物目标。
- Hover 状态显示使用显式成员判断，不吞掉异常。
