# src/cpp/bindings.cpp

## 作用
通过 pybind11 暴露 C++ 规划接口给 Python 调用。

## 导出接口

### flow_planner_cpp.plan_flow(...)
- 作用：调用 C++ `plan_flow`，返回可行性与路径。
- 返回：
  - `{"feasible": bool, "paths": List[List[Tuple[int,int]]]}` 
 - 参数新增 `reserved_edges`：时空边约束

### flow_planner_cpp.plan_flow_sync(...)
- 作用：调用 C++ `plan_flow_sync`，实现同步两段模型（强制 `tau` 时刻在取货点）。
- 返回：
  - `{"feasible": bool, "paths": List[List[Tuple[int,int]]]}` 

## 约束/约定
- 模块名：`flow_planner_cpp`
