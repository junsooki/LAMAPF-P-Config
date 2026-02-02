# 代码实现计划：Event-driven Lifelong Warehouse Routing

> 目标：实现一个可运行的 planner（事件触发重规划），地图为二维格子（MAPF 风格），建图与网络流用 C++，主体控制与仿真用 Python，通过 Python 调用 C++。

---

## 1. 总体架构

- **Python（主控层）**
  - 负责：数据输入/输出、仿真循环、事件触发、状态更新、窗口 T 搜索、结果拼接、日志与可视化。
  - 调用 C++：传入网格、机器人状态、目标集合、窗口 T、占用预留等；获取 flow 路径。

- **C++（核心规划层）**
  - 负责：二维网格建图、时间展开网络构建、Dinic max-flow、流路径分解。
  - 提供统一接口：`plan_loaded(...)`、`plan_empty(...)` 或通用 `solve_flow(...)`。

- **绑定方式**
  - 推荐：`pybind11`（易用、调试方便）
  - 备选：`ctypes` + 共享库（但数据序列化更复杂）

---

## 2. 文件结构规划

```
/home/yimin/research/networkflow_mapf/
├── project_goal.md
├── implementation_plan.md
├── maps/
│   ├── README.md
├── docs/
│   ├── cpp/
│   │   ├── dinic.h.md
│   │   ├── dinic.cpp.md
│   │   ├── grid_graph.h.md
│   │   ├── grid_graph.cpp.md
│   │   ├── flow_planner.h.md
│   │   ├── flow_planner.cpp.md
│   │   └── bindings.cpp.md
│   ├── py/
│       ├── planner.py.md
│       ├── simulator.py.md
│       ├── simulator_full.py.md
│       ├── map_loader.py.md
│       ├── data_types.py.md
│       └── utils.py.md
│   ├── ui/
│       ├── map_editor.py.md
│       ├── map_store.py.md
│       ├── sim_player.py.md
│       ├── visualizer.py.md
│       └── ui_state.py.md
│   └── tests/
│       ├── test_flow_cpp.py.md
│       ├── test_small_cases.py.md
│       └── test_edge_conflict.py.md
├── src/
│   ├── cpp/
│   │   ├── flow_planner.cpp
│   │   ├── flow_planner.h
│   │   ├── grid_graph.cpp
│   │   ├── grid_graph.h
│   │   ├── dinic.cpp
│   │   ├── dinic.h
│   │   └── bindings.cpp
│   └── py/
│       ├── planner.py
│       ├── simulator.py
│       ├── simulator_full.py
│       ├── map_loader.py
│       ├── data_types.py
│       └── utils.py
│   └── ui/
│       ├── map_editor.py
│       ├── map_store.py
│       ├── sim_player.py
│       ├── visualizer.py
│       └── ui_state.py
├── tests/
│   ├── test_small_cases.py
│   ├── test_flow_cpp.py
│   └── test_edge_conflict.py
├── CMakeLists.txt
└── README.md
```

---

## 3. 文档与流程强制要求（必须遵守）

### 3.1 文档要求

- **每个代码文件必须有一个一一对应的文档文件**，放在 `docs/` 目录下，目录结构与 `src/`、`tests/` 对齐。
- 文档内容必须包含：
  - **该文件对外提供的函数/类签名**（C++/Python）
  - **每个函数/类的作用描述**（输入、输出、关键约束、异常/失败条件）
  - 关键数据结构与字段说明
- 任何新增/修改函数都必须同步修改其对应的文档文件。

### 3.2 开发流程约束（写程序前与写完后）

- **写程序前**：必须先阅读对应文件的文档，确认函数签名与用途（若文档不存在则先补文档框架）。
- **写程序后**：必须更新对应文档，确保函数定义与行为描述一致。
- 若实现时调整了接口或语义，必须先更新文档，再改代码，保持文档为“单一事实来源”。

### 3.3 错误处理约束（新增）
- 不要吞掉/压制代码自身错误；默认让异常直接抛出。
- 仅允许在“补充上下文后再次抛出”的情况下捕获异常。

---

## 4. 数据结构与接口设计

### 4.1 Python 数据结构

- `GridMap`：二维障碍/空地数组，支持 4 邻接（可选含等待）。
- `RobotState`：
  - `id`
  - `pos: (x, y)`
  - `state: "Empty" | "Loaded"`
- `PlannerInput`：
  - `grid`
  - `robots: List[RobotState]`
  - `pickup_points: List[(x, y)]`
  - `drop_points: List[(x, y)]`
  - `drop_capacity: Dict[(x,y), int]`（可选）
  - `T_max` / 当前尝试的 `T`
  - `reserved: set[(x, y, t)]`（由已规划路径占用的时空点）

### 4.2 C++ 核心接口（pybind11）

- `build_time_expanded(...)`：构造时间展开图（隐式或显式）。
- `solve_flow(...)`：给定起点集合、终点集合、T、reserved，返回路径。
- `decompose_paths(...)`：从流中解出每个机器人路径。

建议统一为：

```cpp
std::vector<std::vector<std::pair<int,int>>> plan_flow(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int,int>>& starts,
    const std::vector<std::pair<int,int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int,int,int>>& reserved
);
```

---

## 5. 关键算法实现步骤（对应需求文档）

### 5.1 时间展开网络构建（C++）

- 对每个 `(cell, t)` 创建 `in` / `out` 节点。
- 加边：`in -> out`（容量 1）实现点容量。
- 运动边：
  - `out(v,t) -> in(u,t+1)`（邻居移动）
  - `out(v,t) -> in(v,t+1)`（等待）
- 预留占用：若 `(v,t)` 在 `reserved` 中，则该 `in->out` 容量设为 0。

### 5.2 送货 / 取货目标约束

- **Loaded → D**：
  - `S -> start_in` cap=1
  - `out(d,t) -> T` cap=cap(d)
- **Empty → P**：
  - 同上，仅目标集合不同
  - 每个取货点 cap=1

### 5.3 两次流策略（方案 A）

1. `plan_loaded`: 只用 Loaded 机器人与送货点
2. 记录路径占用 `(v,t)`
3. `plan_empty`: 在 reserved 上再规划 Empty 机器人

### 5.4 T 窗口搜索

- 从 `T=0` 递增到 `T_max`
- 每次：
  - 尝试 Loaded
  - 若成功再尝试 Empty
  - 两者都成功则结束，得到最小 makespan

### 5.5 路径解码与整合

- 对每个机器人分配路径序列 `[(x,y)_t]`
- 最终返回给 Python
- 若路径长度 < 当前全局 T，补等待

---

## 6. Python 主控流程

### 6.1 Planner 调用

伪流程：

```
while True:
    # 1. 构造当前状态
    E = empty robots
    L = loaded robots

    # 2. 找最小可行 T
    for T in range(T_max+1):
        paths_L = plan_loaded(..., T, reserved=empty)
        if not feasible: continue
        reserved = occupied(paths_L)
        paths_E = plan_empty(..., T, reserved)
        if feasible: break

    # 3. 合并路径
    all_paths = merge(paths_L, paths_E)

    # 4. 执行到下一次事件
    delta = earliest_event_time(all_paths)
    execute(delta)
    update_states()
```

### 6.2 状态更新

- Empty 到达取货点 → Loaded
- Loaded 到达送货点 → Empty
- 更新 P / D（若有动态变化）

---

## 7. 关键模块划分（Python）

- `planner.py`：封装 C++ 接口 + 两次流 + T 搜索
- `simulator.py`：事件驱动循环、执行、状态更新
- `map_loader.py`：读取 grid 文件（文本或 numpy）
- `data_types.py`：RobotState / Map 等数据结构
- `ui/map_editor.py`：地图编辑器（网格编辑 + 实体放置）
- `ui/map_store.py`：地图文件读写与列表管理
- `ui/visualizer.py`：规划结果可视化与播放
- `ui/ui_state.py`：编辑器 UI 状态与运行时配置

---

## 8. 可视化与编辑器需求（新增，Tkinter）

### 8.1 地图编辑
- 可编辑内容：墙（不可通行）、货架（不可通行）、agent 初始位置、goal 位置
- 支持网格点击切换地块类型，支持拖拽/点选放置 agent 与 goal
- 保存时输出标准 grid + 元数据（agents、goals、货架、墙）

### 8.2 地图存储与加载
- 地图统一存放在 `maps/` 目录
- 编辑器启动时读取 `maps/` 并显示地图列表供选择
- 允许新建地图、覆盖保存、另存为

### 8.3 编辑器与规划器集成
- 编辑器提供“开始/播放”按钮\n
- 点击开始时：\n
  - 依据当前地图随机生成任务数据（按地图中的 agent / goal / 货架 / 墙）
  - 调用 planner 执行 **单个 stage**（每个 agent 到达其下一目标）\n
  - 将生成的路径结果交给可视化播放器播放\n

### 8.4 播放要求
- 支持播放 / 暂停 / 单步\n
- 可视化显示：墙、货架、agent、goal、路径轨迹（可选）\n

---

## 9. 测试与验证

### 8.1 单元测试
- 小型 5x5 网格
- 2-3 机器人：手工可验证路径
- 用例：
  - 全 Empty
  - 全 Loaded
  - 混合 Empty/Loaded
- 覆盖点容量、等待、目标容量（送货点 cap）
- 校验：路径长度、无点冲突、目标可达

### 9.2 可视化调试
- 编辑器内置播放验证
- 可选：导出路径到文本或 matplotlib 动画

---

## 10. 实现阶段安排（建议顺序）

1. **编写/确认 docs/ 对应文档骨架**（先文档）
2. **C++ Dinic + 时间展开图**（不含 reserved；同步更新 docs）
3. **路径分解**（验证与输入一致；同步更新 docs）
4. **pybind11 绑定**（能从 Python 调用并返回路径；同步更新 docs）
5. **Python planner + T 搜索**（同步更新 docs）
6. **事件驱动仿真循环**（同步更新 docs）
7. **地图编辑器 + 可视化播放**（同步更新 docs）
8. **测试 + 可视化**（新增测试用例需更新 docs/ 与 tests/）

---

## 11. 输出与接口约定

- C++ 返回：
  - `vector<vector<pair<int,int>>>` 每个机器人路径
- Python 输出：
  - `Dict[robot_id, List[(x,y)]]`
  - 支持截断执行（只执行到事件点）

---

## 12. 可选扩展

- min-cost max-flow（优化总路径）
- 失败重试策略（交换优先级 / 增大 T）
- 更复杂的容量模型

---

完成后即可形成一个可运行的 event-driven planner，满足需求文档中的核心假设与约束。
