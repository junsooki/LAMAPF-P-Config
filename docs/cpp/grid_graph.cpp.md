# src/cpp/grid_graph.cpp

## 作用
实现 `GridGraph`，包括坐标映射、邻接获取与可通行性判断。

## 函数定义与作用
- `GridGraph::GridGraph(...)`：建立坐标到 id 映射并校验矩形网格。
- `int GridGraph::width() const` / `int GridGraph::height() const`：返回尺寸。
- `int GridGraph::node_count() const`：返回可通行格子数量。
- `bool GridGraph::in_bounds(...) const`：边界判断。
- `bool GridGraph::passable(...) const`：可通行判断。
- `int GridGraph::id(...) const`：坐标到 id。
- `std::pair<int,int> GridGraph::xy(...) const`：id 到坐标。
- `std::vector<int> GridGraph::neighbors(int node_id) const`：4 邻接列表。
