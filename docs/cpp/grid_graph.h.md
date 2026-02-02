# src/cpp/grid_graph.h

## 作用
声明二维格子地图封装与邻接获取接口。

## 主要类型

### class GridGraph
- 构造：`GridGraph(const std::vector<std::vector<int>>& grid)`
  - 作用：从二维数组构建格子图；约定 `0=可通行`，`1=障碍`
- `int width() const` / `int height() const`
  - 作用：返回地图尺寸
- `int node_count() const`
  - 作用：返回可通行格子的数量
- `bool in_bounds(int x, int y) const`
  - 作用：判断坐标是否在地图内
- `bool passable(int x, int y) const`
  - 作用：判断是否可通行
- `int id(int x, int y) const`
  - 作用：返回自由格的内部索引（不可通行则为 -1）
- `std::pair<int,int> xy(int id) const`
  - 作用：内部索引转回坐标
- `std::vector<int> neighbors(int node_id) const`
  - 作用：返回 4 邻接可通行格的 id 列表

## 约束/约定
- 仅支持 4 邻接（上/下/左/右）。
