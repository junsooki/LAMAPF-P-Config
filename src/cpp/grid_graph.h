#pragma once

#include <utility>
#include <vector>

class GridGraph {
public:
    explicit GridGraph(const std::vector<std::vector<int>>& grid);

    int width() const;
    int height() const;
    int node_count() const;

    bool in_bounds(int x, int y) const;
    bool passable(int x, int y) const;

    int id(int x, int y) const;
    std::pair<int, int> xy(int id) const;

    std::vector<int> neighbors(int node_id) const;

private:
    int width_;
    int height_;
    std::vector<std::vector<int>> grid_;
    std::vector<std::vector<int>> id_map_;
    std::vector<std::pair<int, int>> coords_;
};
