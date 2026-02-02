#include "grid_graph.h"

#include <stdexcept>

GridGraph::GridGraph(const std::vector<std::vector<int>>& grid)
    : width_(0), height_(0), grid_(grid) {
    height_ = static_cast<int>(grid_.size());
    width_ = height_ > 0 ? static_cast<int>(grid_[0].size()) : 0;
    id_map_.assign(height_, std::vector<int>(width_, -1));
    coords_.clear();
    int next_id = 0;
    for (int y = 0; y < height_; ++y) {
        if (static_cast<int>(grid_[y].size()) != width_) {
            throw std::runtime_error("Grid rows must have equal width");
        }
        for (int x = 0; x < width_; ++x) {
            if (grid_[y][x] == 0) {
                id_map_[y][x] = next_id++;
                coords_.push_back({x, y});
            }
        }
    }
}

int GridGraph::width() const {
    return width_;
}

int GridGraph::height() const {
    return height_;
}

int GridGraph::node_count() const {
    return static_cast<int>(coords_.size());
}

bool GridGraph::in_bounds(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
}

bool GridGraph::passable(int x, int y) const {
    if (!in_bounds(x, y)) {
        return false;
    }
    return grid_[y][x] == 0;
}

int GridGraph::id(int x, int y) const {
    if (!in_bounds(x, y)) {
        return -1;
    }
    return id_map_[y][x];
}

std::pair<int, int> GridGraph::xy(int id) const {
    if (id < 0 || id >= static_cast<int>(coords_.size())) {
        return {-1, -1};
    }
    return coords_[id];
}

std::vector<int> GridGraph::neighbors(int node_id) const {
    std::vector<int> result;
    if (node_id < 0 || node_id >= static_cast<int>(coords_.size())) {
        return result;
    }
    auto [x, y] = coords_[node_id];
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};
    for (int k = 0; k < 4; ++k) {
        int nx = x + dx[k];
        int ny = y + dy[k];
        if (passable(nx, ny)) {
            int nid = id(nx, ny);
            if (nid >= 0) {
                result.push_back(nid);
            }
        }
    }
    return result;
}
