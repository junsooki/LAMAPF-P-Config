#include "flow_planner.h"

#include "dinic.h"
#include "grid_graph.h"
#include "hlpp.h"

#include <algorithm>
#include <cctype>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace {

struct TimeNodeIndex {
    int num_cells;
    int T;

    int in_node(int cell, int t) const {
        return ((t * num_cells + cell) << 1);
    }

    int out_node(int cell, int t) const {
        return in_node(cell, t) + 1;
    }

    bool is_time_node(int node) const {
        return node >= 0 && node < (T + 1) * num_cells * 2;
    }

    bool is_in_node(int node) const {
        return is_time_node(node) && (node % 2 == 0);
    }

    std::pair<int, int> decode(int node) const {
        int time_cell = node / 2;
        int cell = time_cell % num_cells;
        int t = time_cell / num_cells;
        return {cell, t};
    }
};

int used_flow(const Edge& e) {
    if (e.original_cap <= 0) {
        return 0;
    }
    return e.original_cap - e.cap;
}

std::vector<int> multi_source_dist(const GridGraph& graph, const std::vector<int>& sources) {
    int n = graph.node_count();
    std::vector<int> dist(n, -1);
    std::queue<int> q;
    for (int s : sources) {
        if (s < 0 || s >= n) {
            continue;
        }
        if (dist[s] == 0) {
            continue;
        }
        dist[s] = 0;
        q.push(s);
    }
    while (!q.empty()) {
        int cur = q.front();
        q.pop();
        int d = dist[cur];
        for (int nb : graph.neighbors(cur)) {
            if (dist[nb] != -1) {
                continue;
            }
            dist[nb] = d + 1;
            q.push(nb);
        }
    }
    return dist;
}

std::string normalize_method(const std::string& method) {
    std::string out;
    out.reserve(method.size());
    for (unsigned char c : method) {
        out.push_back(static_cast<char>(std::tolower(c)));
    }
    return out;
}

template <typename FlowAlgo>
std::vector<std::vector<std::pair<int, int>>> extract_paths(
    FlowAlgo& flow,
    const GridGraph& grid,
    const TimeNodeIndex& indexer,
    const std::vector<int>& start_ids,
    int source,
    int sink) {
    std::vector<std::vector<std::pair<int, int>>> paths;
    auto& g = flow.graph();
    (void)source;

    for (int sid : start_ids) {
        int cur = indexer.in_node(sid, 0);
        std::vector<std::pair<int, int>> path;

        while (cur != sink) {
            int next_edge_idx = -1;
            for (int i = 0; i < static_cast<int>(g[cur].size()); ++i) {
                const Edge& e = g[cur][i];
                if (used_flow(e) > 0) {
                    next_edge_idx = i;
                    break;
                }
            }
            if (next_edge_idx < 0) {
                break;
            }

            Edge& e = g[cur][next_edge_idx];
            if (indexer.is_in_node(cur) && e.to == cur + 1) {
                auto [cell, t] = indexer.decode(cur);
                (void)t;
                auto xy = grid.xy(cell);
                path.push_back(xy);
            }

            e.cap += 1;
            g[e.to][e.rev].cap -= 1;
            cur = e.to;
        }

        paths.push_back(path);
    }

    return paths;
}

template <typename FlowAlgo>
PlanResult plan_flow_impl(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int, int, int>>& reserved,
    const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges) {
    PlanResult result;
    result.feasible = false;

    if (starts.empty()) {
        result.feasible = true;
        return result;
    }

    GridGraph graph(grid);
    int num_cells = graph.node_count();
    if (num_cells == 0) {
        return result;
    }

    std::vector<int> caps = target_caps;
    if (caps.empty()) {
        caps.assign(targets.size(), 1);
    }
    if (caps.size() != targets.size()) {
        return result;
    }

    std::vector<int> start_ids;
    start_ids.reserve(starts.size());
    for (const auto& s : starts) {
        int sid = graph.id(s.first, s.second);
        if (sid < 0) {
            return result;
        }
        start_ids.push_back(sid);
    }

    std::vector<int> target_ids;
    target_ids.reserve(targets.size());
    for (const auto& d : targets) {
        int did = graph.id(d.first, d.second);
        if (did < 0) {
            return result;
        }
        target_ids.push_back(did);
    }
    if (target_ids.empty()) {
        return result;
    }

    auto dist_start = multi_source_dist(graph, start_ids);
    auto dist_target = multi_source_dist(graph, target_ids);
    std::vector<int> earliest(num_cells, -1);
    std::vector<int> latest(num_cells, -1);
    for (int cell = 0; cell < num_cells; ++cell) {
        if (dist_start[cell] < 0 || dist_target[cell] < 0) {
            continue;
        }
        int l = T - dist_target[cell];
        if (l < 0) {
            continue;
        }
        earliest[cell] = dist_start[cell];
        latest[cell] = l;
    }
    auto active = [&](int cell, int t) {
        if (earliest[cell] < 0) {
            return false;
        }
        return t >= earliest[cell] && t <= latest[cell];
    };
    for (int sid : start_ids) {
        if (!active(sid, 0)) {
            return result;
        }
    }

    TimeNodeIndex indexer{num_cells, T};
    std::vector<std::pair<int, int>> undirected_edges;
    undirected_edges.reserve(num_cells * 2);
    for (int cell = 0; cell < num_cells; ++cell) {
        for (int nb : graph.neighbors(cell)) {
            if (cell < nb) {
                undirected_edges.push_back({cell, nb});
            }
        }
    }
    int num_edges = static_cast<int>(undirected_edges.size());

    int time_nodes = (T + 1) * num_cells * 2;
    int edge_offset = time_nodes;
    int edge_nodes = T * num_edges * 2;
    int sink = edge_offset + edge_nodes;
    int source = sink + 1;

    FlowAlgo flow(source + 1);

    std::vector<char> blocked((T + 1) * num_cells, 0);
    for (const auto& r : reserved) {
        int x, y, t;
        std::tie(x, y, t) = r;
        if (t < 0 || t > T) {
            continue;
        }
        int cid = graph.id(x, y);
        if (cid < 0) {
            continue;
        }
        blocked[t * num_cells + cid] = 1;
    }

    for (int t = 0; t <= T; ++t) {
        for (int cell = 0; cell < num_cells; ++cell) {
            if (!active(cell, t)) {
                continue;
            }
            int cap = blocked[t * num_cells + cell] ? 0 : 1;
            int in = indexer.in_node(cell, t);
            int out = indexer.out_node(cell, t);
            if (cap > 0) {
                flow.add_edge(in, out, cap);
            }
            if (t == T) {
                continue;
            }
            if (active(cell, t + 1)) {
                flow.add_edge(out, indexer.in_node(cell, t + 1), 1);
            }
        }
    }

    for (int t = 0; t < T; ++t) {
        for (int eidx = 0; eidx < num_edges; ++eidx) {
            int a = undirected_edges[eidx].first;
            int b = undirected_edges[eidx].second;
            bool move_ab = active(a, t) && active(b, t + 1);
            bool move_ba = active(b, t) && active(a, t + 1);
            if (!move_ab && !move_ba) {
                continue;
            }
            int edge_in = edge_offset + (t * num_edges + eidx) * 2;
            int edge_out = edge_in + 1;
            if (move_ab) {
                flow.add_edge(indexer.out_node(a, t), edge_in, 1);
            }
            if (move_ba) {
                flow.add_edge(indexer.out_node(b, t), edge_in, 1);
            }
            flow.add_edge(edge_in, edge_out, 1);
            if (move_ba) {
                flow.add_edge(edge_out, indexer.in_node(a, t + 1), 1);
            }
            if (move_ab) {
                flow.add_edge(edge_out, indexer.in_node(b, t + 1), 1);
            }
        }
    }

    if (!reserved_edges.empty()) {
        std::unordered_map<long long, int> edge_index;
        edge_index.reserve(num_edges * 2);
        for (int i = 0; i < num_edges; ++i) {
            int a = undirected_edges[i].first;
            int b = undirected_edges[i].second;
            long long key = (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
            edge_index[key] = i;
        }
        auto& g = flow.graph();
        for (const auto& e : reserved_edges) {
            int x1, y1, x2, y2, t;
            std::tie(x1, y1, x2, y2, t) = e;
            if (t < 0 || t >= T) {
                continue;
            }
            int id1 = graph.id(x1, y1);
            int id2 = graph.id(x2, y2);
            if (id1 < 0 || id2 < 0) {
                continue;
            }
            int a = std::min(id1, id2);
            int b = std::max(id1, id2);
            long long key = (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
            auto it = edge_index.find(key);
            if (it == edge_index.end()) {
                continue;
            }
            int eidx = it->second;
            int edge_in = edge_offset + (t * num_edges + eidx) * 2;
            int edge_out = edge_in + 1;
            for (auto& edge : g[edge_in]) {
                if (edge.to == edge_out && edge.original_cap > 0) {
                    edge.cap = 0;
                    break;
                }
            }
        }
    }

    for (int sid : start_ids) {
        flow.add_edge(source, indexer.in_node(sid, 0), 1);
    }

    for (size_t i = 0; i < target_ids.size(); ++i) {
        int tid = target_ids[i];
        int cap = caps[i];
        if (cap <= 0) {
            continue;
        }
        for (int t = 0; t <= T; ++t) {
            flow.add_edge(indexer.out_node(tid, t), sink, cap);
        }
    }

    int flow_value = flow.max_flow(source, sink);
    if (flow_value != static_cast<int>(starts.size())) {
        return result;
    }

    result.paths = extract_paths(flow, graph, indexer, start_ids, source, sink);
    result.feasible = true;
    return result;
}

template <typename FlowAlgo>
PlanResult plan_flow_sync_impl(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& pickups,
    const std::vector<std::pair<int, int>>& drops,
    const std::vector<int>& drop_caps,
    int T,
    int tau) {
    PlanResult result;
    result.feasible = false;

    if (starts.empty()) {
        result.feasible = true;
        return result;
    }
    if (tau < 0 || tau > T) {
        return result;
    }

    GridGraph graph(grid);
    int num_cells = graph.node_count();
    if (num_cells == 0) {
        return result;
    }

    std::vector<int> caps = drop_caps;
    if (caps.empty()) {
        caps.assign(drops.size(), 1);
    }
    if (caps.size() != drops.size()) {
        return result;
    }

    std::vector<int> start_ids;
    start_ids.reserve(starts.size());
    for (const auto& s : starts) {
        int sid = graph.id(s.first, s.second);
        if (sid < 0) {
            return result;
        }
        start_ids.push_back(sid);
    }

    std::vector<char> pickup_mask(num_cells, 0);
    for (const auto& p : pickups) {
        int pid = graph.id(p.first, p.second);
        if (pid < 0) {
            return result;
        }
        pickup_mask[pid] = 1;
    }

    std::vector<int> drop_ids;
    drop_ids.reserve(drops.size());
    for (const auto& d : drops) {
        int did = graph.id(d.first, d.second);
        if (did < 0) {
            return result;
        }
        drop_ids.push_back(did);
    }

    TimeNodeIndex indexer{num_cells, T};
    std::vector<std::pair<int, int>> undirected_edges;
    undirected_edges.reserve(num_cells * 2);
    for (int cell = 0; cell < num_cells; ++cell) {
        for (int nb : graph.neighbors(cell)) {
            if (cell < nb) {
                undirected_edges.push_back({cell, nb});
            }
        }
    }
    int num_edges = static_cast<int>(undirected_edges.size());

    int time_nodes = (T + 1) * num_cells * 2;
    int edge_offset = time_nodes;
    int edge_nodes = T * num_edges * 2;
    int target_offset = edge_offset + edge_nodes;
    int sink = target_offset + static_cast<int>(drops.size());
    int source = sink + 1;

    FlowAlgo flow(source + 1);

    auto dist_start = multi_source_dist(graph, start_ids);
    auto dist_drop = multi_source_dist(graph, drop_ids);
    if (pickups.empty()) {
        return result;
    }
    std::vector<int> pick_ids;
    pick_ids.reserve(pickups.size());
    for (const auto& p : pickups) {
        int pid = graph.id(p.first, p.second);
        if (pid < 0) {
            return result;
        }
        pick_ids.push_back(pid);
    }
    auto dist_pick = multi_source_dist(graph, pick_ids);

    std::vector<int> earliest(num_cells, -1);
    std::vector<int> latest(num_cells, -1);
    for (int cell = 0; cell < num_cells; ++cell) {
        if (dist_start[cell] < 0 || dist_drop[cell] < 0) {
            continue;
        }
        int l = T - dist_drop[cell];
        if (l < 0) {
            continue;
        }
        earliest[cell] = dist_start[cell];
        latest[cell] = l;
    }
    auto active = [&](int cell, int t) {
        if (earliest[cell] < 0) {
            return false;
        }
        if (t < earliest[cell] || t > latest[cell]) {
            return false;
        }
        if (t >= tau) {
            if (dist_pick[cell] < 0 || dist_pick[cell] > t - tau) {
                return false;
            }
        }
        return true;
    };
    for (int sid : start_ids) {
        if (!active(sid, 0)) {
            return result;
        }
    }

    for (int t = 0; t <= T; ++t) {
        for (int cell = 0; cell < num_cells; ++cell) {
            if (!active(cell, t)) {
                continue;
            }
            int cap = 1;
            if (t == tau && !pickup_mask[cell]) {
                cap = 0;
            }
            int in = indexer.in_node(cell, t);
            int out = indexer.out_node(cell, t);
            if (cap > 0) {
                flow.add_edge(in, out, cap);
            }
            if (t == T) {
                continue;
            }
            if (active(cell, t + 1)) {
                flow.add_edge(out, indexer.in_node(cell, t + 1), 1);
            }
        }
    }

    for (int t = 0; t < T; ++t) {
        for (int eidx = 0; eidx < num_edges; ++eidx) {
            int a = undirected_edges[eidx].first;
            int b = undirected_edges[eidx].second;
            bool move_ab = active(a, t) && active(b, t + 1);
            bool move_ba = active(b, t) && active(a, t + 1);
            if (!move_ab && !move_ba) {
                continue;
            }
            int edge_in = edge_offset + (t * num_edges + eidx) * 2;
            int edge_out = edge_in + 1;
            if (move_ab) {
                flow.add_edge(indexer.out_node(a, t), edge_in, 1);
            }
            if (move_ba) {
                flow.add_edge(indexer.out_node(b, t), edge_in, 1);
            }
            flow.add_edge(edge_in, edge_out, 1);
            if (move_ba) {
                flow.add_edge(edge_out, indexer.in_node(a, t + 1), 1);
            }
            if (move_ab) {
                flow.add_edge(edge_out, indexer.in_node(b, t + 1), 1);
            }
        }
    }

    for (int sid : start_ids) {
        flow.add_edge(source, indexer.in_node(sid, 0), 1);
    }

    for (size_t i = 0; i < drop_ids.size(); ++i) {
        int tid = drop_ids[i];
        int cap = caps[i];
        if (cap <= 0) {
            continue;
        }
        int tnode = target_offset + static_cast<int>(i);
        flow.add_edge(tnode, sink, cap);
        flow.add_edge(indexer.out_node(tid, T), tnode, 1);
    }

    int flow_value = flow.max_flow(source, sink);
    if (flow_value != static_cast<int>(starts.size())) {
        return result;
    }

    result.paths = extract_paths(flow, graph, indexer, start_ids, source, sink);
    result.feasible = true;
    return result;
}

// --- Rotation-aware helpers ---
// Direction constants: EAST=0, WEST=1, SOUTH=2, NORTH=3
static constexpr int DIR_DX[4] = {1, -1, 0, 0};
static constexpr int DIR_DY[4] = {0, 0, 1, -1};
static constexpr int ROT_NEIGHBORS[4][2] = {
    {2, 3},  // EAST -> SOUTH, NORTH
    {2, 3},  // WEST -> SOUTH, NORTH
    {0, 1},  // SOUTH -> EAST, WEST
    {0, 1},  // NORTH -> EAST, WEST
};

int delta_to_dir(int dx, int dy) {
    if (dx == 1 && dy == 0) return 0;
    if (dx == -1 && dy == 0) return 1;
    if (dx == 0 && dy == 1) return 2;
    if (dx == 0 && dy == -1) return 3;
    return -1;
}

struct RotTimeNodeIndex {
    int num_cells;
    int T;

    int in_node(int cell, int dir, int t) const {
        return ((t * num_cells * 4 + cell * 4 + dir) << 1);
    }

    int out_node(int cell, int dir, int t) const {
        return in_node(cell, dir, t) + 1;
    }

    int time_dir_node_count() const {
        return (T + 1) * num_cells * 4 * 2;
    }

    bool is_in_node(int node) const {
        return node >= 0 && node < time_dir_node_count() && (node % 2 == 0);
    }

    std::tuple<int, int, int> decode(int node) const {
        int idx = node / 2;
        int dir = idx % 4;
        idx /= 4;
        int cell = idx % num_cells;
        int t = idx / num_cells;
        return {cell, dir, t};
    }
};

template <typename FlowAlgo>
std::pair<std::vector<std::vector<std::pair<int, int>>>, std::vector<std::vector<int>>>
extract_paths_rot(
    FlowAlgo& flow,
    const GridGraph& grid,
    const RotTimeNodeIndex& indexer,
    const std::vector<int>& start_ids,
    const std::vector<int>& start_dirs,
    int source,
    int sink) {

    std::vector<std::vector<std::pair<int, int>>> paths;
    std::vector<std::vector<int>> path_dirs;
    auto& g = flow.graph();

    for (size_t i = 0; i < start_ids.size(); ++i) {
        int sid = start_ids[i];
        int sdir = start_dirs[i];
        int cur = indexer.in_node(sid, sdir, 0);
        std::vector<std::pair<int, int>> path;
        std::vector<int> dirs;

        while (cur != sink) {
            int next_edge_idx = -1;
            for (int j = 0; j < static_cast<int>(g[cur].size()); ++j) {
                const Edge& e = g[cur][j];
                if (used_flow(e) > 0) {
                    next_edge_idx = j;
                    break;
                }
            }
            if (next_edge_idx < 0) break;

            Edge& e = g[cur][next_edge_idx];
            if (indexer.is_in_node(cur) && e.to == cur + 1) {
                auto [cell, dir, t] = indexer.decode(cur);
                (void)t;
                path.push_back(grid.xy(cell));
                dirs.push_back(dir);
            }

            e.cap += 1;
            g[e.to][e.rev].cap -= 1;
            cur = e.to;
        }

        paths.push_back(path);
        path_dirs.push_back(dirs);
    }

    return {paths, path_dirs};
}

template <typename FlowAlgo>
PlanResult plan_flow_rot_impl(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<int>& start_dirs,
    const std::vector<std::pair<int, int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int, int, int>>& reserved,
    const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges) {

    PlanResult result;
    result.feasible = false;

    if (starts.empty()) {
        result.feasible = true;
        return result;
    }

    GridGraph graph(grid);
    int num_cells = graph.node_count();
    if (num_cells == 0) return result;

    std::vector<int> caps = target_caps;
    if (caps.empty()) caps.assign(targets.size(), 1);
    if (caps.size() != targets.size()) return result;
    if (start_dirs.size() != starts.size()) return result;

    std::vector<int> start_ids;
    start_ids.reserve(starts.size());
    for (const auto& s : starts) {
        int sid = graph.id(s.first, s.second);
        if (sid < 0) return result;
        start_ids.push_back(sid);
    }

    std::vector<int> target_ids;
    target_ids.reserve(targets.size());
    for (const auto& d : targets) {
        int did = graph.id(d.first, d.second);
        if (did < 0) return result;
        target_ids.push_back(did);
    }
    if (target_ids.empty()) return result;

    // Active cell pruning (position-based, still valid lower bounds)
    auto dist_start = multi_source_dist(graph, start_ids);
    auto dist_target = multi_source_dist(graph, target_ids);
    std::vector<int> earliest(num_cells, -1);
    std::vector<int> latest(num_cells, -1);
    for (int cell = 0; cell < num_cells; ++cell) {
        if (dist_start[cell] < 0 || dist_target[cell] < 0) continue;
        int l = T - dist_target[cell];
        if (l < 0) continue;
        earliest[cell] = dist_start[cell];
        latest[cell] = l;
    }
    auto active = [&](int cell, int t) {
        if (earliest[cell] < 0) return false;
        return t >= earliest[cell] && t <= latest[cell];
    };
    for (int sid : start_ids) {
        if (!active(sid, 0)) return result;
    }

    RotTimeNodeIndex indexer{num_cells, T};

    // Build undirected edges with direction info
    struct DirEdge {
        int a, b;
        int dir_ab, dir_ba;
    };
    std::vector<DirEdge> undirected_edges;
    undirected_edges.reserve(num_cells * 2);
    for (int cell = 0; cell < num_cells; ++cell) {
        auto [cx, cy] = graph.xy(cell);
        for (int nb : graph.neighbors(cell)) {
            if (cell < nb) {
                auto [nx, ny] = graph.xy(nb);
                int dab = delta_to_dir(nx - cx, ny - cy);
                int dba = delta_to_dir(cx - nx, cy - ny);
                undirected_edges.push_back({cell, nb, dab, dba});
            }
        }
    }
    int num_edges = static_cast<int>(undirected_edges.size());

    int time_dir_nodes = (T + 1) * num_cells * 4 * 2;
    int edge_offset = time_dir_nodes;
    int edge_nodes = T * num_edges * 2;
    int sink = edge_offset + edge_nodes;
    int source = sink + 1;

    FlowAlgo flow(source + 1);

    // Blocked cells from reservations (position-based, blocks all 4 dirs)
    std::vector<char> blocked((T + 1) * num_cells, 0);
    for (const auto& r : reserved) {
        int x, y, t;
        std::tie(x, y, t) = r;
        if (t < 0 || t > T) continue;
        int cid = graph.id(x, y);
        if (cid < 0) continue;
        blocked[t * num_cells + cid] = 1;
    }

    // Vertex capacity + wait + rotation edges
    for (int t = 0; t <= T; ++t) {
        for (int cell = 0; cell < num_cells; ++cell) {
            if (!active(cell, t)) continue;
            bool is_blocked = blocked[t * num_cells + cell] != 0;
            for (int dir = 0; dir < 4; ++dir) {
                int in = indexer.in_node(cell, dir, t);
                int out = indexer.out_node(cell, dir, t);
                if (!is_blocked) {
                    flow.add_edge(in, out, 1);
                }
                if (t == T) continue;
                if (!active(cell, t + 1)) continue;
                // Wait: same direction
                flow.add_edge(out, indexer.in_node(cell, dir, t + 1), 1);
                // Rotate 90 degrees
                flow.add_edge(out, indexer.in_node(cell, ROT_NEIGHBORS[dir][0], t + 1), 1);
                flow.add_edge(out, indexer.in_node(cell, ROT_NEIGHBORS[dir][1], t + 1), 1);
            }
        }
    }

    // Move edges through undirected edge intermediaries
    for (int t = 0; t < T; ++t) {
        for (int eidx = 0; eidx < num_edges; ++eidx) {
            const auto& ue = undirected_edges[eidx];
            bool move_ab = active(ue.a, t) && active(ue.b, t + 1);
            bool move_ba = active(ue.b, t) && active(ue.a, t + 1);
            if (!move_ab && !move_ba) continue;

            int edge_in = edge_offset + (t * num_edges + eidx) * 2;
            int edge_out = edge_in + 1;

            if (move_ab) {
                flow.add_edge(indexer.out_node(ue.a, ue.dir_ab, t), edge_in, 1);
            }
            if (move_ba) {
                flow.add_edge(indexer.out_node(ue.b, ue.dir_ba, t), edge_in, 1);
            }
            flow.add_edge(edge_in, edge_out, 1);
            if (move_ba) {
                flow.add_edge(edge_out, indexer.in_node(ue.a, ue.dir_ba, t + 1), 1);
            }
            if (move_ab) {
                flow.add_edge(edge_out, indexer.in_node(ue.b, ue.dir_ab, t + 1), 1);
            }
        }
    }

    // Reserved edges
    if (!reserved_edges.empty()) {
        std::unordered_map<long long, int> edge_index;
        edge_index.reserve(num_edges * 2);
        for (int i = 0; i < num_edges; ++i) {
            int a = undirected_edges[i].a;
            int b = undirected_edges[i].b;
            long long key = (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
            edge_index[key] = i;
        }
        auto& g = flow.graph();
        for (const auto& e : reserved_edges) {
            int x1, y1, x2, y2, t;
            std::tie(x1, y1, x2, y2, t) = e;
            if (t < 0 || t >= T) continue;
            int id1 = graph.id(x1, y1);
            int id2 = graph.id(x2, y2);
            if (id1 < 0 || id2 < 0) continue;
            int a = std::min(id1, id2);
            int b = std::max(id1, id2);
            long long key = (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
            auto it = edge_index.find(key);
            if (it == edge_index.end()) continue;
            int eidx = it->second;
            int ei = edge_offset + (t * num_edges + eidx) * 2;
            int eo = ei + 1;
            for (auto& edge : g[ei]) {
                if (edge.to == eo && edge.original_cap > 0) {
                    edge.cap = 0;
                    break;
                }
            }
        }
    }

    // Source edges
    for (size_t i = 0; i < start_ids.size(); ++i) {
        int sd = start_dirs[i];
        if (sd < 0 || sd >= 4) sd = 0;
        flow.add_edge(source, indexer.in_node(start_ids[i], sd, 0), 1);
    }

    // Sink edges: any direction at target is acceptable
    for (size_t i = 0; i < target_ids.size(); ++i) {
        int tid = target_ids[i];
        int cap = caps[i];
        if (cap <= 0) continue;
        for (int t = 0; t <= T; ++t) {
            for (int dir = 0; dir < 4; ++dir) {
                flow.add_edge(indexer.out_node(tid, dir, t), sink, cap);
            }
        }
    }

    int flow_value = flow.max_flow(source, sink);
    if (flow_value != static_cast<int>(starts.size())) return result;

    auto [paths, path_dirs] = extract_paths_rot(flow, graph, indexer, start_ids, start_dirs, source, sink);
    result.paths = std::move(paths);
    result.path_dirs = std::move(path_dirs);
    result.feasible = true;
    return result;
}

}  // namespace

PlanResult plan_flow(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int, int, int>>& reserved,
    const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges) {
    return plan_flow_impl<Dinic>(grid, starts, targets, target_caps, T, reserved, reserved_edges);
}

PlanResult plan_flow_with_method(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int, int, int>>& reserved,
    const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges,
    const std::string& method) {
    std::string key = normalize_method(method);
    if (key.empty() || key == "dinic") {
        return plan_flow_impl<Dinic>(grid, starts, targets, target_caps, T, reserved, reserved_edges);
    }
    if (key == "hlpp") {
        return plan_flow_impl<HLPP>(grid, starts, targets, target_caps, T, reserved, reserved_edges);
    }
    throw std::invalid_argument("Unknown max-flow method: " + method);
}

PlanResult plan_flow_sync(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& pickups,
    const std::vector<std::pair<int, int>>& drops,
    const std::vector<int>& drop_caps,
    int T,
    int tau) {
    return plan_flow_sync_impl<Dinic>(grid, starts, pickups, drops, drop_caps, T, tau);
}

PlanResult plan_flow_sync_with_method(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& pickups,
    const std::vector<std::pair<int, int>>& drops,
    const std::vector<int>& drop_caps,
    int T,
    int tau,
    const std::string& method) {
    std::string key = normalize_method(method);
    if (key.empty() || key == "dinic") {
        return plan_flow_sync_impl<Dinic>(grid, starts, pickups, drops, drop_caps, T, tau);
    }
    if (key == "hlpp") {
        return plan_flow_sync_impl<HLPP>(grid, starts, pickups, drops, drop_caps, T, tau);
    }
    throw std::invalid_argument("Unknown max-flow method: " + method);
}

PlanResult plan_flow_rot(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<int>& start_dirs,
    const std::vector<std::pair<int, int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int, int, int>>& reserved,
    const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges) {
    return plan_flow_rot_impl<Dinic>(grid, starts, start_dirs, targets, target_caps, T, reserved, reserved_edges);
}

PlanResult plan_flow_rot_with_method(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<int>& start_dirs,
    const std::vector<std::pair<int, int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int, int, int>>& reserved,
    const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges,
    const std::string& method) {
    std::string key = normalize_method(method);
    if (key.empty() || key == "dinic") {
        return plan_flow_rot_impl<Dinic>(grid, starts, start_dirs, targets, target_caps, T, reserved, reserved_edges);
    }
    if (key == "hlpp") {
        return plan_flow_rot_impl<HLPP>(grid, starts, start_dirs, targets, target_caps, T, reserved, reserved_edges);
    }
    throw std::invalid_argument("Unknown max-flow method: " + method);
}
