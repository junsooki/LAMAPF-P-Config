#include "flow_planner.h"

#include "dinic.h"
#include "grid_graph.h"

#include <algorithm>
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

std::vector<std::vector<std::pair<int, int>>> extract_paths(
    Dinic& dinic,
    const GridGraph& grid,
    const TimeNodeIndex& indexer,
    const std::vector<int>& start_ids,
    int source,
    int sink) {
    std::vector<std::vector<std::pair<int, int>>> paths;
    auto& g = dinic.graph();
    (void)source;

    for (int sid : start_ids) {
        int cur = indexer.in_node(sid, 0);
        std::vector<std::pair<int, int>> path;

        // Start from source edge to in-node.
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

}  // namespace

PlanResult plan_flow(
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

    Dinic dinic(source + 1);

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
            int cap = blocked[t * num_cells + cell] ? 0 : 1;
            int in = indexer.in_node(cell, t);
            int out = indexer.out_node(cell, t);
            if (cap > 0) {
                dinic.add_edge(in, out, cap);
            }
            if (t == T) {
                continue;
            }
            dinic.add_edge(out, indexer.in_node(cell, t + 1), 1);
        }
    }

    for (int t = 0; t < T; ++t) {
        for (int eidx = 0; eidx < num_edges; ++eidx) {
            int a = undirected_edges[eidx].first;
            int b = undirected_edges[eidx].second;
            int edge_in = edge_offset + (t * num_edges + eidx) * 2;
            int edge_out = edge_in + 1;
            dinic.add_edge(indexer.out_node(a, t), edge_in, 1);
            dinic.add_edge(indexer.out_node(b, t), edge_in, 1);
            int edge_cap = 1;
            dinic.add_edge(edge_in, edge_out, edge_cap);
            dinic.add_edge(edge_out, indexer.in_node(a, t + 1), 1);
            dinic.add_edge(edge_out, indexer.in_node(b, t + 1), 1);
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
            for (auto& edge : dinic.graph()[edge_in]) {
                if (edge.to == edge_out && edge.original_cap > 0) {
                    edge.cap = 0;
                    break;
                }
            }
        }
    }

    for (int sid : start_ids) {
        dinic.add_edge(source, indexer.in_node(sid, 0), 1);
    }

    for (size_t i = 0; i < target_ids.size(); ++i) {
        int tid = target_ids[i];
        int cap = caps[i];
        if (cap <= 0) {
            continue;
        }
        for (int t = 0; t <= T; ++t) {
            dinic.add_edge(indexer.out_node(tid, t), sink, cap);
        }
    }

    int flow = dinic.max_flow(source, sink);
    if (flow != static_cast<int>(starts.size())) {
        return result;
    }

    result.paths = extract_paths(dinic, graph, indexer, start_ids, source, sink);
    result.feasible = true;
    return result;
}

PlanResult plan_flow_sync(
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

    Dinic dinic(source + 1);

    for (int t = 0; t <= T; ++t) {
        for (int cell = 0; cell < num_cells; ++cell) {
            int cap = 1;
            if (t == tau && !pickup_mask[cell]) {
                cap = 0;
            }
            int in = indexer.in_node(cell, t);
            int out = indexer.out_node(cell, t);
            if (cap > 0) {
                dinic.add_edge(in, out, cap);
            }
            if (t == T) {
                continue;
            }
            dinic.add_edge(out, indexer.in_node(cell, t + 1), 1);
        }
    }

    for (int t = 0; t < T; ++t) {
        for (int eidx = 0; eidx < num_edges; ++eidx) {
            int a = undirected_edges[eidx].first;
            int b = undirected_edges[eidx].second;
            int edge_in = edge_offset + (t * num_edges + eidx) * 2;
            int edge_out = edge_in + 1;
            dinic.add_edge(indexer.out_node(a, t), edge_in, 1);
            dinic.add_edge(indexer.out_node(b, t), edge_in, 1);
            dinic.add_edge(edge_in, edge_out, 1);
            dinic.add_edge(edge_out, indexer.in_node(a, t + 1), 1);
            dinic.add_edge(edge_out, indexer.in_node(b, t + 1), 1);
        }
    }

    for (int sid : start_ids) {
        dinic.add_edge(source, indexer.in_node(sid, 0), 1);
    }

    for (size_t i = 0; i < drop_ids.size(); ++i) {
        int tid = drop_ids[i];
        int cap = caps[i];
        if (cap <= 0) {
            continue;
        }
        int tnode = target_offset + static_cast<int>(i);
        dinic.add_edge(tnode, sink, cap);
        dinic.add_edge(indexer.out_node(tid, T), tnode, 1);
    }

    int flow = dinic.max_flow(source, sink);
    if (flow != static_cast<int>(starts.size())) {
        return result;
    }

    result.paths = extract_paths(dinic, graph, indexer, start_ids, source, sink);
    result.feasible = true;
    return result;
}
