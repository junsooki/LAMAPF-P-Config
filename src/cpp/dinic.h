#pragma once

#include <vector>

struct Edge {
    int to;
    int rev;
    int cap;
    int original_cap;
};

class Dinic {
public:
    explicit Dinic(int n);

    void add_edge(int u, int v, int cap);
    int max_flow(int s, int t);

    std::vector<std::vector<Edge>>& graph();
    const std::vector<std::vector<Edge>>& graph() const;

private:
    bool bfs(int s, int t);
    int dfs(int v, int t, int f);

    int n_;
    std::vector<std::vector<Edge>> g_;
    std::vector<int> level_;
    std::vector<int> it_;
};
