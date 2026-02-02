#include "dinic.h"

#include <algorithm>
#include <queue>

Dinic::Dinic(int n)
    : n_(n), g_(n), level_(n), it_(n) {}

void Dinic::add_edge(int u, int v, int cap) {
    Edge a{v, static_cast<int>(g_[v].size()), cap, cap};
    Edge b{u, static_cast<int>(g_[u].size()), 0, 0};
    g_[u].push_back(a);
    g_[v].push_back(b);
}

bool Dinic::bfs(int s, int t) {
    std::fill(level_.begin(), level_.end(), -1);
    std::queue<int> q;
    level_[s] = 0;
    q.push(s);
    while (!q.empty()) {
        int v = q.front();
        q.pop();
        for (const auto& e : g_[v]) {
            if (e.cap > 0 && level_[e.to] < 0) {
                level_[e.to] = level_[v] + 1;
                q.push(e.to);
            }
        }
    }
    return level_[t] >= 0;
}

int Dinic::dfs(int v, int t, int f) {
    if (v == t) {
        return f;
    }
    for (int& i = it_[v]; i < static_cast<int>(g_[v].size()); ++i) {
        Edge& e = g_[v][i];
        if (e.cap <= 0 || level_[v] + 1 != level_[e.to]) {
            continue;
        }
        int pushed = dfs(e.to, t, std::min(f, e.cap));
        if (pushed > 0) {
            e.cap -= pushed;
            g_[e.to][e.rev].cap += pushed;
            return pushed;
        }
    }
    return 0;
}

int Dinic::max_flow(int s, int t) {
    int flow = 0;
    const int kInf = 1'000'000'000;
    while (bfs(s, t)) {
        std::fill(it_.begin(), it_.end(), 0);
        while (true) {
            int pushed = dfs(s, t, kInf);
            if (pushed == 0) {
                break;
            }
            flow += pushed;
        }
    }
    return flow;
}

std::vector<std::vector<Edge>>& Dinic::graph() {
    return g_;
}

const std::vector<std::vector<Edge>>& Dinic::graph() const {
    return g_;
}
