#pragma once

#include "dinic.h"

#include <vector>

class HLPP {
public:
    explicit HLPP(int n);

    void add_edge(int u, int v, int cap);
    int max_flow(int s, int t);

    std::vector<std::vector<Edge>>& graph();
    const std::vector<std::vector<Edge>>& graph() const;

private:
    void add_active(int v);
    int pop_active();
    void push(int u, Edge& e);
    void relabel(int v);
    void global_relabel(int s, int t);

    int n_;
    int s_;
    int t_;
    int max_height_;
    int inf_height_;

    std::vector<std::vector<Edge>> g_;
    std::vector<long long> excess_;
    std::vector<int> height_;
    std::vector<int> count_;
    std::vector<int> current_;
    std::vector<char> active_;
    std::vector<std::vector<int>> buckets_;
};
