#include "hlpp.h"

#include <algorithm>
#include <queue>

HLPP::HLPP(int n)
    : n_(n),
      s_(0),
      t_(0),
      max_height_(0),
      inf_height_(2 * n),
      g_(n) {}

void HLPP::add_edge(int u, int v, int cap) {
    Edge a{v, static_cast<int>(g_[v].size()), cap, cap};
    Edge b{u, static_cast<int>(g_[u].size()), 0, 0};
    g_[u].push_back(a);
    g_[v].push_back(b);
}

std::vector<std::vector<Edge>>& HLPP::graph() {
    return g_;
}

const std::vector<std::vector<Edge>>& HLPP::graph() const {
    return g_;
}

void HLPP::add_active(int v) {
    if (v == s_ || v == t_) {
        return;
    }
    if (active_[v]) {
        return;
    }
    if (excess_[v] <= 0 || height_[v] >= inf_height_) {
        return;
    }
    active_[v] = 1;
    buckets_[height_[v]].push_back(v);
    if (height_[v] > max_height_) {
        max_height_ = height_[v];
    }
}

int HLPP::pop_active() {
    while (max_height_ >= 0 && buckets_[max_height_].empty()) {
        --max_height_;
    }
    if (max_height_ < 0) {
        return -1;
    }
    int v = buckets_[max_height_].back();
    buckets_[max_height_].pop_back();
    active_[v] = 0;
    return v;
}

void HLPP::push(int u, Edge& e) {
    if (excess_[u] <= 0 || e.cap <= 0) {
        return;
    }
    int v = e.to;
    int send = static_cast<int>(std::min<long long>(excess_[u], e.cap));
    if (send <= 0) {
        return;
    }
    e.cap -= send;
    g_[v][e.rev].cap += send;
    excess_[u] -= send;
    excess_[v] += send;
    if (v != s_ && v != t_ && excess_[v] == send) {
        add_active(v);
    }
}

void HLPP::relabel(int v) {
    int old_height = height_[v];
    int min_height = inf_height_;
    for (const auto& e : g_[v]) {
        if (e.cap <= 0) {
            continue;
        }
        min_height = std::min(min_height, height_[e.to]);
    }
    int new_height = (min_height >= inf_height_) ? inf_height_ : (min_height + 1);
    height_[v] = new_height;
    current_[v] = 0;
    if (old_height < static_cast<int>(count_.size())) {
        count_[old_height]--;
    }
    if (new_height < static_cast<int>(count_.size())) {
        count_[new_height]++;
    }
    if (old_height < n_ && count_[old_height] == 0) {
        for (int i = 0; i < n_; ++i) {
            if (height_[i] > old_height && height_[i] < inf_height_) {
                if (height_[i] < static_cast<int>(count_.size())) {
                    count_[height_[i]]--;
                }
                height_[i] = inf_height_;
                active_[i] = 0;
            }
        }
    }
}

void HLPP::global_relabel(int s, int t) {
    std::fill(height_.begin(), height_.end(), inf_height_);
    std::queue<int> q;
    height_[t] = 0;
    q.push(t);
    while (!q.empty()) {
        int v = q.front();
        q.pop();
        for (const auto& e : g_[v]) {
            const Edge& rev = g_[e.to][e.rev];
            if (rev.cap <= 0) {
                continue;
            }
            if (height_[e.to] != inf_height_) {
                continue;
            }
            height_[e.to] = height_[v] + 1;
            q.push(e.to);
        }
    }
    height_[s] = n_;
}

int HLPP::max_flow(int s, int t) {
    if (s == t) {
        return 0;
    }
    s_ = s;
    t_ = t;
    inf_height_ = 2 * n_;
    height_.assign(n_, 0);
    excess_.assign(n_, 0);
    active_.assign(n_, 0);
    current_.assign(n_, 0);
    count_.assign(inf_height_ + 1, 0);
    buckets_.assign(inf_height_ + 1, {});
    max_height_ = 0;

    global_relabel(s, t);
    for (int i = 0; i < n_; ++i) {
        int h = height_[i];
        if (h >= 0 && h < static_cast<int>(count_.size())) {
            count_[h]++;
        }
    }

    for (auto& e : g_[s]) {
        if (e.cap <= 0) {
            continue;
        }
        int send = e.cap;
        e.cap = 0;
        g_[e.to][e.rev].cap += send;
        excess_[e.to] += send;
        excess_[s] -= send;
        add_active(e.to);
    }

    while (true) {
        int v = pop_active();
        if (v < 0) {
            break;
        }
        while (excess_[v] > 0) {
            if (current_[v] >= static_cast<int>(g_[v].size())) {
                relabel(v);
                if (height_[v] >= inf_height_) {
                    break;
                }
                continue;
            }
            Edge& e = g_[v][current_[v]];
            if (e.cap > 0 && height_[v] == height_[e.to] + 1) {
                push(v, e);
            } else {
                current_[v]++;
            }
        }
        if (excess_[v] > 0) {
            add_active(v);
        }
    }
    return static_cast<int>(excess_[t]);
}
