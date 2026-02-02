#pragma once

#include <string>
#include <tuple>
#include <utility>
#include <vector>

struct PlanResult {
    bool feasible;
    std::vector<std::vector<std::pair<int, int>>> paths;
};

PlanResult plan_flow(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int, int, int>>& reserved,
    const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges);

PlanResult plan_flow_with_method(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& targets,
    const std::vector<int>& target_caps,
    int T,
    const std::vector<std::tuple<int, int, int>>& reserved,
    const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges,
    const std::string& method);

PlanResult plan_flow_sync(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& pickups,
    const std::vector<std::pair<int, int>>& drops,
    const std::vector<int>& drop_caps,
    int T,
    int tau);

PlanResult plan_flow_sync_with_method(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& pickups,
    const std::vector<std::pair<int, int>>& drops,
    const std::vector<int>& drop_caps,
    int T,
    int tau,
    const std::string& method);
