#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "flow_planner.h"

namespace py = pybind11;

PYBIND11_MODULE(flow_planner_cpp, m) {
    m.doc() = "Time-expanded max-flow planner bindings";

    m.def("plan_flow", [](const std::vector<std::vector<int>>& grid,
                           const std::vector<std::pair<int, int>>& starts,
                           const std::vector<std::pair<int, int>>& targets,
                           const std::vector<int>& target_caps,
                           int T,
                           const std::vector<std::tuple<int, int, int>>& reserved,
                           const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges) {
        PlanResult result = plan_flow(grid, starts, targets, target_caps, T, reserved, reserved_edges);
        py::dict out;
        out["feasible"] = result.feasible;
        out["paths"] = result.paths;
        return out;
    });

    m.def("plan_flow_sync", [](const std::vector<std::vector<int>>& grid,
                                const std::vector<std::pair<int, int>>& starts,
                                const std::vector<std::pair<int, int>>& pickups,
                                const std::vector<std::pair<int, int>>& drops,
                                const std::vector<int>& drop_caps,
                                int T,
                                int tau) {
        PlanResult result = plan_flow_sync(grid, starts, pickups, drops, drop_caps, T, tau);
        py::dict out;
        out["feasible"] = result.feasible;
        out["paths"] = result.paths;
        return out;
    });
}
