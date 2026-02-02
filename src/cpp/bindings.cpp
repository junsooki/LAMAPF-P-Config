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
                           const std::vector<std::tuple<int, int, int, int, int>>& reserved_edges,
                           const std::string& method) {
        PlanResult result;
        {
            py::gil_scoped_release release;
            result = plan_flow_with_method(
                grid, starts, targets, target_caps, T, reserved, reserved_edges, method);
        }
        py::dict out;
        out["feasible"] = result.feasible;
        out["paths"] = result.paths;
        return out;
    }, py::arg("grid"), py::arg("starts"), py::arg("targets"), py::arg("target_caps"), py::arg("T"),
       py::arg("reserved"), py::arg("reserved_edges"), py::arg("method") = "dinic");

    m.def("plan_flow_sync", [](const std::vector<std::vector<int>>& grid,
                                const std::vector<std::pair<int, int>>& starts,
                                const std::vector<std::pair<int, int>>& pickups,
                                const std::vector<std::pair<int, int>>& drops,
                                const std::vector<int>& drop_caps,
                                int T,
                                int tau,
                                const std::string& method) {
        PlanResult result;
        {
            py::gil_scoped_release release;
            result = plan_flow_sync_with_method(
                grid, starts, pickups, drops, drop_caps, T, tau, method);
        }
        py::dict out;
        out["feasible"] = result.feasible;
        out["paths"] = result.paths;
        return out;
    }, py::arg("grid"), py::arg("starts"), py::arg("pickups"), py::arg("drops"), py::arg("drop_caps"),
       py::arg("T"), py::arg("tau"), py::arg("method") = "dinic");
}
