import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "py")))


def _maybe_add_build_path():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    candidates = [
        os.path.join(root, "build"),
        os.path.join(root, "build", "Release"),
        os.path.join(root, "build", "Debug"),
    ]
    for path in candidates:
        if not os.path.isdir(path):
            continue
        for name in os.listdir(path):
            if name.startswith("flow_planner_cpp") and (
                name.endswith(".so") or name.endswith(".pyd") or name.endswith(".dylib")
            ):
                sys.path.append(path)
                return


def _import_flow_planner():
    try:
        import flow_planner_cpp  # type: ignore
        return flow_planner_cpp
    except ImportError:
        _maybe_add_build_path()
        try:
            import flow_planner_cpp  # type: ignore
            return flow_planner_cpp
        except ImportError as exc:
            raise ImportError("flow_planner_cpp not found; build the C++ module before running tests") from exc


flow_planner_cpp = _import_flow_planner()


def test_sync_two_stage_feasible():
    grid = [
        [0, 0],
        [0, 0],
    ]
    starts = [(0, 0), (1, 0)]
    pickups = [(0, 1), (1, 1)]
    drops = [(0, 0), (1, 0)]
    result = flow_planner_cpp.plan_flow_sync(grid, starts, pickups, drops, [1, 1], 2, 1)
    assert result["feasible"] is True
    paths = result["paths"]
    assert len(paths) == 2
    assert all(len(p) == 3 for p in paths)
    for path in paths:
        assert path[1] in pickups
        assert path[2] in drops


def test_sync_two_stage_infeasible_when_pickups_too_few():
    grid = [
        [0, 0],
        [0, 0],
    ]
    starts = [(0, 0), (1, 0)]
    pickups = [(0, 1)]
    drops = [(0, 0), (1, 0)]
    result = flow_planner_cpp.plan_flow_sync(grid, starts, pickups, drops, [1, 1], 2, 1)
    assert result["feasible"] is False


def test_sync_tau_too_small_infeasible():
    grid = [
        [0, 0],
        [0, 0],
    ]
    starts = [(0, 0)]
    pickups = [(1, 1)]
    drops = [(0, 1)]
    result = flow_planner_cpp.plan_flow_sync(grid, starts, pickups, drops, [1], 2, 0)
    assert result["feasible"] is False


def test_sync_hlpp_solver_feasible():
    grid = [
        [0, 0],
        [0, 0],
    ]
    starts = [(0, 0), (1, 0)]
    pickups = [(0, 1), (1, 1)]
    drops = [(0, 0), (1, 0)]
    result = flow_planner_cpp.plan_flow_sync(grid, starts, pickups, drops, [1, 1], 2, 1, "hlpp")
    assert result["feasible"] is True
