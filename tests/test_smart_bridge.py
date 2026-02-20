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
            raise ImportError(
                "flow_planner_cpp not found; build the C++ module before running tests"
            ) from exc


flow_planner_cpp = _import_flow_planner()

from smart_bridge import plan_standard_mapf
from utils import validate_paths


def test_bridge_small_grid():
    """4x4 empty grid, 2 agents, verify collision-free paths."""
    grid = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ]
    starts = [(0, 0), (3, 3)]
    goals = [(3, 0), (0, 3)]

    T, paths = plan_standard_mapf(grid, starts, goals, T_max=20)
    assert T is not None, "Solver should find a solution"
    assert len(paths) == 2
    assert validate_paths(paths, grid)
    # Each agent should reach one of the goals
    final_positions = {paths[i][-1] for i in paths}
    assert final_positions == set(goals)


def test_bridge_with_obstacles():
    """Routing around obstacles."""
    grid = [
        [0, 0, 0, 0],
        [0, 1, 1, 0],
        [0, 1, 1, 0],
        [0, 0, 0, 0],
    ]
    starts = [(0, 0), (3, 0)]
    goals = [(0, 3), (3, 3)]

    T, paths = plan_standard_mapf(grid, starts, goals, T_max=30)
    assert T is not None, "Solver should find a solution around obstacles"
    assert len(paths) == 2
    assert validate_paths(paths, grid)
    final_positions = {paths[i][-1] for i in paths}
    assert final_positions == set(goals)
