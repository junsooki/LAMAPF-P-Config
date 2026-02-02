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
            if name.startswith("flow_planner_cpp") and (name.endswith(".so") or name.endswith(".pyd") or name.endswith(".dylib")):
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

from data_types import RobotState
from planner import plan_round
from utils import validate_paths


def test_plan_round_mixed():
    grid = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ]
    robots = [
        RobotState(id=1, pos=(0, 0), state="Loaded"),
        RobotState(id=2, pos=(0, 3), state="Empty"),
    ]
    pickup_points = [(3, 3)]
    drop_points = [(3, 0)]
    drop_caps = {(3, 0): 1}

    T, paths = plan_round(grid, robots, pickup_points, drop_points, drop_caps, T_max=8)
    assert T is not None
    assert paths
    assert validate_paths(paths, grid)
    assert paths[1][-1] in drop_points
    assert paths[2][-1] in pickup_points


def test_plan_round_all_empty():
    grid = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ]
    robots = [
        RobotState(id=1, pos=(0, 0), state="Empty"),
        RobotState(id=2, pos=(2, 2), state="Empty"),
    ]
    pickup_points = [(2, 0), (0, 2)]
    drop_points = []
    drop_caps = {}

    T, paths = plan_round(grid, robots, pickup_points, drop_points, drop_caps, T_max=6)
    assert T is not None
    assert paths
    assert validate_paths(paths, grid)
    assert paths[1][-1] in pickup_points
    assert paths[2][-1] in pickup_points
