import os
import sys

import pytest

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


_import_flow_planner()

from data_types import RobotState
from planner import plan_round_sync


def test_sync_requires_goals_ge_agents():
    grid = [[0]]
    robots = [
        RobotState(id=1, pos=(0, 0), state="Empty"),
        RobotState(id=2, pos=(0, 0), state="Empty"),
    ]
    with pytest.raises(RuntimeError):
        plan_round_sync(
            grid,
            robots,
            pickup_points=[(0, 0), (0, 0)],
            drop_points=[(0, 0)],
            drop_caps={(0, 0): 1},
            T_max=0,
        )
