import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "py")))

from data_types import RobotState
from planner import search_min_T_sync


def _basic_sync_instance():
    grid = [
        [0, 0],
        [0, 0],
    ]
    robots = [
        RobotState(id=1, pos=(0, 0), state="Empty"),
        RobotState(id=2, pos=(1, 0), state="Empty"),
    ]
    pickups = [(0, 1), (1, 1)]
    drops = [(0, 0), (1, 0)]
    drop_caps = {d: 1 for d in drops}
    return grid, robots, pickups, drops, drop_caps


def test_parallel_T_matches_serial():
    grid, robots, pickups, drops, drop_caps = _basic_sync_instance()
    T1, tau1, _ = search_min_T_sync(
        grid, robots, pickups, drops, drop_caps, T_max=6, parallel_workers=1, parallel_T_workers=1
    )
    T2, tau2, _ = search_min_T_sync(
        grid, robots, pickups, drops, drop_caps, T_max=6, parallel_workers=4, parallel_T_workers=2
    )
    assert (T1, tau1) == (2, 1)
    assert (T2, tau2) == (2, 1)


def test_parallel_tau_matches_serial():
    grid, robots, pickups, drops, drop_caps = _basic_sync_instance()
    T1, tau1, _ = search_min_T_sync(
        grid, robots, pickups, drops, drop_caps, T_max=6, parallel_workers=1, parallel_T_workers=1
    )
    T2, tau2, _ = search_min_T_sync(
        grid, robots, pickups, drops, drop_caps, T_max=6, parallel_workers=4, parallel_T_workers=1
    )
    assert (T1, tau1) == (2, 1)
    assert (T2, tau2) == (2, 1)


def test_parallel_search_infeasible():
    grid = [
        [0, 1],
        [1, 0],
    ]
    robots = [RobotState(id=1, pos=(0, 0), state="Empty")]
    pickups = [(1, 1)]
    drops = [(0, 0)]
    drop_caps = {(0, 0): 1}
    T, tau, paths = search_min_T_sync(
        grid, robots, pickups, drops, drop_caps, T_max=4, parallel_workers=4, parallel_T_workers=2
    )
    assert T is None
    assert tau is None
    assert paths == {}
