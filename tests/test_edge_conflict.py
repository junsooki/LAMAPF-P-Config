import os
import sys

import pytest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "py")))

from simulator_full import _validate_collision_free


def test_edge_swap_forbidden():
    trajectories = {
        1: [(0, 0), (1, 0)],
        2: [(1, 0), (0, 0)],
    }
    with pytest.raises(RuntimeError):
        _validate_collision_free(trajectories)
