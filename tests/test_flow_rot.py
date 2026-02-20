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
        import flow_planner_cpp
        return flow_planner_cpp
    except ImportError:
        _maybe_add_build_path()
        import flow_planner_cpp
        return flow_planner_cpp


flow_planner_cpp = _import_flow_planner()

# Direction constants (must match C++ and data_types.py)
DIR_EAST = 0
DIR_WEST = 1
DIR_SOUTH = 2
DIR_NORTH = 3


def _no_vertex_conflict(paths):
    if not paths:
        return True
    max_len = max(len(p) for p in paths)
    for t in range(max_len):
        seen = set()
        for path in paths:
            pos = path[min(t, len(path) - 1)]
            if pos in seen:
                return False
            seen.add(pos)
    return True


def _no_edge_conflict(paths):
    if not paths:
        return True
    max_len = max(len(p) for p in paths)
    for t in range(1, max_len):
        used = {}
        for i, path in enumerate(paths):
            prev = path[min(t - 1, len(path) - 1)]
            curr = path[min(t, len(path) - 1)]
            rev = (curr, prev)
            if rev in used and used[rev] != i:
                return False
            used[(prev, curr)] = i
    return True


def test_aligned_agent_same_T_as_non_rot():
    """Agent facing target direction needs no rotation — same T as non-rotation."""
    grid = [[0, 0, 0]]
    starts = [(0, 0)]
    targets = [(2, 0)]
    # Facing EAST, target is to the east -> no rotation needed
    res_rot = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_EAST], targets, [1], 2, [], []
    )
    res_std = flow_planner_cpp.plan_flow(
        grid, starts, targets, [1], 2, [], []
    )
    assert res_rot["feasible"] is True
    assert res_std["feasible"] is True
    assert len(res_rot["paths"][0]) == len(res_std["paths"][0])


def test_90_degree_rotation_adds_one_step():
    """Agent 90 degrees off needs 1 extra timestep to rotate."""
    grid = [[0, 0, 0]]
    starts = [(0, 0)]
    targets = [(2, 0)]
    # Facing EAST: should reach in T=2
    res_east = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_EAST], targets, [1], 2, [], []
    )
    assert res_east["feasible"] is True

    # Facing SOUTH (90 off): needs T=2 steps but must rotate first -> needs T=3
    res_south = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_SOUTH], targets, [1], 2, [], []
    )
    assert res_south["feasible"] is False  # T=2 not enough

    res_south_3 = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_SOUTH], targets, [1], 3, [], []
    )
    assert res_south_3["feasible"] is True
    assert len(res_south_3["paths"][0]) == 4  # t=0,1,2,3


def test_180_degree_rotation_adds_two_steps():
    """Agent 180 degrees off needs 2 extra timesteps."""
    grid = [[0, 0, 0]]
    starts = [(0, 0)]
    targets = [(2, 0)]
    # Facing WEST (180 off): needs T=4 (2 rotations + 2 moves)
    res_west_3 = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_WEST], targets, [1], 3, [], []
    )
    assert res_west_3["feasible"] is False

    res_west_4 = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_WEST], targets, [1], 4, [], []
    )
    assert res_west_4["feasible"] is True
    assert len(res_west_4["paths"][0]) == 5  # t=0..4


def test_two_agents_rotation_no_collision():
    """Two agents needing rotation still avoid collisions."""
    grid = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ]
    starts = [(0, 0), (2, 0)]
    targets = [(2, 0), (0, 0)]
    dirs = [DIR_EAST, DIR_WEST]
    res = flow_planner_cpp.plan_flow_rot(
        grid, starts, dirs, targets, [1, 1], 6, [], []
    )
    assert res["feasible"] is True
    paths = res["paths"]
    assert len(paths) == 2
    assert _no_vertex_conflict(paths)
    assert _no_edge_conflict(paths)


def test_reserved_vertex_blocks_all_directions():
    """A reserved vertex blocks an agent regardless of facing direction."""
    grid = [[0, 0, 0]]
    starts = [(0, 0)]
    targets = [(2, 0)]
    # Reserve (1,0) at t=1 — agent facing east cannot pass through
    reserved = [(1, 0, 1)]
    res = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_EAST], targets, [1], 2, reserved, []
    )
    assert res["feasible"] is False

    # With more time, agent can wait then proceed
    res3 = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_EAST], targets, [1], 4, reserved, []
    )
    assert res3["feasible"] is True


def test_wait_preserves_direction():
    """Direction stays the same when waiting."""
    grid = [[0, 0, 0]]
    starts = [(0, 0)]
    targets = [(2, 0)]
    # Reserve (1,0) at t=1, so agent must wait at (0,0) for t=1, then move
    reserved = [(1, 0, 1)]
    res = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_EAST], targets, [1], 4, reserved, []
    )
    assert res["feasible"] is True
    path_dirs = res["path_dirs"][0]
    path = res["paths"][0]
    # While at same position, direction should stay the same
    for i in range(len(path) - 1):
        if path[i] == path[i + 1]:
            assert path_dirs[i] == path_dirs[i + 1], (
                f"Direction changed during wait at t={i}: {path_dirs[i]} -> {path_dirs[i+1]}"
            )


def test_path_dirs_returned():
    """path_dirs has same length as paths and contains valid directions."""
    grid = [[0, 0, 0]]
    starts = [(0, 0)]
    targets = [(2, 0)]
    res = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_EAST], targets, [1], 2, [], []
    )
    assert res["feasible"] is True
    assert len(res["path_dirs"]) == len(res["paths"])
    for dirs in res["path_dirs"]:
        for d in dirs:
            assert d in (DIR_EAST, DIR_WEST, DIR_SOUTH, DIR_NORTH)


def test_empty_starts():
    """Empty starts should return feasible with empty paths."""
    grid = [[0, 0, 0]]
    res = flow_planner_cpp.plan_flow_rot(grid, [], [], [(2, 0)], [1], 2, [], [])
    assert res["feasible"] is True
    assert res["paths"] == []
    assert res["path_dirs"] == []


def test_hlpp_solver():
    """HLPP solver works for rotation-aware planning too."""
    grid = [[0, 0, 0]]
    starts = [(0, 0)]
    targets = [(2, 0)]
    res = flow_planner_cpp.plan_flow_rot(
        grid, starts, [DIR_EAST], targets, [1], 2, [], [], "hlpp"
    )
    assert res["feasible"] is True
    assert res["paths"][0][-1] == (2, 0)
