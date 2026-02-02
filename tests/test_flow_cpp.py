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


def _no_vertex_conflict(paths):
    max_len = max(len(p) for p in paths)
    for t in range(max_len):
        seen = set()
        for path in paths:
            if t >= len(path):
                continue
            if path[t] in seen:
                return False
            seen.add(path[t])
    return True


def test_single_agent_reaches_target():
    grid = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ]
    starts = [(0, 0)]
    targets = [(2, 0)]
    result = flow_planner_cpp.plan_flow(grid, starts, targets, [1], 2, [], [])
    assert result["feasible"] is True
    paths = result["paths"]
    assert len(paths) == 1
    assert paths[0][0] == (0, 0)
    assert paths[0][-1] in targets
    assert len(paths[0]) <= 3


def test_two_agents_no_conflict():
    grid = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ]
    starts = [(0, 0), (2, 2)]
    targets = [(2, 0), (0, 2)]
    result = flow_planner_cpp.plan_flow(grid, starts, targets, [1, 1], 4, [], [])
    assert result["feasible"] is True
    paths = result["paths"]
    assert len(paths) == 2
    assert _no_vertex_conflict(paths)
    assert paths[0][0] == (0, 0)
    assert paths[1][0] == (2, 2)


def test_same_target_different_times():
    grid = [
        [0, 0, 0],
    ]
    starts = [(0, 0), (2, 0)]
    targets = [(1, 0)]
    result = flow_planner_cpp.plan_flow(grid, starts, targets, [1], 2, [], [])
    assert result["feasible"] is True
    paths = result["paths"]
    assert len(paths) == 2
    assert all(path[-1] == (1, 0) for path in paths)
