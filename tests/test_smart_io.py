import os
import sys
import tempfile

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "py")))

from smart_io import load_movingai_map, load_scen, export_paths_smart


def _write_tmp(content: str, suffix: str) -> str:
    fd, path = tempfile.mkstemp(suffix=suffix)
    with os.fdopen(fd, "w") as f:
        f.write(content)
    return path


# --- load_movingai_map tests ---

def test_load_movingai_map_empty():
    content = "type octile\nheight 4\nwidth 4\nmap\n....\n....\n....\n....\n"
    path = _write_tmp(content, ".map")
    try:
        grid = load_movingai_map(path)
        assert len(grid) == 4
        assert all(len(row) == 4 for row in grid)
        assert all(cell == 0 for row in grid for cell in row)
    finally:
        os.unlink(path)


def test_load_movingai_map_obstacles():
    content = "type octile\nheight 3\nwidth 4\nmap\n....\n.@T.\n....\n"
    path = _write_tmp(content, ".map")
    try:
        grid = load_movingai_map(path)
        assert grid[0] == [0, 0, 0, 0]
        assert grid[1] == [0, 1, 1, 0]
        assert grid[2] == [0, 0, 0, 0]
    finally:
        os.unlink(path)


def test_load_movingai_map_bad_dims():
    content = "type octile\nheight 2\nwidth 4\nmap\n....\n....\n....\n"
    path = _write_tmp(content, ".map")
    try:
        try:
            load_movingai_map(path)
            assert False, "Should have raised ValueError"
        except ValueError:
            pass
    finally:
        os.unlink(path)


# --- load_scen tests ---

def test_load_scen():
    content = (
        "version 1\n"
        "0\tmap.map\t8\t8\t1\t2\t5\t6\t5.0\n"
        "0\tmap.map\t8\t8\t3\t4\t7\t0\t6.0\n"
    )
    path = _write_tmp(content, ".scen")
    try:
        agents = load_scen(path)
        assert len(agents) == 2
        assert agents[0] == ((1, 2), (5, 6))
        assert agents[1] == ((3, 4), (7, 0))
    finally:
        os.unlink(path)


def test_load_scen_num_agents():
    content = (
        "version 1\n"
        "0\tmap.map\t8\t8\t0\t0\t1\t1\t1.0\n"
        "0\tmap.map\t8\t8\t2\t2\t3\t3\t1.0\n"
        "0\tmap.map\t8\t8\t4\t4\t5\t5\t1.0\n"
    )
    path = _write_tmp(content, ".scen")
    try:
        agents = load_scen(path, num_agents=2)
        assert len(agents) == 2
        assert agents[0] == ((0, 0), (1, 1))
        assert agents[1] == ((2, 2), (3, 3))
    finally:
        os.unlink(path)


# --- export_paths_smart tests ---

def test_export_paths_smart_continuous():
    paths = {
        0: [(0, 0), (1, 0), (2, 0)],
        1: [(3, 3), (3, 2), (3, 1)],
    }
    path = _write_tmp("", ".txt")
    try:
        export_paths_smart(paths, path, continuous=True)
        with open(path) as f:
            lines = f.read().strip().split("\n")
        assert len(lines) == 2
        assert lines[0] == "Agent 0:(0,0,0)->(1,0,1)->(2,0,2)->"
        assert lines[1] == "Agent 1:(3,3,0)->(3,2,1)->(3,1,2)->"
    finally:
        os.unlink(path)


def test_export_paths_smart_discrete():
    paths = {
        0: [(0, 0), (1, 0), (2, 0)],
    }
    path = _write_tmp("", ".txt")
    try:
        export_paths_smart(paths, path, continuous=False)
        with open(path) as f:
            lines = f.read().strip().split("\n")
        assert len(lines) == 1
        assert lines[0] == "Agent 0:(0,0)->(1,0)->(2,0)->"
    finally:
        os.unlink(path)
