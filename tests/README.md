# tests/

Pytest test suite for the planner.

- `test_flow_cpp.py`: sanity checks for C++ max-flow binding
- `test_small_cases.py`: end-to-end checks for Python planner on small grids
- `test_edge_conflict.py`: ensures edge-swap collisions are forbidden
- `test_sync_parallel.py`: checks parallel search matches serial for sync planner
- `test_sync_two_stage.py`: validates sync planner on a small scenario
- `test_sync_planner_guard.py`: guards against invalid sync inputs
- `test_simulator_full_sync_reachability.py`: ensures unreachable regions are excluded from starts
