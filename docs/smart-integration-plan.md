# Plan: Integrate LAMAPF-P with SMART-MAPF Testbed

## Context

The [SMART-MAPF](https://github.com/smart-mapf/smart) project is a testbed that bridges MAPF algorithms with real robot simulation (ARGoS3). It uses an Action Dependency Graph (ADG) server to convert planned paths into robot actions with automatic rotation handling. The goal is to allow LAMAPF-P's network flow planner to read standard MAPF benchmark inputs (Moving AI `.map` and `.scen` files), plan collision-free paths, and export them in SMART's path format for robot simulation.

**Key insight**: SMART's `processAgentActions()` in `parser.cpp` automatically inserts rotation steps between path waypoints, so LAMAPF-P can export standard (non-rotation) paths and SMART handles turning.

---

## New Files

| File | Purpose |
|------|---------|
| `src/py/smart_io.py` | Parsers for `.map`/`.scen` files + path exporter |
| `src/py/smart_bridge.py` | CLI entry point: read inputs, plan, export |
| `tests/test_smart_io.py` | Unit tests for parsers and exporter |
| `tests/test_smart_bridge.py` | Integration test (requires C++ build) |

No existing files are modified.

---

## 1. `src/py/smart_io.py` — Format Parsers & Exporter

### `load_movingai_map(path) -> List[List[int]]`

Parse Moving AI `.map` format:
```
type octile
height H
width W
map
<grid: . G S = passable(0), @ T W O = obstacle(1)>
```

Returns `grid[row][col]` matching LAMAPF-P convention. Validates dimensions match header.

### `load_scen(path, num_agents=None) -> List[Tuple[Tuple[int,int], Tuple[int,int]]]`

Parse `.scen` format (TSV):
```
version 1
bucket  map_file  width  height  startx  starty  goalx  goaly  optimal_length
```

Returns list of `((start_x, start_y), (goal_x, goal_y))` where `x=col, y=row` (matching LAMAPF-P `(x,y)` convention). Columns 4-7 from TSV. `num_agents` truncates the list.

### `export_paths_smart(paths, output_path, continuous=True)`

Export paths in SMART format. Default is continuous (what SMART's ADG server expects):
```
Agent 0:(x0,y0,0)->(x1,y1,1)->(x2,y2,2)->
```

Agents are 0-indexed (SMART convention). Time is implicit from position index.

---

## 2. `src/py/smart_bridge.py` — CLI Bridge

### `plan_standard_mapf(grid, starts, goals, T_max, method) -> (T, paths_dict)`

Uses existing `_find_min_T_single()` from `planner.py` (line 147) which calls `flow_planner_cpp.plan_flow()`. Binary search over T to find optimal makespan.

- `starts` and `goals` from scen file
- `caps = [1] * len(goals)` (one agent per goal)
- `reserved_v = []`, `reserved_e = []` (no external reservations)
- Returns `{agent_idx: [(x,y), ...]}` with 0-indexed agent IDs

**Note on goal assignment**: The flow solver finds optimal matching of agents to goals (not necessarily respecting scen file pairing). This is fine — SMART follows whatever paths are in the file and doesn't enforce specific start-goal assignments.

### CLI interface

```
python3 smart_bridge.py \
    --map MAP.map --scen SCENARIO.scen \
    --agents N --output paths.txt \
    [--T_max 200] [--solver dinic] [--verbose]
```

Prints: grid size, agent count, optimal makespan T, output path. Reminds user to use `--flip_coord=0` with SMART (LAMAPF-P uses xy coordinates).

---

## 3. Coordinate Convention (verified end-to-end)

```
.map file → grid[row][col] → positions as (x=col, y=row) → export as (x,y,t) → SMART --flip_coord=0
```

All consistent. LAMAPF-P's `(x,y)` = `(col,row)` matches SMART's xy mode.

---

## 4. Tests

### `tests/test_smart_io.py` (no C++ dependency)
- `test_load_movingai_map_empty` — 4x4 empty grid
- `test_load_movingai_map_obstacles` — grid with `@`/`T` obstacles
- `test_load_movingai_map_bad_dims` — dimension mismatch raises error
- `test_load_scen` — basic parsing, verify start/goal extraction
- `test_load_scen_num_agents` — truncation with `num_agents` param
- `test_export_paths_smart_continuous` — verify output format matches `Agent N:(x,y,t)->...`
- `test_export_paths_smart_discrete` — verify format without time

### `tests/test_smart_bridge.py` (requires C++ build)
- `test_bridge_small_grid` — 4x4 empty, 2 agents, verify collision-free paths
- `test_bridge_with_obstacles` — routing around obstacles

---

## 5. Implementation Order

1. `src/py/smart_io.py` — parsers + exporter
2. `tests/test_smart_io.py` — unit tests (no C++ needed)
3. `src/py/smart_bridge.py` — bridge script
4. `tests/test_smart_bridge.py` — integration tests

---

## 6. Verification

```bash
# Unit tests (no build needed)
cd src/py && python3 -m pytest ../../tests/test_smart_io.py -v

# Integration tests (needs C++ build)
cd src/py && python3 -m pytest ../../tests/test_smart_bridge.py -v

# All tests still pass
cd src/py && python3 -m pytest ../../tests/ -v

# Manual end-to-end: download a Moving AI benchmark and run
python3 smart_bridge.py --map empty-8-8.map --scen empty-8-8-random-1.scen --agents 5 --output paths.txt --verbose
```

## Key Existing Code to Reuse

- `planner._find_min_T_single()` (`src/py/planner.py:147`) — core planning function
- `map_loader.normalize_grid()` (`src/py/map_loader.py:26`) — grid validation
- Test boilerplate from `tests/test_small_cases.py:1-30` — sys.path and build path setup
