# networkflow_mapf

Event-driven Lifelong Warehouse Routing planner based on time-expanded max-flow.

## Project Structure
- `src/cpp/`: C++ core (grid, max-flow, time-expanded planner) + pybind11 bindings
- `src/py/`: Python planner + simulator utilities
- `docs/`: One-to-one documentation for every code file
- `tests/`: Pytest cases for C++ binding and Python planner

## Build (C++ module)
Requires CMake and pybind11.

```
rm -rf build
export CXX=g++
cmake -S . -B build
cmake --build build
```

By default, tests run automatically after build (requires pytest installed in the active Python env).

This produces a Python extension module `flow_planner_cpp` in `build/`.

## Use (Python)
Example usage from Python (ensure `build/` is on `PYTHONPATH`):

```python
from src.py.planner import plan_round
from src.py.data_types import RobotState

grid = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
]
robots = [RobotState(id=1, pos=(0, 0), state="Loaded")]

T, paths = plan_round(
    grid=grid,
    robots=robots,
    pickup_points=[(2, 2)],
    drop_points=[(2, 0)],
    drop_caps={(2, 0): 1},
    T_max=6,
)
print(T, paths)
```

## Map Editor (Tkinter)
Launch the editor:
```
python src/py/ui/map_editor.py
```

The editor loads maps from `maps/`, lets you edit walls/shelves/agents/goals, and can run a single-stage plan and play it back.

You can also load a simulation output JSON (`sim_output.json`) to replay with a timestep slider, step forward/back, and play/pause.

## Full Simulation (CLI)
Run a full event-driven simulation from a map file:
```
python src/py/simulator_full.py --map maps/your_map.json --agents 5 --max_timestep 200 --output sim_output.json --seed 0
```

## Full Simulation (Sync Two-Stage)
Run a synchronized two-stage simulation (all robots reach pickups at tau, then all reach drops at T):
```
python src/py/simulator_full_sync.py --map maps/your_map.json --agents 5 --max_timestep 200 --output sim_output.json --seed 0
```
Add debug progress output:
```
python src/py/simulator_full_sync.py --map maps/your_map.json --agents 5 --max_timestep 200 --output sim_output.json --seed 0 --debug --debug_every 25
```

## Tests
```
pytest -q
```

If the module is built in `build/`, tests will automatically add that path.
