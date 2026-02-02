import argparse
import json
import os
import random
import sys
from typing import Dict, List, Tuple

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), ".")))

from data_types import RobotState
from planner import plan_round


def load_map(map_path: str) -> Dict:
    with open(map_path, "r", encoding="utf-8") as f:
        return json.load(f)


def random_free_positions(cells: List[List[int]], count: int, avoid: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    height = len(cells)
    width = len(cells[0]) if height > 0 else 0
    candidates = []
    avoid_set = set(avoid)
    for y in range(height):
        for x in range(width):
            if cells[y][x] == 1:
                continue
            if (x, y) in avoid_set:
                continue
            candidates.append((x, y))
    if count > len(candidates):
        raise ValueError("Not enough free cells for agents")
    return random.sample(candidates, count)


def ensure_tasks(
    tasks: List[Dict],
    shelf_cells: List[Tuple[int, int]],
    current_timestep: int,
    agent_count: int,
    next_task_id: int,
) -> int:
    available_positions = {
        tuple(t["pos"])
        for t in tasks
        if t["spawn_time"] <= current_timestep and t["picked_time"] is None
    }
    if len(available_positions) < agent_count:
        need = agent_count + 1 - len(available_positions)
        candidates = [p for p in shelf_cells if p not in available_positions]
        if need > len(candidates):
            raise RuntimeError("Not enough unique shelf cells to allocate tasks")
        for _ in range(need):
            x, y = random.choice(candidates)
            candidates.remove((x, y))
            available_positions.add((x, y))
            tasks.append(
                {
                    "id": next_task_id,
                    "pos": (x, y),
                    "spawn_time": current_timestep,
                    "picked_time": None,
                    "delivered_time": None,
                    "picked_by": None,
                    "delivered_by": None,
                }
            )
            next_task_id += 1
    return next_task_id


def run_simulation(map_path: str, agent_count: int, max_timestep: int, output_path: str, seed: int) -> None:
    random.seed(seed)
    data = load_map(map_path)
    cells = data.get("cells")
    if not cells:
        raise ValueError("Map cells missing")
    goals = [tuple(p) for p in data.get("goals", [])]
    if not goals:
        raise ValueError("Map goals missing")

    height = len(cells)
    width = len(cells[0]) if height > 0 else 0

    shelf_cells = []
    for y in range(height):
        for x in range(width):
            if cells[y][x] == 2:
                shelf_cells.append((x, y))
    if not shelf_cells:
        raise ValueError("No shelf cells in map")

    starts = random_free_positions(cells, agent_count, avoid=goals)

    agents: Dict[int, Dict] = {}
    trajectories: Dict[int, List[Tuple[int, int]]] = {}
    for i, pos in enumerate(starts, start=1):
        agents[i] = {
            "pos": pos,
            "state": "Empty",
            "carrying": None,
        }
        trajectories[i] = [pos]

    tasks: List[Dict] = []
    next_task_id = 1
    current_timestep = 0

    next_task_id = ensure_tasks(tasks, shelf_cells, current_timestep, agent_count, next_task_id)

    grid = [[1 if cell == 1 else 0 for cell in row] for row in cells]
    drop_caps = {g: 1 for g in goals}

    while current_timestep < max_timestep:
        next_task_id = ensure_tasks(tasks, shelf_cells, current_timestep, agent_count, next_task_id)

        available_tasks = [t for t in tasks if t["spawn_time"] <= current_timestep and t["picked_time"] is None]
        unique_tasks: Dict[Tuple[int, int], Dict] = {}
        for t in available_tasks:
            pos = tuple(t["pos"])
            if pos not in unique_tasks:
                unique_tasks[pos] = t
        pickup_points = [t["pos"] for t in unique_tasks.values()]

        robots: List[RobotState] = []
        for rid in sorted(agents.keys()):
            agent = agents[rid]
            robots.append(RobotState(id=rid, pos=agent["pos"], state=agent["state"]))

        T, paths = plan_round(grid, robots, pickup_points, goals, drop_caps, T_max=max_timestep)
        if T is None:
            empty_count = sum(1 for r in robots if r.state == "Empty")
            loaded_count = sum(1 for r in robots if r.state == "Loaded")
            unique_pickups = len({tuple(p) for p in pickup_points})
            from planner import explain_infeasible
            reasons = explain_infeasible(grid, robots, pickup_points, goals, drop_caps, max_timestep)
            diagnostics = (
                f"Planning failed at timestep {current_timestep}. "
                f"empty={empty_count}, loaded={loaded_count}, "
                f"pickup_points={len(pickup_points)} (unique={unique_pickups}), "
                f"goals={len(goals)}, max_timestep={max_timestep}, "
                f"loaded_first={reasons['loaded_first']}, empty_first={reasons['empty_first']}, "
                f"loaded_only={reasons['loaded_only']}, empty_only={reasons['empty_only']}."
            )
            raise RuntimeError(diagnostics)

        arrival_times: Dict[int, int] = {}
        for r in robots:
            path = paths.get(r.id, [])
            if not path:
                arrival_times[r.id] = len(path)
                continue
            target_set = set(pickup_points) if r.state == "Empty" else set(goals)
            arrival = None
            for t, pos in enumerate(path):
                if pos in target_set:
                    arrival = t
                    break
            if arrival is None:
                arrival = len(path) - 1
            arrival_times[r.id] = arrival

        delta = min(arrival_times.values()) if arrival_times else 0
        if delta <= 0:
            delta = 1
        exceeds = current_timestep + delta > max_timestep

        for rid in sorted(agents.keys()):
            path = paths.get(rid, [])
            for step in range(1, delta + 1):
                if step < len(path):
                    trajectories[rid].append(path[step])
                elif path:
                    trajectories[rid].append(path[-1])
                else:
                    trajectories[rid].append(agents[rid]["pos"])

        current_timestep += delta

        for rid in sorted(agents.keys()):
            path = paths.get(rid, [])
            if path:
                agents[rid]["pos"] = path[min(delta, len(path) - 1)]

        for rid, arrival in arrival_times.items():
            if arrival != delta:
                continue
            agent = agents[rid]
            pos = agent["pos"]
            if agent["state"] == "Empty" and pos in pickup_points:
                for task in tasks:
                    if task["picked_time"] is None and task["pos"] == pos and task["spawn_time"] <= current_timestep:
                        task["picked_time"] = current_timestep
                        task["picked_by"] = rid
                        agent["carrying"] = task["id"]
                        agent["state"] = "Loaded"
                        break
            elif agent["state"] == "Loaded" and pos in goals:
                task_id = agent["carrying"]
                if task_id is not None:
                    for task in tasks:
                        if task["id"] == task_id:
                            task["delivered_time"] = current_timestep
                            task["delivered_by"] = rid
                            break
                agent["carrying"] = None
                agent["state"] = "Empty"

        if exceeds:
            break

    _validate_collision_free(trajectories)

    output = {
        "map": map_path,
        "max_timestep": max_timestep,
        "seed": seed,
        "agents": {
            str(rid): {
                "trajectory": trajectories[rid],
            }
            for rid in trajectories
        },
        "tasks": [
            {
                "id": t["id"],
                "pos": t["pos"],
                "spawn_time": t["spawn_time"],
                "picked_time": t["picked_time"],
                "delivered_time": t["delivered_time"],
                "picked_by": t["picked_by"],
                "delivered_by": t["delivered_by"],
            }
            for t in tasks
        ],
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)


def _validate_collision_free(trajectories: Dict[int, List[Tuple[int, int]]]) -> None:
    if not trajectories:
        return
    max_len = max(len(t) for t in trajectories.values())
    for t in range(max_len):
        occupied = {}
        for rid, traj in trajectories.items():
            pos = traj[min(t, len(traj) - 1)]
            if pos in occupied:
                raise RuntimeError(f"Vertex collision at t={t} between agents {occupied[pos]} and {rid}")
            occupied[pos] = rid
        if t == 0:
            continue
        edge_used = {}
        for rid, traj in trajectories.items():
            prev = traj[min(t - 1, len(traj) - 1)]
            curr = traj[min(t, len(traj) - 1)]
            edge = (prev, curr)
            rev = (curr, prev)
            if rev in edge_used and edge_used[rev] != rid:
                raise RuntimeError(f"Edge collision at t={t} between agents {edge_used[rev]} and {rid}")
            edge_used[edge] = rid


def main() -> None:
    parser = argparse.ArgumentParser(description="Run full MAPF simulation")
    parser.add_argument("--map", required=True, help="Path to map json")
    parser.add_argument("--agents", type=int, required=True, help="Number of agents")
    parser.add_argument("--max_timestep", type=int, required=True, help="Max timestep")
    parser.add_argument("--output", default="simulation_output.json", help="Output JSON path")
    parser.add_argument("--seed", type=int, default=0, help="Random seed")
    args = parser.parse_args()

    run_simulation(args.map, args.agents, args.max_timestep, args.output, args.seed)


if __name__ == "__main__":
    main()
