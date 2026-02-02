import argparse
import json
import os
import random
import sys
from typing import Dict, List, Tuple

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), ".")))

from data_types import RobotState
from planner import plan_round_sync


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


def run_simulation(
    map_path: str,
    agent_count: int,
    max_timestep: int,
    output_path: str,
    seed: int,
    debug: bool = False,
    debug_every: int = 25,
) -> None:
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

        if debug:
            print(
                f"[sync] timestep={current_timestep} agents={agent_count} "
                f"pickups={len(pickup_points)} goals={len(goals)}"
            )

        robots: List[RobotState] = []
        for rid in sorted(agents.keys()):
            agent = agents[rid]
            robots.append(RobotState(id=rid, pos=agent["pos"], state="Empty"))

        T, tau, paths = plan_round_sync(
            grid,
            robots,
            pickup_points,
            goals,
            drop_caps,
            T_max=max_timestep,
            verbose=debug,
            progress_every=debug_every,
        )
        if T is None or tau is None:
            diagnostics = (
                f"Sync planning failed at timestep {current_timestep}. "
                f"agents={agent_count}, pickup_points={len(pickup_points)}, goals={len(goals)}, "
                f"max_timestep={max_timestep}."
            )
            raise RuntimeError(diagnostics)

        if debug:
            print(f"[sync] phase tau={tau} total={T}")
        _validate_sync_round(grid, paths, pickup_points, goals, tau, T)

        delta = max(1, T)
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

        for rid in sorted(agents.keys()):
            path = paths.get(rid, [])
            if path:
                agents[rid]["pos"] = path[min(delta, len(path) - 1)]

        pickup_time = current_timestep + tau
        task_by_pos = {tuple(t["pos"]): t for t in available_tasks}
        assigned: Dict[int, int] = {}
        for rid in sorted(agents.keys()):
            path = paths.get(rid, [])
            if not path:
                continue
            pos_tau = path[min(tau, len(path) - 1)]
            task = task_by_pos.get(tuple(pos_tau))
            if task is None:
                continue
            if task["picked_time"] is None:
                task["picked_time"] = pickup_time
                task["picked_by"] = rid
                assigned[rid] = task["id"]

        drop_time = current_timestep + T
        for rid, task_id in assigned.items():
            for task in tasks:
                if task["id"] == task_id:
                    task["delivered_time"] = drop_time
                    task["delivered_by"] = rid
                    break

        current_timestep += delta

        if exceeds:
            break

    _validate_collision_free(trajectories)

    stats = _compute_stats(tasks, trajectories)

    output = {
        "map": map_path,
        "max_timestep": max_timestep,
        "seed": seed,
        "stats": stats,
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


def _compute_stats(tasks: List[Dict], trajectories: Dict[int, List[Tuple[int, int]]]) -> Dict:
    def _avg(values: List[int]):
        if not values:
            return None
        return sum(values) / len(values)

    sim_end_timestep = 0
    if trajectories:
        sim_end_timestep = max(len(traj) - 1 for traj in trajectories.values())

    tasks_total = len(tasks)
    picked = [t for t in tasks if t["picked_time"] is not None]
    delivered = [t for t in tasks if t["delivered_time"] is not None]

    pickup_waits = [t["picked_time"] - t["spawn_time"] for t in picked]
    delivery_times = [t["delivered_time"] - t["spawn_time"] for t in delivered]
    carry_times = [
        t["delivered_time"] - t["picked_time"]
        for t in delivered
        if t["picked_time"] is not None
    ]

    idle_steps = 0
    total_steps = 0
    for traj in trajectories.values():
        for i in range(1, len(traj)):
            total_steps += 1
            if traj[i] == traj[i - 1]:
                idle_steps += 1

    backlog_over_time: List[int] = []
    for t in range(sim_end_timestep + 1):
        backlog = 0
        for task in tasks:
            spawn = task["spawn_time"]
            picked_time = task["picked_time"]
            if spawn <= t and (picked_time is None or picked_time > t):
                backlog += 1
        backlog_over_time.append(backlog)

    return {
        "sim_end_timestep": sim_end_timestep,
        "agents": len(trajectories),
        "tasks_total": tasks_total,
        "tasks_picked": len(picked),
        "tasks_delivered": len(delivered),
        "tasks_unpicked": tasks_total - len(picked),
        "tasks_undelivered": tasks_total - len(delivered),
        "throughput_picked_per_timestep": len(picked) / max(1, sim_end_timestep),
        "throughput_delivered_per_timestep": len(delivered) / max(1, sim_end_timestep),
        "avg_pickup_wait": _avg(pickup_waits),
        "avg_delivery_time": _avg(delivery_times),
        "avg_carry_time": _avg(carry_times),
        "avg_backlog": _avg(backlog_over_time),
        "max_backlog": max(backlog_over_time) if backlog_over_time else 0,
        "idle_ratio": idle_steps / total_steps if total_steps > 0 else None,
        "move_ratio": (total_steps - idle_steps) / total_steps if total_steps > 0 else None,
    }


def _validate_sync_round(
    grid: List[List[int]],
    paths_by_id: Dict[int, List[Tuple[int, int]]],
    pickup_points: List[Tuple[int, int]],
    drop_points: List[Tuple[int, int]],
    tau: int,
    T: int,
) -> None:
    height = len(grid)
    width = len(grid[0]) if height > 0 else 0
    pickups = set(pickup_points)
    drops = set(drop_points)
    for rid, path in paths_by_id.items():
        if len(path) != T + 1:
            raise RuntimeError(f"Sync plan path length invalid for agent {rid}: len={len(path)} T={T}")
        for t in range(1, len(path)):
            x0, y0 = path[t - 1]
            x1, y1 = path[t]
            if not (0 <= x1 < width and 0 <= y1 < height):
                raise RuntimeError(f"Sync plan out of bounds at t={t} for agent {rid}: {(x1, y1)}")
            if grid[y1][x1] != 0:
                raise RuntimeError(f"Sync plan hit obstacle at t={t} for agent {rid}: {(x1, y1)}")
            if abs(x0 - x1) + abs(y0 - y1) not in (0, 1):
                raise RuntimeError(f"Sync plan invalid move at t={t} for agent {rid}: {path[t-1]}->{path[t]}")
        if path[tau] not in pickups:
            raise RuntimeError(
                f"Sync plan missing pickup at tau={tau} for agent {rid}: {path[tau]}"
            )
        if path[T] not in drops:
            raise RuntimeError(f"Sync plan missing drop at T={T} for agent {rid}: {path[T]}")
    _validate_collision_free({rid: path for rid, path in paths_by_id.items()})


def main() -> None:
    parser = argparse.ArgumentParser(description="Run full MAPF simulation (sync two-stage model)")
    parser.add_argument("--map", required=True, help="Path to map json")
    parser.add_argument("--agents", type=int, required=True, help="Number of agents")
    parser.add_argument("--max_timestep", type=int, required=True, help="Max timestep")
    parser.add_argument("--output", default="simulation_output.json", help="Output JSON path")
    parser.add_argument("--seed", type=int, default=0, help="Random seed")
    parser.add_argument("--debug", action="store_true", help="Print sync search progress")
    parser.add_argument("--debug_every", type=int, default=25, help="Tau progress print interval")
    args = parser.parse_args()

    run_simulation(
        args.map,
        args.agents,
        args.max_timestep,
        args.output,
        args.seed,
        debug=args.debug,
        debug_every=args.debug_every,
    )


if __name__ == "__main__":
    main()
