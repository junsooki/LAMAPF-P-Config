#!/usr/bin/env python3
"""CLI bridge: read Moving AI .map/.scen inputs, plan with LAMAPF-P, export SMART paths."""

import argparse
import sys
from typing import Dict, List, Tuple

from smart_io import load_movingai_map, load_scen, export_paths_smart
from planner import _find_min_T_single


def plan_standard_mapf(
    grid: List[List[int]],
    starts: List[Tuple[int, int]],
    goals: List[Tuple[int, int]],
    T_max: int,
    method: str = "dinic",
    verbose: bool = False,
) -> Tuple[int, Dict[int, List[Tuple[int, int]]]]:
    """Plan collision-free paths using LAMAPF-P's network flow solver.

    Returns (T, paths_dict) where paths_dict maps 0-indexed agent IDs to paths.
    """
    caps = [1] * len(goals)
    reserved_v: List[Tuple[int, int, int]] = []
    reserved_e: List[Tuple[int, int, int, int, int]] = []

    T, paths = _find_min_T_single(
        grid, starts, goals, caps, reserved_v, reserved_e, T_max,
        method=method, verbose=verbose,
    )

    if T is None:
        return None, {}

    paths_dict: Dict[int, List[Tuple[int, int]]] = {}
    for idx, path in enumerate(paths):
        paths_dict[idx] = path

    return T, paths_dict


def main():
    parser = argparse.ArgumentParser(
        description="Bridge LAMAPF-P planner to SMART-MAPF format"
    )
    parser.add_argument("--map", required=True, help="Moving AI .map file")
    parser.add_argument("--scen", required=True, help="Moving AI .scen file")
    parser.add_argument("--agents", type=int, default=None, help="Number of agents (default: all in .scen)")
    parser.add_argument("--output", required=True, help="Output path file (SMART format)")
    parser.add_argument("--T_max", type=int, default=200, help="Maximum makespan (default: 200)")
    parser.add_argument("--solver", default="dinic", help="Flow solver method (default: dinic)")
    parser.add_argument("--verbose", action="store_true", help="Print solver progress")

    args = parser.parse_args()

    grid = load_movingai_map(args.map)
    height = len(grid)
    width = len(grid[0]) if height else 0
    print(f"Grid: {width}x{height}")

    agent_data = load_scen(args.scen, num_agents=args.agents)
    num_agents = len(agent_data)
    print(f"Agents: {num_agents}")

    starts = [s for s, _ in agent_data]
    goals = [g for _, g in agent_data]

    T, paths = plan_standard_mapf(
        grid, starts, goals, args.T_max,
        method=args.solver, verbose=args.verbose,
    )

    if T is None:
        print(f"No solution found within T_max={args.T_max}")
        sys.exit(1)

    print(f"Optimal makespan: T={T}")

    export_paths_smart(paths, args.output, continuous=True)
    print(f"Paths written to: {args.output}")
    print(f"Note: Use --flip_coord=0 with SMART (LAMAPF-P uses xy coordinates)")


if __name__ == "__main__":
    main()
