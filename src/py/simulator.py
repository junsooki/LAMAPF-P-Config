from typing import Dict, List, Tuple

from data_types import PlannerState, RobotState


def apply_paths(state: PlannerState, paths: Dict[int, List[Tuple[int, int]]], steps: int) -> None:
    id_to_robot = {r.id: r for r in state.robots}
    for rid, path in paths.items():
        if rid not in id_to_robot or not path:
            continue
        idx = min(steps, len(path) - 1)
        id_to_robot[rid].pos = path[idx]


def update_states_on_event(
    state: PlannerState,
    pickup_points: List[Tuple[int, int]],
    drop_points: List[Tuple[int, int]],
) -> None:
    pickup_set = set(pickup_points)
    drop_set = set(drop_points)
    for robot in state.robots:
        if robot.state == "Empty" and robot.pos in pickup_set:
            robot.state = "Loaded"
        elif robot.state == "Loaded" and robot.pos in drop_set:
            robot.state = "Empty"


def run_simulation(initial_state: PlannerState, planner, max_steps: int):
    state = initial_state
    step = 0
    history = []
    while step < max_steps:
        T, paths = planner(
            state.grid,
            state.robots,
            state.pickup_points,
            state.drop_points,
            state.drop_caps,
            max_steps - step,
        )
        if T is None:
            break
        pickup_set = set(state.pickup_points)
        drop_set = set(state.drop_points)
        earliest = None
        for robot in state.robots:
            path = paths.get(robot.id, [])
            if not path:
                continue
            target_set = pickup_set if robot.state == "Empty" else drop_set
            arrival = None
            for t, pos in enumerate(path):
                if pos in target_set:
                    arrival = t
                    break
            if arrival is None:
                arrival = len(path) - 1
            if earliest is None or arrival < earliest:
                earliest = arrival
        if earliest is None:
            break
        apply_paths(state, paths, earliest)
        update_states_on_event(state, state.pickup_points, state.drop_points)
        history.append(paths)
        step += earliest
    return state, history
