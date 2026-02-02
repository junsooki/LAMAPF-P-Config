from typing import Dict, List, Tuple

from concurrent.futures import ThreadPoolExecutor
from collections import deque
import os
import sys

from data_types import RobotState
from utils import pad_path


def _import_flow_planner():
    try:
        import flow_planner_cpp  # type: ignore
        return flow_planner_cpp
    except ImportError:
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        candidates = [
            os.path.join(root, "build"),
            os.path.join(root, "build", "Release"),
            os.path.join(root, "build", "Debug"),
        ]
        for path in candidates:
            if not os.path.isdir(path):
                continue
            for name in os.listdir(path):
                if name.startswith("flow_planner_cpp") and (
                    name.endswith(".so") or name.endswith(".pyd") or name.endswith(".dylib")
                ):
                    sys.path.append(path)
                    try:
                        import flow_planner_cpp  # type: ignore
                        return flow_planner_cpp
                    except ImportError:
                        pass
        raise


flow_planner_cpp = _import_flow_planner()

_GRID_CACHE: Dict[Tuple, Dict] = {}
_DIST_CACHE: Dict[Tuple, List[int]] = {}


def _grid_key(grid: List[List[int]]) -> Tuple:
    height = len(grid)
    width = len(grid[0]) if height > 0 else 0
    rows = tuple(tuple(row) for row in grid)
    return (height, width, rows)


def _get_grid_cache(grid: List[List[int]]) -> Dict:
    key = _grid_key(grid)
    cached = _GRID_CACHE.get(key)
    if cached is not None:
        return cached
    height = len(grid)
    width = len(grid[0]) if height > 0 else 0
    passable = []
    for y in range(height):
        for x in range(width):
            passable.append(grid[y][x] == 0)
    neighbors: List[List[int]] = [[] for _ in range(height * width)]
    for y in range(height):
        for x in range(width):
            idx = y * width + x
            if not passable[idx]:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = x + dx
                ny = y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                nidx = ny * width + nx
                if not passable[nidx]:
                    continue
                neighbors[idx].append(nidx)
    cached = {
        "key": key,
        "width": width,
        "height": height,
        "passable": passable,
        "neighbors": neighbors,
    }
    _GRID_CACHE[key] = cached
    return cached


def _bfs_multi_source(grid_cache: Dict, sources: List[Tuple[int, int]], use_cache: bool = True) -> List[int]:
    width = grid_cache["width"]
    height = grid_cache["height"]
    neighbors = grid_cache["neighbors"]
    passable = grid_cache["passable"]
    n = width * height
    if not sources:
        return [-1] * n
    src_key = tuple(sorted(sources))
    cache_key = (grid_cache["key"], src_key)
    if use_cache:
        cached = _DIST_CACHE.get(cache_key)
        if cached is not None:
            return cached
    dist = [-1] * n
    q = deque()
    for x, y in sources:
        if x < 0 or y < 0 or x >= width or y >= height:
            raise ValueError(f"Source out of bounds: {(x, y)}")
        idx = y * width + x
        if not passable[idx]:
            raise ValueError(f"Source on blocked cell: {(x, y)}")
        if dist[idx] == 0:
            continue
        dist[idx] = 0
        q.append(idx)
    while q:
        cur = q.popleft()
        for nb in neighbors[cur]:
            if dist[nb] != -1:
                continue
            dist[nb] = dist[cur] + 1
            q.append(nb)
    if use_cache:
        _DIST_CACHE[cache_key] = dist
    return dist


def build_reserved_vertices(paths: List[List[Tuple[int, int]]]) -> List[Tuple[int, int, int]]:
    reserved = []
    for path in paths:
        for t, (x, y) in enumerate(path):
            reserved.append((x, y, t))
    return reserved


def build_reserved_edges(paths: List[List[Tuple[int, int]]]) -> List[Tuple[int, int, int, int, int]]:
    reserved = []
    for path in paths:
        for t in range(len(path) - 1):
            x1, y1 = path[t]
            x2, y2 = path[t + 1]
            if (x1, y1) == (x2, y2):
                continue
            reserved.append((x1, y1, x2, y2, t))
    return reserved


def _find_min_T_single(
    grid: List[List[int]],
    starts: List[Tuple[int, int]],
    targets: List[Tuple[int, int]],
    caps: List[int],
    reserved_v: List[Tuple[int, int, int]],
    reserved_e: List[Tuple[int, int, int, int, int]],
    T_max: int,
    method: str = "dinic",
    verbose: bool = False,
):
    if not starts:
        return 0, []
    if T_max < 0:
        return None, []

    def feasible(T: int):
        if verbose:
            print(f"[flow] T={T}")
        res = flow_planner_cpp.plan_flow(grid, starts, targets, caps, T, reserved_v, reserved_e, method)
        return res["feasible"], res["paths"]

    low = 0
    high = 1
    best_paths = []

    while high <= T_max:
        ok, paths = feasible(high)
        if ok:
            best_paths = paths
            break
        low = high + 1
        high *= 2

    if high > T_max:
        ok, paths = feasible(T_max)
        if not ok:
            return None, []
        high = T_max
        best_paths = paths

    while low <= high:
        mid = (low + high) // 2
        ok, paths = feasible(mid)
        if ok:
            best_paths = paths
            high = mid - 1
        else:
            low = mid + 1

    return low, best_paths


def _plan_with_order(
    grid: List[List[int]],
    robots: List[RobotState],
    pickup_points: List[Tuple[int, int]],
    drop_points: List[Tuple[int, int]],
    drop_caps: Dict[Tuple[int, int], int],
    T: int,
    first_loaded: bool,
    method: str = "dinic",
):
    loaded = [r for r in robots if r.state == "Loaded"]
    empty = [r for r in robots if r.state == "Empty"]
    drop_caps_list = [drop_caps.get(p, 1) for p in drop_points]

    def plan_loaded(reserved_v, reserved_e):
        if not loaded:
            return True, [], [], []
        t_loaded, paths_loaded = _find_min_T_single(
            grid,
            [r.pos for r in loaded],
            drop_points,
            drop_caps_list,
            reserved_v,
            reserved_e,
            T,
            method=method,
        )
        if t_loaded is None:
            return False, [], [], []
        return True, paths_loaded, build_reserved_vertices(paths_loaded), build_reserved_edges(paths_loaded)

    def plan_empty(reserved_v, reserved_e):
        if not empty:
            return True, []
        t_empty, paths_empty = _find_min_T_single(
            grid,
            [r.pos for r in empty],
            pickup_points,
            [1] * len(pickup_points),
            reserved_v,
            reserved_e,
            T,
            method=method,
        )
        if t_empty is None:
            return False, []
        return True, paths_empty

    if first_loaded:
        ok_l, paths_loaded, res_v, res_e = plan_loaded([], [])
        if not ok_l:
            return False, {}, "loaded_stage_infeasible"
        ok_e, paths_empty = plan_empty(res_v, res_e)
        if not ok_e:
            return False, {}, "empty_stage_infeasible"
    else:
        ok_e, paths_empty = plan_empty([], [])
        if not ok_e:
            return False, {}, "empty_stage_infeasible"
        res_v = build_reserved_vertices(paths_empty)
        res_e = build_reserved_edges(paths_empty)
        ok_l, paths_loaded, _, _ = plan_loaded(res_v, res_e)
        if not ok_l:
            return False, {}, "loaded_stage_infeasible"

    paths_by_id: Dict[int, List[Tuple[int, int]]] = {}
    for robot, path in zip(loaded, paths_loaded):
        paths_by_id[robot.id] = pad_path(path, T)
    for robot, path in zip(empty, paths_empty):
        paths_by_id[robot.id] = pad_path(path, T)
    return True, paths_by_id, ""


def explain_infeasible(
    grid: List[List[int]],
    robots: List[RobotState],
    pickup_points: List[Tuple[int, int]],
    drop_points: List[Tuple[int, int]],
    drop_caps: Dict[Tuple[int, int], int],
    T: int,
    method: str = "dinic",
):
    loaded = [r for r in robots if r.state == "Loaded"]
    empty = [r for r in robots if r.state == "Empty"]
    drop_caps_list = [drop_caps.get(p, 1) for p in drop_points]
    loaded_only = True
    empty_only = True
    if loaded:
        res_l = flow_planner_cpp.plan_flow(
            grid, [r.pos for r in loaded], drop_points, drop_caps_list, T, [], [], method
        )
        loaded_only = res_l["feasible"]
    if empty:
        res_e = flow_planner_cpp.plan_flow(
            grid, [r.pos for r in empty], pickup_points, [1] * len(pickup_points), T, [], [], method
        )
        empty_only = res_e["feasible"]
    ok1, _, reason1 = _plan_with_order(grid, robots, pickup_points, drop_points, drop_caps, T, True, method)
    ok2, _, reason2 = _plan_with_order(grid, robots, pickup_points, drop_points, drop_caps, T, False, method)
    return {
        "loaded_first": "ok" if ok1 else reason1,
        "empty_first": "ok" if ok2 else reason2,
        "loaded_only": "ok" if loaded_only else "infeasible",
        "empty_only": "ok" if empty_only else "infeasible",
    }


def search_min_T(
    grid: List[List[int]],
    robots: List[RobotState],
    pickup_points: List[Tuple[int, int]],
    drop_points: List[Tuple[int, int]],
    drop_caps: Dict[Tuple[int, int], int],
    T_max: int,
    method: str = "dinic",
):
    loaded = [r for r in robots if r.state == "Loaded"]
    empty = [r for r in robots if r.state == "Empty"]

    drop_caps_list = [drop_caps.get(p, 1) for p in drop_points]

    def try_T(T: int):
        ok, paths, _ = _plan_with_order(grid, robots, pickup_points, drop_points, drop_caps, T, True, method)
        if ok:
            return True, paths
        ok, paths, _ = _plan_with_order(grid, robots, pickup_points, drop_points, drop_caps, T, False, method)
        return ok, paths

    if T_max < 0:
        return None, {}

    low = 0
    high = 1
    best_paths: Dict[int, List[Tuple[int, int]]] = {}

    while high <= T_max:
        ok, paths = try_T(high)
        if ok:
            best_paths = paths
            break
        low = high + 1
        high *= 2

    if high > T_max:
        ok, paths = try_T(T_max)
        if not ok:
            return None, {}
        high = T_max
        best_paths = paths

    while low <= high:
        mid = (low + high) // 2
        ok, paths = try_T(mid)
        if ok:
            best_paths = paths
            high = mid - 1
        else:
            low = mid + 1

    return low, best_paths


def plan_round(
    grid: List[List[int]],
    robots: List[RobotState],
    pickup_points: List[Tuple[int, int]],
    drop_points: List[Tuple[int, int]],
    drop_caps: Dict[Tuple[int, int], int],
    T_max: int,
    method: str = "dinic",
):
    return search_min_T(grid, robots, pickup_points, drop_points, drop_caps, T_max, method=method)


def search_min_T_sync(
    grid: List[List[int]],
    robots: List[RobotState],
    pickup_points: List[Tuple[int, int]],
    drop_points: List[Tuple[int, int]],
    drop_caps: Dict[Tuple[int, int], int],
    T_max: int,
    method: str = "dinic",
    parallel_workers: int = 1,
    verbose: bool = False,
    progress_every: int = 25,
):
    if not robots:
        return 0, 0, {}
    if T_max < 0:
        return None, None, {}

    starts = [r.pos for r in robots]
    drop_caps_list = [drop_caps.get(p, 1) for p in drop_points]

    grid_cache = _get_grid_cache(grid)
    dist_to_pick = _bfs_multi_source(grid_cache, pickup_points, use_cache=False)
    dist_to_drop = _bfs_multi_source(grid_cache, drop_points, use_cache=True)

    width = grid_cache["width"]
    tau_min = 0
    for sx, sy in starts:
        idx = sy * width + sx
        if dist_to_pick[idx] < 0:
            return None, None, {}
        tau_min = max(tau_min, dist_to_pick[idx])

    pickup_drop_dists = []
    for px, py in pickup_points:
        idx = py * width + px
        if dist_to_drop[idx] >= 0:
            pickup_drop_dists.append(dist_to_drop[idx])
    pickup_drop_dists.sort()
    if len(pickup_drop_dists) < len(robots):
        return None, None, {}
    min_drop_needed = pickup_drop_dists[len(robots) - 1]

    def try_T(T: int):
        tau_max = T - min_drop_needed
        if tau_max < tau_min:
            return False, None, {}
        if verbose:
            print(f"[sync-search] T={T}/{T_max} tau={tau_min}..{tau_max}")
        if parallel_workers <= 1:
            for tau in range(tau_min, tau_max + 1):
                if verbose and progress_every > 0 and tau % progress_every == 0 and tau != 0:
                    print(f"[sync-search] T={T} tau={tau}/{T}")
                res = flow_planner_cpp.plan_flow_sync(
                    grid, starts, pickup_points, drop_points, drop_caps_list, T, tau, method
                )
                if not res["feasible"]:
                    continue
                paths_by_id: Dict[int, List[Tuple[int, int]]] = {}
                for robot, path in zip(robots, res["paths"]):
                    paths_by_id[robot.id] = pad_path(path, T)
                return True, tau, paths_by_id
            return False, None, {}

        def solve_tau(tau: int):
            res = flow_planner_cpp.plan_flow_sync(
                grid, starts, pickup_points, drop_points, drop_caps_list, T, tau, method
            )
            return tau, res

        batch_size = max(1, parallel_workers)
        with ThreadPoolExecutor(max_workers=parallel_workers) as executor:
            for batch_start in range(tau_min, tau_max + 1, batch_size):
                batch_end = min(tau_max + 1, batch_start + batch_size)
                batch = list(range(batch_start, batch_end))
                if verbose and progress_every > 0 and batch_start % progress_every == 0 and batch_start != 0:
                    print(f"[sync-search] T={T} tau={batch_start}/{T}")
                results = list(executor.map(solve_tau, batch))
                for tau, res in results:
                    if not res["feasible"]:
                        continue
                    paths_by_id: Dict[int, List[Tuple[int, int]]] = {}
                    for robot, path in zip(robots, res["paths"]):
                        paths_by_id[robot.id] = pad_path(path, T)
                    return True, tau, paths_by_id
        return False, None, {}

    lower_T = max(tau_min + min_drop_needed, 0)
    if lower_T > T_max:
        return None, None, {}

    if lower_T == 0:
        ok, tau, paths = try_T(0)
        if ok:
            return 0, tau, paths
    last_fail = lower_T - 1
    high = max(1, lower_T)

    while high <= T_max:
        ok, tau, paths = try_T(high)
        if ok:
            break
        last_fail = high
        high *= 2

    if high > T_max:
        ok, tau, paths = try_T(T_max)
        if not ok:
            return None, None, {}
        high = T_max

    for T in range(last_fail + 1, high + 1):
        ok, tau, paths = try_T(T)
        if ok:
            return T, tau, paths

    return None, None, {}


def plan_round_sync(
    grid: List[List[int]],
    robots: List[RobotState],
    pickup_points: List[Tuple[int, int]],
    drop_points: List[Tuple[int, int]],
    drop_caps: Dict[Tuple[int, int], int],
    T_max: int,
    method: str = "dinic",
    parallel_workers: int = 1,
    verbose: bool = False,
    progress_every: int = 25,
):
    if len(drop_points) < len(robots):
        raise RuntimeError(
            f"Sync model requires goals >= agents (goals={len(drop_points)}, agents={len(robots)})"
        )
    return search_min_T_sync(
        grid,
        robots,
        pickup_points,
        drop_points,
        drop_caps,
        T_max,
        method=method,
        parallel_workers=parallel_workers,
        verbose=verbose,
        progress_every=progress_every,
    )
