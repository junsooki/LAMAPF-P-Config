"""Parsers for Moving AI .map/.scen files and SMART-MAPF path exporter."""

from typing import List, Tuple, Dict, Optional

from map_loader import normalize_grid


def load_movingai_map(path: str) -> List[List[int]]:
    """Parse a Moving AI .map file into grid[row][col] (0=passable, 1=obstacle).

    Format:
        type octile
        height H
        width W
        map
        <grid chars: . G S = passable, @ T W O = obstacle>
    """
    passable_chars = frozenset(".GS")
    height = None
    width = None
    grid_lines: List[str] = []
    reading_grid = False

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n").rstrip("\r")
            if reading_grid:
                if line:
                    grid_lines.append(line)
                continue
            lower = line.strip().lower()
            if lower.startswith("height"):
                height = int(lower.split()[1])
            elif lower.startswith("width"):
                width = int(lower.split()[1])
            elif lower == "map":
                reading_grid = True

    if height is None or width is None:
        raise ValueError("Missing height/width in .map header")

    if len(grid_lines) != height:
        raise ValueError(
            f"Dimension mismatch: header says height={height}, got {len(grid_lines)} rows"
        )

    grid: List[List[int]] = []
    for r, line in enumerate(grid_lines):
        if len(line) != width:
            raise ValueError(
                f"Dimension mismatch: header says width={width}, row {r} has {len(line)} chars"
            )
        row = [0 if ch in passable_chars else 1 for ch in line]
        grid.append(row)

    return normalize_grid(grid)


def load_scen(
    path: str, num_agents: Optional[int] = None
) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
    """Parse a Moving AI .scen file.

    Format (TSV):
        version 1
        bucket  map_file  width  height  startx  starty  goalx  goaly  optimal_length

    Returns list of ((start_x, start_y), (goal_x, goal_y)) where x=col, y=row
    (matching LAMAPF-P (x,y) convention). Columns 4-7 from TSV.
    """
    agents: List[Tuple[Tuple[int, int], Tuple[int, int]]] = []

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("version"):
                continue
            parts = line.split("\t")
            if len(parts) < 9:
                parts = line.split()
            if len(parts) < 9:
                continue
            start_x = int(parts[4])
            start_y = int(parts[5])
            goal_x = int(parts[6])
            goal_y = int(parts[7])
            agents.append(((start_x, start_y), (goal_x, goal_y)))

    if num_agents is not None:
        agents = agents[:num_agents]

    return agents


def export_paths_smart(
    paths: Dict[int, List[Tuple[int, int]]],
    output_path: str,
    continuous: bool = True,
) -> None:
    """Export paths in SMART format.

    Continuous (default, what SMART's ADG server expects):
        Agent 0:(x0,y0,0)->(x1,y1,1)->(x2,y2,2)->

    Discrete (no time):
        Agent 0:(x0,y0)->(x1,y1)->(x2,y2)->

    Agents are 0-indexed (SMART convention).
    """
    sorted_ids = sorted(paths.keys())

    lines: List[str] = []
    for idx, agent_id in enumerate(sorted_ids):
        waypoints = paths[agent_id]
        if continuous:
            parts = [f"({x},{y},{t})" for t, (x, y) in enumerate(waypoints)]
        else:
            parts = [f"({x},{y})" for x, y in waypoints]
        line = f"Agent {idx}:" + "->".join(parts) + "->"
        lines.append(line)

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
