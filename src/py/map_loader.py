from typing import List


def load_grid_from_txt(path: str) -> List[List[int]]:
    lines = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            if not line:
                continue
            lines.append(line)
    grid = []
    for line in lines:
        row = []
        for ch in line:
            if ch in ("0", "."):
                row.append(0)
            elif ch in ("1", "#"):
                row.append(1)
            else:
                row.append(1)
        grid.append(row)
    return normalize_grid(grid)


def normalize_grid(grid: List[List[int]]) -> List[List[int]]:
    if not grid:
        return []
    width = len(grid[0])
    normalized = []
    for row in grid:
        if len(row) != width:
            raise ValueError("Grid must be rectangular")
        normalized.append([0 if int(cell) == 0 else 1 for cell in row])
    return normalized
