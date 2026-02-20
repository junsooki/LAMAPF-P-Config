from dataclasses import dataclass, field
from typing import Dict, List, Tuple

# Direction constants
DIR_EAST = 0
DIR_WEST = 1
DIR_SOUTH = 2
DIR_NORTH = 3

# (dx, dy) -> direction
DELTA_TO_DIR = {
    (1, 0): DIR_EAST,
    (-1, 0): DIR_WEST,
    (0, 1): DIR_SOUTH,
    (0, -1): DIR_NORTH,
}

# direction -> (dx, dy)
DIR_TO_DELTA = {
    DIR_EAST: (1, 0),
    DIR_WEST: (-1, 0),
    DIR_SOUTH: (0, 1),
    DIR_NORTH: (0, -1),
}


@dataclass
class RobotState:
    id: int
    pos: Tuple[int, int]
    state: str  # "Empty" or "Loaded"
    facing: int = DIR_EAST


@dataclass
class PlannerState:
    grid: List[List[int]]
    robots: List[RobotState]
    pickup_points: List[Tuple[int, int]]
    drop_points: List[Tuple[int, int]]
    drop_caps: Dict[Tuple[int, int], int]
