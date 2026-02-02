from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class RobotState:
    id: int
    pos: Tuple[int, int]
    state: str  # "Empty" or "Loaded"


@dataclass
class PlannerState:
    grid: List[List[int]]
    robots: List[RobotState]
    pickup_points: List[Tuple[int, int]]
    drop_points: List[Tuple[int, int]]
    drop_caps: Dict[Tuple[int, int], int]
