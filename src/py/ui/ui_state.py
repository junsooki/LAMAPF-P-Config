from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class UIState:
    width: int
    height: int
    cells: List[List[int]]
    agents: List[Tuple[int, int]]
    goals: List[Tuple[int, int]]


def empty_state(width: int, height: int) -> UIState:
    cells = [[0 for _ in range(width)] for _ in range(height)]
    return UIState(width=width, height=height, cells=cells, agents=[], goals=[])
