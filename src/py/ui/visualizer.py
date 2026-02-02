import tkinter as tk
from typing import Dict, List, Tuple


class PlanPlayer:
    def __init__(self, canvas: tk.Canvas, cell_size: int = 24, on_finish=None, on_step=None):
        self.canvas = canvas
        self.cell_size = cell_size
        self.paths: Dict[int, List[Tuple[int, int]]] = {}
        self.agent_ids: List[int] = []
        self.step_idx = 0
        self.max_step = None
        self.start_timestep = 0
        self.end_timestep = 0
        self.running = False
        self._after_id = None
        self.on_finish = on_finish
        self.on_step = on_step

    def load(
        self,
        paths: Dict[int, List[Tuple[int, int]]],
        agent_ids: List[int],
        start_timestep: int,
        end_timestep: int,
    ):
        self.paths = paths
        self.agent_ids = agent_ids
        self.step_idx = 0
        self.start_timestep = start_timestep
        self.end_timestep = end_timestep
        self.max_step = max(0, end_timestep - start_timestep)

    def play(self, delay_ms: int = 200):
        self.running = True
        self._tick(delay_ms)

    def pause(self):
        self.running = False
        if self._after_id is not None:
            self.canvas.after_cancel(self._after_id)
            self._after_id = None

    def step(self):
        self.running = False
        self._advance()

    def _advance(self):
        if self.max_step is not None and self.step_idx >= self.max_step:
            self.running = False
            if self.on_finish is not None:
                self.on_finish()
            return
        self.step_idx += 1
        if self.on_step is not None:
            self.on_step(self.start_timestep + self.step_idx)
        self.draw()

    def current_positions(self) -> Dict[int, Tuple[int, int]]:
        positions: Dict[int, Tuple[int, int]] = {}
        for rid in self.agent_ids:
            path = self.paths.get(rid, [])
            if not path:
                continue
            idx = min(self.step_idx, len(path) - 1)
            positions[rid] = path[idx]
        return positions

    def _tick(self, delay_ms: int):
        if not self.running:
            return
        self._advance()
        self._after_id = self.canvas.after(delay_ms, lambda: self._tick(delay_ms))

    def draw(self):
        self.canvas.delete("agent")
        self.canvas.delete("agent_static")
        for rid in self.agent_ids:
            path = self.paths.get(rid, [])
            if not path:
                continue
            idx = min(self.step_idx, len(path) - 1)
            x, y = path[idx]
            x0 = x * self.cell_size
            y0 = y * self.cell_size
            x1 = x0 + self.cell_size
            y1 = y0 + self.cell_size
            self.canvas.create_oval(
                x0 + 4, y0 + 4, x1 - 4, y1 - 4, fill="#2c7be5", outline="", tags="agent"
            )
