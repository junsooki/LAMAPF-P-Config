import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog
from typing import Dict, List, Tuple

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from map_store import list_maps, load_map, save_map
from sim_player import load_sim_output
from ui_state import UIState, empty_state

CELL_COLORS = {
    0: "#ffffff",
    1: "#333333",
    2: "#8d6e63",
}

GOAL_COLOR = "#4caf50"
AGENT_COLOR = "#2196f3"
CARGO_COLOR = "#ff9800"
GRID_LINE = "#cccccc"


class MapEditor:
    def __init__(self, root: tk.Tk, maps_dir: str):
        self.root = root
        self.maps_dir = maps_dir
        self.state = empty_state(12, 12)
        self.tool = tk.StringVar(value="none")
        self.selected_map = None
        self.cell_size = 24
        self.min_cell_size = 8
        self.max_cell_size = 64
        self.view_offset_x = 0.0
        self.view_offset_y = 0.0
        self._panning = False
        self._pan_last = (0, 0)
        self._space_down = False

        self.sim_mode = False
        self.sim_tasks: List[Dict] = []
        self.sim_trajectories: Dict[int, List[Tuple[int, int]]] = {}
        self.sim_max_timestep = 0
        self.sim_timestep_var = tk.IntVar(value=0)
        self.sim_running = False
        self.sim_after_id = None
        self.cargo_goals: List[Tuple[int, int]] = []

        self._build_ui()
        self._load_map_list()
        self._draw_all()

    def _build_ui(self):
        self.root.title("MAPF Map Editor")

        left = tk.Frame(self.root)
        left.pack(side=tk.LEFT, fill=tk.Y)

        tk.Label(left, text="Maps").pack(anchor=tk.W)
        self.map_list = tk.Listbox(left, height=10)
        self.map_list.pack(fill=tk.X)
        self.map_list.bind("<<ListboxSelect>>", self._on_select_map)

        tk.Button(left, text="New", command=self._new_map).pack(fill=tk.X)
        tk.Button(left, text="Save", command=self._save_map).pack(fill=tk.X)
        tk.Button(left, text="Save As", command=self._save_as).pack(fill=tk.X)

        tk.Label(left, text="Tools").pack(anchor=tk.W, pady=(8, 0))
        for tool in ["none", "wall", "shelf", "agent", "goal", "erase"]:
            tk.Radiobutton(left, text=tool, variable=self.tool, value=tool).pack(anchor=tk.W)

        tk.Label(left, text="Replay").pack(anchor=tk.W, pady=(8, 0))
        tk.Button(left, text="Load Sim", command=self._load_sim).pack(fill=tk.X)
        tk.Button(left, text="Play", command=self._play_sim).pack(fill=tk.X)
        tk.Button(left, text="Pause", command=self._pause_sim).pack(fill=tk.X)
        tk.Button(left, text="Step", command=self._step_sim).pack(fill=tk.X)
        tk.Button(left, text="Back", command=self._back_sim).pack(fill=tk.X)

        self.timestep_label = tk.Label(left, text="T=0", width=24, anchor="w")
        self.timestep_label.pack(fill=tk.X, pady=(8, 0))

        self.throughput_label = tk.Label(left, text="", width=24, anchor="w")
        self.throughput_label.pack(fill=tk.X)

        self.sim_scale = tk.Scale(
            left,
            from_=0,
            to=0,
            orient=tk.HORIZONTAL,
            variable=self.sim_timestep_var,
            command=self._on_sim_scale,
            state="disabled",
        )
        self.sim_scale.pack(fill=tk.X, pady=(4, 0))

        tk.Label(left, text="Zoom").pack(anchor=tk.W, pady=(8, 0))
        self.zoom_scale = tk.Scale(
            left,
            from_=self.min_cell_size,
            to=self.max_cell_size,
            orient=tk.HORIZONTAL,
            command=self._on_zoom_scale,
        )
        self.zoom_scale.set(self.cell_size)
        self.zoom_scale.pack(fill=tk.X, pady=(4, 0))

        self.status_label = tk.Label(left, text="Hover agent to see state", width=24, anchor="w")
        self.status_label.pack(fill=tk.X, pady=(8, 0))

        right = tk.Frame(self.root)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(right, bg="#f5f5f5")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_left_up)
        self.canvas.bind("<Motion>", self._on_hover)
        self.canvas.bind("<MouseWheel>", self._on_zoom)
        self.canvas.bind("<Button-4>", self._on_zoom)
        self.canvas.bind("<Button-5>", self._on_zoom)
        self.root.bind("<KeyPress-space>", self._on_space_down)
        self.root.bind("<KeyRelease-space>", self._on_space_up)

    def _grid_to_canvas(self, x: int, y: int) -> Tuple[int, int, int, int]:
        x0 = x * self.cell_size + self.view_offset_x
        y0 = y * self.cell_size + self.view_offset_y
        return int(x0), int(y0), int(x0 + self.cell_size), int(y0 + self.cell_size)

    def _canvas_to_grid(self, event) -> Tuple[int, int]:
        x = (event.x - self.view_offset_x) / self.cell_size
        y = (event.y - self.view_offset_y) / self.cell_size
        return int(x), int(y)

    def _draw_all(self):
        self.canvas.delete("all")
        for y in range(self.state.height):
            for x in range(self.state.width):
                x0, y0, x1, y1 = self._grid_to_canvas(x, y)
                color = CELL_COLORS.get(self.state.cells[y][x], "#ffffff")
                self.canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline=GRID_LINE)
        for gx, gy in self.state.goals:
            x0, y0, x1, y1 = self._grid_to_canvas(gx, gy)
            self.canvas.create_rectangle(x0 + 6, y0 + 6, x1 - 6, y1 - 6, fill=GOAL_COLOR, outline="")
        if self.sim_mode:
            self._refresh_sim_goals()
        for cx, cy in self.cargo_goals:
            x0, y0, x1, y1 = self._grid_to_canvas(cx, cy)
            self.canvas.create_polygon(
                x0 + self.cell_size // 2,
                y0 + 4,
                x1 - 4,
                y1 - 4,
                x0 + 4,
                y1 - 4,
                fill=CARGO_COLOR,
                outline="",
                tags="cargo_goal",
            )
        agents_to_draw = self.state.agents
        if self.sim_mode:
            agents_to_draw = self._sim_positions()
        for ax, ay in agents_to_draw:
            x0, y0, x1, y1 = self._grid_to_canvas(ax, ay)
            self.canvas.create_oval(
                x0 + 4,
                y0 + 4,
                x1 - 4,
                y1 - 4,
                fill=AGENT_COLOR,
                outline="",
                tags="agent_static",
            )
        current_t = self.sim_timestep_var.get() if self.sim_mode else 0
        self.timestep_label.config(text=f"T={current_t}")
        if self.sim_mode and self.sim_tasks:
            delivered = sum(
                1 for t in self.sim_tasks
                if t.get("delivered_time") is not None and t["delivered_time"] <= current_t
            )
            rate = delivered / current_t if current_t > 0 else 0.0
            self.throughput_label.config(text=f"Delivered: {delivered} | Throughput: {rate:.2f}/t")
        else:
            self.throughput_label.config(text="")

    def _on_zoom(self, event):
        old_size = self.cell_size
        delta = 0
        if hasattr(event, "delta") and event.delta != 0:
            delta = 1 if event.delta > 0 else -1
        elif hasattr(event, "num"):
            if event.num == 4:
                delta = 1
            elif event.num == 5:
                delta = -1
        if delta == 0:
            return
        step = max(1, int(self.cell_size * 0.12))
        new_size = self.cell_size + delta * step
        new_size = max(self.min_cell_size, min(self.max_cell_size, new_size))
        if new_size == self.cell_size:
            return
        if old_size > 0:
            grid_x = (event.x - self.view_offset_x) / old_size
            grid_y = (event.y - self.view_offset_y) / old_size
        else:
            grid_x = 0.0
            grid_y = 0.0
        self.cell_size = new_size
        self.view_offset_x = event.x - grid_x * new_size
        self.view_offset_y = event.y - grid_y * new_size
        if hasattr(self, "zoom_scale"):
            self.zoom_scale.set(new_size)
        self._draw_all()

    def _on_zoom_scale(self, value):
        try:
            new_size = int(float(value))
        except ValueError:
            return
        new_size = max(self.min_cell_size, min(self.max_cell_size, new_size))
        if new_size == self.cell_size:
            return
        cx = self.canvas.winfo_width() / 2
        cy = self.canvas.winfo_height() / 2
        old_size = self.cell_size
        grid_x = (cx - self.view_offset_x) / old_size if old_size > 0 else 0.0
        grid_y = (cy - self.view_offset_y) / old_size if old_size > 0 else 0.0
        self.cell_size = new_size
        self.view_offset_x = cx - grid_x * new_size
        self.view_offset_y = cy - grid_y * new_size
        self._draw_all()

    def _on_click(self, event):
        if self.tool.get() == "none" or self._space_down:
            self._panning = True
            self._pan_last = (event.x, event.y)
            return
        x, y = self._canvas_to_grid(event)
        if x < 0 or y < 0 or x >= self.state.width or y >= self.state.height:
            return
        if self._apply_tool(x, y):
            self._draw_all()

    def _on_drag(self, event):
        if self._panning:
            dx = event.x - self._pan_last[0]
            dy = event.y - self._pan_last[1]
            self.view_offset_x += dx
            self.view_offset_y += dy
            self._pan_last = (event.x, event.y)
            self._draw_all()
            return
        x, y = self._canvas_to_grid(event)
        if x < 0 or y < 0 or x >= self.state.width or y >= self.state.height:
            return
        if self._apply_tool(x, y, drag=True):
            self._draw_all()

    def _on_hover(self, event):
        if self._panning:
            return
        x, y = self._canvas_to_grid(event)
        if self.sim_mode:
            for rid in sorted(self.sim_trajectories.keys()):
                traj = self.sim_trajectories[rid]
                if not traj:
                    continue
                idx = min(self.sim_timestep_var.get(), len(traj) - 1)
                if traj[idx] == (x, y):
                    self.status_label.config(text=f"Agent {rid}")
                    return
            self.status_label.config(text="Hover agent to see state")
            return
        if (x, y) in self.state.agents:
            idx = self.state.agents.index((x, y))
            self.status_label.config(text=f"Agent {idx + 1}")
        else:
            self.status_label.config(text="Hover agent to see state")

    def _on_left_up(self, _event):
        self._panning = False

    def _on_space_down(self, _event):
        self._space_down = True

    def _on_space_up(self, _event):
        self._space_down = False

    def _apply_tool(self, x: int, y: int, drag: bool = False) -> bool:
        tool = self.tool.get()
        if tool == "none":
            return False
        if tool in ("wall", "shelf"):
            self.state.cells[y][x] = 1 if tool == "wall" else 2
            return True
        if tool == "erase":
            self.state.cells[y][x] = 0
            if (x, y) in self.state.agents:
                self.state.agents.remove((x, y))
            if (x, y) in self.state.goals:
                self.state.goals.remove((x, y))
            return True
        if tool == "agent":
            if drag:
                return False
            if (x, y) not in self.state.agents and self.state.cells[y][x] == 0:
                self.state.agents.append((x, y))
                return True
            return False
        if tool == "goal":
            if (x, y) not in self.state.goals and self.state.cells[y][x] == 0:
                self.state.goals.append((x, y))
                return True
            return False
        return False

    def _load_map_list(self):
        self.map_list.delete(0, tk.END)
        for name in list_maps(self.maps_dir):
            self.map_list.insert(tk.END, name)

    def _on_select_map(self, _event):
        selection = self.map_list.curselection()
        if not selection:
            return
        name = self.map_list.get(selection[0])
        data = load_map(self.maps_dir, name)
        self.selected_map = name
        self._load_state_from_data(data)
        self._draw_all()

    def _new_map(self):
        width = simpledialog.askinteger("Width", "Map width", initialvalue=12, minvalue=2, maxvalue=64)
        height = simpledialog.askinteger("Height", "Map height", initialvalue=12, minvalue=2, maxvalue=64)
        if not width or not height:
            return
        self.state = empty_state(width, height)
        self.cargo_goals = []
        self.sim_mode = False
        self.sim_tasks = []
        self.sim_trajectories = {}
        self.sim_scale.config(state="disabled", to=0)
        self.sim_timestep_var.set(0)
        self.selected_map = None
        self._draw_all()

    def _load_state_from_data(self, data: Dict):
        width = data.get("width", 10)
        height = data.get("height", 10)
        cells = data.get("cells")
        if not cells:
            cells = [[0 for _ in range(width)] for _ in range(height)]
        agents = [tuple(p) for p in data.get("agents", [])]
        goals = [tuple(p) for p in data.get("goals", [])]
        self.state = UIState(width=width, height=height, cells=cells, agents=agents, goals=goals)
        self.cargo_goals = []

    def _state_to_data(self) -> Dict:
        return {
            "width": self.state.width,
            "height": self.state.height,
            "cells": self.state.cells,
            "agents": self.state.agents,
            "goals": self.state.goals,
        }

    def _save_map(self):
        if not self.selected_map:
            return self._save_as()
        save_map(self.maps_dir, self.selected_map, self._state_to_data())
        self._load_map_list()

    def _save_as(self):
        name = simpledialog.askstring("Save As", "Map name")
        if not name:
            return
        self.selected_map = save_map(self.maps_dir, name, self._state_to_data())
        self._load_map_list()

    def _load_sim(self):
        path = filedialog.askopenfilename(title="Load simulation output", filetypes=[("JSON", "*.json")])
        if not path:
            return
        data = load_sim_output(path)
        map_path = data.get("map")
        if not map_path:
            raise ValueError("Simulation output missing map path")
        map_abs = os.path.abspath(map_path)
        map_dir = os.path.dirname(map_abs)
        map_name = os.path.basename(map_abs)
        map_data = load_map(map_dir, map_name)
        self._load_state_from_data(map_data)
        self.sim_tasks = data.get("tasks", [])
        agents_data = data.get("agents", {})
        trajectories = {}
        for rid, info in agents_data.items():
            traj = [tuple(p) for p in info.get("trajectory", [])]
            trajectories[int(rid)] = traj
        self.sim_trajectories = trajectories
        self.sim_max_timestep = int(data.get("max_timestep", 0))
        self.sim_timestep_var.set(0)
        self.sim_scale.config(state="normal", to=self.sim_max_timestep)
        self.sim_mode = True
        self._draw_all()

    def _refresh_sim_goals(self):
        visible = []
        current = self.sim_timestep_var.get()
        for task in self.sim_tasks:
            spawn = task.get("spawn_time", 0)
            picked = task.get("picked_time")
            if spawn <= current and picked is None:
                visible.append(tuple(task.get("pos")))
            elif spawn <= current and picked is not None and picked > current:
                visible.append(tuple(task.get("pos")))
        self.cargo_goals = visible

    def _sim_positions(self) -> List[Tuple[int, int]]:
        positions = []
        current = self.sim_timestep_var.get()
        for rid in sorted(self.sim_trajectories.keys()):
            traj = self.sim_trajectories[rid]
            if not traj:
                continue
            idx = min(current, len(traj) - 1)
            positions.append(traj[idx])
        return positions

    def _on_sim_scale(self, _value):
        if not self.sim_mode:
            return
        self._draw_all()

    def _play_sim(self):
        if not self.sim_mode:
            return
        self.sim_running = True
        self._tick_sim()

    def _pause_sim(self):
        self.sim_running = False
        if self.sim_after_id is not None:
            self.canvas.after_cancel(self.sim_after_id)
            self.sim_after_id = None

    def _step_sim(self):
        if not self.sim_mode:
            return
        self.sim_running = False
        current = self.sim_timestep_var.get()
        if current >= self.sim_max_timestep:
            return
        self.sim_timestep_var.set(current + 1)
        self._draw_all()

    def _back_sim(self):
        if not self.sim_mode:
            return
        self.sim_running = False
        current = self.sim_timestep_var.get()
        if current <= 0:
            return
        self.sim_timestep_var.set(current - 1)
        self._draw_all()

    def _tick_sim(self):
        if not self.sim_running:
            return
        current = self.sim_timestep_var.get()
        if current >= self.sim_max_timestep:
            self.sim_running = False
            return
        self.sim_timestep_var.set(current + 1)
        self._draw_all()
        self.sim_after_id = self.canvas.after(200, self._tick_sim)


def run_editor(maps_dir: str) -> None:
    root = tk.Tk()
    editor = MapEditor(root, maps_dir)
    root.mainloop()


if __name__ == "__main__":
    run_editor(os.path.join(os.path.dirname(__file__), "..", "..", "..", "maps"))
