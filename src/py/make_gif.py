"""Generate a GIF animation from simulation output JSON."""
import argparse
import json
import math
import os
import sys
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
from PIL import Image
import io

# Direction constants
DIR_EAST = 0
DIR_WEST = 1
DIR_SOUTH = 2
DIR_NORTH = 3

# Direction arrows (dx, dy in screen coords where y points down)
DIR_ARROW = {
    DIR_EAST: (0.3, 0),
    DIR_WEST: (-0.3, 0),
    DIR_SOUTH: (0, 0.3),
    DIR_NORTH: (0, -0.3),
}

AGENT_COLORS = [
    "#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
    "#42d4f4", "#f032e6", "#bfef45", "#fabed4", "#469990",
    "#dcbeff", "#9A6324", "#800000", "#aaffc3", "#808000",
    "#000075", "#a9a9a9",
]


def load_sim(path: str) -> Dict:
    with open(path, "r") as f:
        return json.load(f)


def load_map_data(map_path: str) -> Dict:
    with open(map_path, "r") as f:
        return json.load(f)


def render_frame(
    cells: List[List[int]],
    goals: List[Tuple[int, int]],
    positions: Dict[int, Tuple[int, int]],
    facings: Dict[int, int],
    timestep: int,
    title: str,
    cell_size: float = 0.5,
    throughput_text: str = "",
) -> Image.Image:
    height = len(cells)
    width = len(cells[0]) if height > 0 else 0

    fig_w = width * cell_size + 1.0
    fig_h = height * cell_size + 1.5
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.set_xlim(-0.5, width - 0.5)
    ax.set_ylim(height - 0.5, -0.5)
    ax.set_aspect("equal")
    ax.set_title(f"{title}  t={timestep}", fontsize=10, fontweight="bold")
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    # Draw grid
    for y in range(height):
        for x in range(width):
            cell = cells[y][x]
            if cell == 1:
                ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="#333333"))
            elif cell == 2:
                ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="#ffe0b2"))
            else:
                ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="#f5f5f5", ec="#ddd", lw=0.3))

    # Draw goals
    goal_set = set(tuple(g) for g in goals)
    for gx, gy in goal_set:
        ax.add_patch(plt.Rectangle((gx - 0.4, gy - 0.4), 0.8, 0.8, color="#a5d6a7", alpha=0.7, zorder=1))

    # Draw agents
    for i, (rid, (px, py)) in enumerate(sorted(positions.items())):
        color = AGENT_COLORS[i % len(AGENT_COLORS)]
        circle = plt.Circle((px, py), 0.35, color=color, ec="black", lw=1.0, zorder=3)
        ax.add_patch(circle)
        ax.text(px, py, str(rid), ha="center", va="center", fontsize=7, fontweight="bold", color="white", zorder=4)

        # Draw facing arrow if available
        if rid in facings:
            d = facings[rid]
            if d in DIR_ARROW:
                dx, dy = DIR_ARROW[d]
                ax.annotate(
                    "",
                    xy=(px + dx, py + dy),
                    xytext=(px, py),
                    arrowprops=dict(arrowstyle="->", color="white", lw=1.5),
                    zorder=5,
                )

    if throughput_text:
        fig.text(0.05, 0.01, throughput_text, fontsize=8, color="#555555", va="bottom")

    plt.tight_layout()
    fig.subplots_adjust(bottom=0.08 if throughput_text else 0.05)

    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).copy()


def make_gif(
    sim_path: str,
    output_path: str,
    max_frames: int = 60,
    frame_duration: int = 300,
    title: str = "",
):
    sim = load_sim(sim_path)
    raw_map = sim.get("map", "")
    map_path = raw_map
    if not os.path.isabs(map_path):
        # Try relative to sim file
        map_path = os.path.join(os.path.dirname(os.path.abspath(sim_path)), raw_map)
    if not os.path.exists(map_path):
        # Try relative to this script's dir (src/py — the usual working dir)
        map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), raw_map)
    if not os.path.exists(map_path):
        # Try relative to project root
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        map_path = os.path.join(root, "maps", os.path.basename(raw_map))
    map_data = load_map_data(map_path)
    cells = map_data["cells"]
    goals = [tuple(g) for g in map_data.get("goals", [])]

    agents_data = sim["agents"]
    trajectories: Dict[int, List[Tuple[int, int]]] = {}
    for rid_str, adata in agents_data.items():
        rid = int(rid_str)
        traj = [tuple(p) for p in adata["trajectory"]]
        trajectories[rid] = traj

    max_len = max(len(t) for t in trajectories.values()) if trajectories else 0

    # Sample frames if too many
    if max_len > max_frames:
        step = max(1, max_len // max_frames)
        frame_indices = list(range(0, max_len, step))
        if frame_indices[-1] != max_len - 1:
            frame_indices.append(max_len - 1)
    else:
        frame_indices = list(range(max_len))

    tasks = sim.get("tasks", [])

    height = len(cells)
    width = len(cells[0]) if height > 0 else 0
    cell_size = 0.5 if width <= 20 else 0.25

    frames = []
    for t_idx in frame_indices:
        positions = {}
        facings = {}
        for rid, traj in trajectories.items():
            idx = min(t_idx, len(traj) - 1)
            positions[rid] = traj[idx]
            # Infer facing from movement
            if idx > 0:
                prev = traj[idx - 1]
                curr = traj[idx]
                dx = curr[0] - prev[0]
                dy = curr[1] - prev[1]
                if dx == 1:
                    facings[rid] = DIR_EAST
                elif dx == -1:
                    facings[rid] = DIR_WEST
                elif dy == 1:
                    facings[rid] = DIR_SOUTH
                elif dy == -1:
                    facings[rid] = DIR_NORTH
                elif rid in facings:
                    pass  # keep previous
                else:
                    facings[rid] = DIR_EAST
            else:
                facings[rid] = DIR_EAST

        delivered = sum(
            1 for t in tasks
            if t.get("delivered_time") is not None and t["delivered_time"] <= t_idx
        )
        rate = delivered / t_idx if t_idx > 0 else 0.0
        tp_text = f"Delivered: {delivered} | Throughput: {rate:.2f}/t" if tasks else ""

        frame = render_frame(cells, goals, positions, facings, t_idx, title, cell_size, tp_text)
        frames.append(frame)
        if len(frames) % 10 == 0:
            print(f"  rendered {len(frames)}/{len(frame_indices)} frames")

    if not frames:
        print("No frames to render!")
        return

    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=frame_duration,
        loop=0,
    )
    print(f"Saved {len(frames)} frames to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Generate GIF from simulation output")
    parser.add_argument("--sim", required=True, help="Path to simulation output JSON")
    parser.add_argument("--output", required=True, help="Output GIF path")
    parser.add_argument("--max_frames", type=int, default=60, help="Max frames in GIF")
    parser.add_argument("--duration", type=int, default=300, help="Frame duration in ms")
    parser.add_argument("--title", default="", help="Title for the animation")
    args = parser.parse_args()
    make_gif(args.sim, args.output, args.max_frames, args.duration, args.title)


if __name__ == "__main__":
    main()
