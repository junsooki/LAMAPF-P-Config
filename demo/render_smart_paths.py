#!/usr/bin/env python3
"""Render SMART-format paths on a Moving AI map as a GIF."""
import sys, os, re
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "py"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
import io

from smart_io import load_movingai_map

COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#42d4f4", "#f032e6", "#bfef45"]


def parse_smart_paths(path):
    """Parse SMART path file into {agent_id: [(x,y), ...]}"""
    paths = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = re.match(r"Agent (\d+):(.*)", line)
            if not m:
                continue
            aid = int(m.group(1))
            waypoints = re.findall(r"\(([^)]+)\)", m.group(2))
            coords = []
            for wp in waypoints:
                parts = wp.split(",")
                coords.append((int(parts[0]), int(parts[1])))
            paths[aid] = coords
    return paths


def render_frame(grid, paths, goals, t, T):
    H = len(grid)
    W = len(grid[0])
    cs = 0.6
    fig, ax = plt.subplots(figsize=(W * cs + 0.8, H * cs + 1.2))
    ax.set_xlim(-0.5, W - 0.5)
    ax.set_ylim(H - 0.5, -0.5)
    ax.set_aspect("equal")
    ax.set_title(f"LAMAPF-P → SMART   t={t}/{T}", fontsize=11, fontweight="bold")
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    for y in range(H):
        for x in range(W):
            if grid[y][x] == 1:
                ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="#333"))
            else:
                ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="#f5f5f5", ec="#ddd", lw=0.3))

    for gx, gy in goals:
        ax.add_patch(plt.Rectangle((gx - 0.4, gy - 0.4), 0.8, 0.8, color="#a5d6a7", alpha=0.6, zorder=1))
        ax.text(gx, gy, "G", ha="center", va="center", fontsize=7, color="#2e7d32", zorder=2)

    for aid in sorted(paths):
        p = paths[aid]
        idx = min(t, len(p) - 1)
        x, y = p[idx]
        color = COLORS[aid % len(COLORS)]

        # Trail
        trail_start = max(0, idx - 3)
        for ti in range(trail_start, idx):
            tx, ty = p[ti]
            alpha = 0.15 + 0.15 * (ti - trail_start)
            ax.add_patch(plt.Circle((tx, ty), 0.2, color=color, alpha=alpha, zorder=2))

        # Agent
        ax.add_patch(plt.Circle((x, y), 0.35, color=color, ec="black", lw=1.2, zorder=5))
        ax.text(x, y, str(aid), ha="center", va="center", fontsize=8, fontweight="bold", color="white", zorder=6)

    plt.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=120, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).copy()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--paths", required=True)
    ap.add_argument("--output", default="demo.gif")
    ap.add_argument("--duration", type=int, default=500)
    args = ap.parse_args()

    grid = load_movingai_map(args.map)
    paths = parse_smart_paths(args.paths)

    T = max(len(p) - 1 for p in paths.values())
    goals = [p[-1] for p in paths.values()]

    frames = []
    for t in range(T + 1):
        frames.append(render_frame(grid, paths, goals, t, T))
    # Hold last frame
    frames.append(render_frame(grid, paths, goals, T, T))
    frames.append(render_frame(grid, paths, goals, T, T))

    frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=args.duration, loop=0)
    print(f"Saved {len(frames)} frames to {args.output}")


if __name__ == "__main__":
    main()
