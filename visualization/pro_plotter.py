"""Professional visualization for navigation results."""

import os
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import numpy as np


def visualize_path_pro(environment, path, metrics, *, title=None, save_dir="outputs", animate=False):
    """
    Generate a high-quality 2D visualization of the navigation path.

    Args:
        environment: GridEnvironment instance
        path: List of (action, state) tuples from planner
        metrics: PlannerMetrics instance
        title: Optional custom title for the plot
        save_dir: Directory to save output files
        animate: If True, generate an animated GIF of the path
    """
    if not path:
        print("No valid path found to the goal.")
        return

    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Extract path coordinates
    path_coords = [state[1] for state in path]
    start = path_coords[0]
    goal = path_coords[-1]

    # Set up dark theme
    plt.style.use("dark_background")

    fig, ax = plt.subplots(figsize=(10, 8))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#16213e")

    grid = np.array(environment.grid)
    rows, cols = grid.shape

    # Draw grid cells
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 1:
                # Obstacle - dark red/maroon
                rect = mpatches.Rectangle(
                    (c - 0.5, r - 0.5), 1, 1,
                    facecolor="#8b0000", edgecolor="#ff4444", linewidth=1.5
                )
            else:
                # Open cell - dark blue
                rect = mpatches.Rectangle(
                    (c - 0.5, r - 0.5), 1, 1,
                    facecolor="#1a1a2e", edgecolor="#2d3a4f", linewidth=0.5
                )
            ax.add_patch(rect)

    # Draw subtle grid lines
    for r in range(rows + 1):
        ax.axhline(y=r - 0.5, color="#2d3a4f", linewidth=0.3, alpha=0.5)
    for c in range(cols + 1):
        ax.axvline(x=c - 0.5, color="#2d3a4f", linewidth=0.3, alpha=0.5)

    # Draw path with directional arrows
    if len(path_coords) > 1:
        for i in range(len(path_coords) - 1):
            r1, c1 = path_coords[i]
            r2, c2 = path_coords[i + 1]

            # Draw arrow
            ax.annotate(
                "", xy=(c2, r2), xytext=(c1, r1),
                arrowprops=dict(
                    arrowstyle="->",
                    color="#00ffaa",
                    lw=2.5,
                    mutation_scale=15
                )
            )

        # Draw path line underneath for better visibility
        path_r = [coord[0] for coord in path_coords]
        path_c = [coord[1] for coord in path_coords]
        ax.plot(path_c, path_r, color="#00ffaa", linewidth=3, alpha=0.3, zorder=1)

    # Start marker - cyan diamond
    ax.plot(start[1], start[0], marker="D", color="#00ffff", markersize=18,
            markeredgecolor="white", markeredgewidth=2, zorder=10)
    ax.annotate("START", (start[1], start[0]), textcoords="offset points",
                xytext=(15, 15), fontsize=11, fontweight="bold", color="#00ffff",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#1a1a2e", edgecolor="#00ffff"))

    # Goal marker - gold star
    ax.plot(goal[1], goal[0], marker="*", color="#ffd700", markersize=25,
            markeredgecolor="white", markeredgewidth=1.5, zorder=10)
    ax.annotate("GOAL", (goal[1], goal[0]), textcoords="offset points",
                xytext=(15, -20), fontsize=11, fontweight="bold", color="#ffd700",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#1a1a2e", edgecolor="#ffd700"))

    # Metrics overlay box
    metrics_text = (
        f"Planner: {metrics.planner_name if hasattr(metrics, 'planner_name') else 'N/A'}\n"
        f"Path Length: {metrics.path_length}\n"
        f"Nodes Expanded: {metrics.nodes_expanded}\n"
        f"Runtime: {metrics.runtime:.4f}s"
    )
    props = dict(boxstyle="round,pad=0.5", facecolor="#1a1a2e", edgecolor="#00ffaa", alpha=0.9)
    ax.text(0.02, 0.98, metrics_text, transform=ax.transAxes, fontsize=10,
            verticalalignment="top", fontfamily="monospace", color="#ffffff", bbox=props)

    # Title
    plot_title = title if title else "Robot Navigation Path"
    ax.set_title(plot_title, fontsize=16, fontweight="bold", color="#ffffff", pad=15)

    # Axis labels and limits
    ax.set_xlabel("Column", fontsize=12, color="#aaaaaa")
    ax.set_ylabel("Row", fontsize=12, color="#aaaaaa")
    ax.set_xlim(-0.6, cols - 0.4)
    ax.set_ylim(rows - 0.4, -0.6)
    ax.set_aspect("equal")
    ax.tick_params(colors="#aaaaaa")

    plt.tight_layout()

    # Save PNG
    png_path = os.path.join(save_dir, f"navigation_{timestamp}.png")
    fig.savefig(png_path, dpi=150, facecolor=fig.get_facecolor(), edgecolor="none")
    print(f"Saved PNG: {png_path}")

    # Generate animated GIF if requested
    if animate and len(path_coords) > 1:
        gif_path = os.path.join(save_dir, f"navigation_{timestamp}.gif")
        _generate_animation(environment, path_coords, metrics, title, gif_path)
        print(f"Saved GIF: {gif_path}")

    plt.show()
    plt.close(fig)


def _generate_animation(environment, path_coords, metrics, title, gif_path):
    """Generate an animated GIF of the path traversal."""
    plt.style.use("dark_background")

    fig, ax = plt.subplots(figsize=(10, 8))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#16213e")

    grid = np.array(environment.grid)
    rows, cols = grid.shape

    start = path_coords[0]
    goal = path_coords[-1]

    def draw_base():
        ax.clear()
        ax.set_facecolor("#16213e")

        # Draw grid cells
        for r in range(rows):
            for c in range(cols):
                if grid[r, c] == 1:
                    rect = mpatches.Rectangle(
                        (c - 0.5, r - 0.5), 1, 1,
                        facecolor="#8b0000", edgecolor="#ff4444", linewidth=1.5
                    )
                else:
                    rect = mpatches.Rectangle(
                        (c - 0.5, r - 0.5), 1, 1,
                        facecolor="#1a1a2e", edgecolor="#2d3a4f", linewidth=0.5
                    )
                ax.add_patch(rect)

        # Subtle grid lines
        for r in range(rows + 1):
            ax.axhline(y=r - 0.5, color="#2d3a4f", linewidth=0.3, alpha=0.5)
        for c in range(cols + 1):
            ax.axvline(x=c - 0.5, color="#2d3a4f", linewidth=0.3, alpha=0.5)

        # Start marker
        ax.plot(start[1], start[0], marker="D", color="#00ffff", markersize=18,
                markeredgecolor="white", markeredgewidth=2, zorder=10)
        ax.annotate("START", (start[1], start[0]), textcoords="offset points",
                    xytext=(15, 15), fontsize=11, fontweight="bold", color="#00ffff",
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="#1a1a2e", edgecolor="#00ffff"))

        # Goal marker
        ax.plot(goal[1], goal[0], marker="*", color="#ffd700", markersize=25,
                markeredgecolor="white", markeredgewidth=1.5, zorder=10)
        ax.annotate("GOAL", (goal[1], goal[0]), textcoords="offset points",
                    xytext=(15, -20), fontsize=11, fontweight="bold", color="#ffd700",
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="#1a1a2e", edgecolor="#ffd700"))

        # Metrics box
        metrics_text = (
            f"Planner: {metrics.planner_name if hasattr(metrics, 'planner_name') else 'N/A'}\n"
            f"Path Length: {metrics.path_length}\n"
            f"Nodes Expanded: {metrics.nodes_expanded}\n"
            f"Runtime: {metrics.runtime:.4f}s"
        )
        props = dict(boxstyle="round,pad=0.5", facecolor="#1a1a2e", edgecolor="#00ffaa", alpha=0.9)
        ax.text(0.02, 0.98, metrics_text, transform=ax.transAxes, fontsize=10,
                verticalalignment="top", fontfamily="monospace", color="#ffffff", bbox=props)

        # Title
        plot_title = title if title else "Robot Navigation Path"
        ax.set_title(plot_title, fontsize=16, fontweight="bold", color="#ffffff", pad=15)

        ax.set_xlabel("Column", fontsize=12, color="#aaaaaa")
        ax.set_ylabel("Row", fontsize=12, color="#aaaaaa")
        ax.set_xlim(-0.6, cols - 0.4)
        ax.set_ylim(rows - 0.4, -0.6)
        ax.set_aspect("equal")
        ax.tick_params(colors="#aaaaaa")

    def animate(frame):
        draw_base()

        # Draw path up to current frame
        if frame > 0:
            for i in range(min(frame, len(path_coords) - 1)):
                r1, c1 = path_coords[i]
                r2, c2 = path_coords[i + 1]
                ax.annotate(
                    "", xy=(c2, r2), xytext=(c1, r1),
                    arrowprops=dict(arrowstyle="->", color="#00ffaa", lw=2.5, mutation_scale=15)
                )

        # Draw robot position
        if frame < len(path_coords):
            curr = path_coords[frame]
            ax.plot(curr[1], curr[0], marker="o", color="#ff00ff", markersize=14,
                    markeredgecolor="white", markeredgewidth=2, zorder=15)

    # Extra frames at end to hold final position
    total_frames = len(path_coords) + 5
    anim = FuncAnimation(fig, animate, frames=total_frames, interval=300, repeat=True)
    anim.save(gif_path, writer="pillow", fps=3, dpi=100)
    plt.close(fig)
