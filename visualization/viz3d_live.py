"""3D live visualization of search progress using PyVista."""

import os
import time
from datetime import datetime

import numpy as np
import pyvista as pv


def visualize_search_3d_live(environment, search_iter, start, goal, *, save_dir="outputs", step_delay=0.02):
    """
    Render A* search progress in real-time 3D using PyVista.

    Args:
        environment: GridEnvironment instance
        search_iter: Iterator yielding search events from AStarStepperPlanner.iterate()
        start: Start position (row, col)
        goal: Goal position (row, col)
        save_dir: Directory for output files
        step_delay: Delay between visualization updates in seconds

    Returns:
        Tuple of (path_coords, metrics) from the final path event
    """
    os.makedirs(save_dir, exist_ok=True)

    grid = np.array(environment.grid)
    rows, cols = grid.shape

    # Set up PyVista plotter
    pv.global_theme.background = "#1a1a2e"
    plotter = pv.Plotter(title="A* Search Visualization")
    plotter.set_background("#1a1a2e")

    # Create ground plane
    ground = pv.Plane(
        center=(cols / 2 - 0.5, rows / 2 - 0.5, -0.05),
        direction=(0, 0, 1),
        i_size=cols + 1,
        j_size=rows + 1
    )
    plotter.add_mesh(ground, color="#16213e", opacity=0.8)

    # Draw grid lines on ground
    for r in range(rows + 1):
        line = pv.Line((0 - 0.5, r - 0.5, 0), (cols - 0.5, r - 0.5, 0))
        plotter.add_mesh(line, color="#2d3a4f", line_width=1, opacity=0.5)
    for c in range(cols + 1):
        line = pv.Line((c - 0.5, 0 - 0.5, 0), (c - 0.5, rows - 0.5, 0))
        plotter.add_mesh(line, color="#2d3a4f", line_width=1, opacity=0.5)

    # Draw obstacles as extruded cubes
    obstacle_height = 0.8
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 1:
                cube = pv.Cube(
                    center=(c, r, obstacle_height / 2),
                    x_length=0.9, y_length=0.9, z_length=obstacle_height
                )
                plotter.add_mesh(cube, color="#8b0000", opacity=1.0, show_edges=True, edge_color="#ff4444")

    # Start marker - cyan cylinder
    start_marker = pv.Cylinder(
        center=(start[1], start[0], 0.1),
        direction=(0, 0, 1),
        radius=0.35,
        height=0.2
    )
    plotter.add_mesh(start_marker, color="#00ffff", opacity=1.0)
    plotter.add_point_labels(
        [(start[1], start[0], 0.5)],
        ["START"],
        font_size=12,
        text_color="cyan",
        shape_color="#1a1a2e",
        shape_opacity=0.7
    )

    # Goal marker - gold star (approximated with cone)
    goal_marker = pv.Cone(
        center=(goal[1], goal[0], 0.25),
        direction=(0, 0, 1),
        height=0.4,
        radius=0.35
    )
    plotter.add_mesh(goal_marker, color="#ffd700", opacity=1.0)
    plotter.add_point_labels(
        [(goal[1], goal[0], 0.7)],
        ["GOAL"],
        font_size=12,
        text_color="gold",
        shape_color="#1a1a2e",
        shape_opacity=0.7
    )

    # Robot sphere (starts at start position)
    robot_sphere = pv.Sphere(center=(start[1], start[0], 0.3), radius=0.25)
    robot_actor = plotter.add_mesh(robot_sphere, color="#ff00ff", opacity=1.0, name="robot")

    # Tracking structures for dynamic updates
    explored_actors = {}
    frontier_actors = {}

    # Set camera
    plotter.camera.position = (cols / 2, -rows, rows * 1.5)
    plotter.camera.focal_point = (cols / 2 - 0.5, rows / 2 - 0.5, 0)
    plotter.camera.up = (0, 0, 1)

    # Add text overlay for metrics
    metrics_text = plotter.add_text(
        "Nodes: 0 | Frontier: 0",
        position="upper_left",
        font_size=10,
        color="white"
    )

    plotter.show(interactive_update=True, auto_close=False)

    # Process search events
    path_coords = []
    final_metrics = None
    nodes_expanded = 0
    frontier_count = 0

    for event in search_iter:
        event_type = event["type"]

        if event_type == "expand":
            pos = event["pos"]
            nodes_expanded += 1

            # Remove from frontier visualization if present
            if pos in frontier_actors:
                plotter.remove_actor(frontier_actors[pos])
                del frontier_actors[pos]
                frontier_count -= 1

            # Add to explored (semi-transparent green cube)
            if pos != start and pos != goal and pos not in explored_actors:
                cube = pv.Cube(
                    center=(pos[1], pos[0], 0.15),
                    x_length=0.7, y_length=0.7, z_length=0.3
                )
                actor = plotter.add_mesh(cube, color="#00ff88", opacity=0.4, name=f"exp_{pos}")
                explored_actors[pos] = actor

            # Move robot to current expansion position
            plotter.remove_actor("robot")
            robot_sphere = pv.Sphere(center=(pos[1], pos[0], 0.3), radius=0.25)
            plotter.add_mesh(robot_sphere, color="#ff00ff", opacity=1.0, name="robot")

        elif event_type == "frontier":
            pos = event["pos"]

            # Add to frontier visualization (orange cube)
            if pos != start and pos != goal and pos not in explored_actors and pos not in frontier_actors:
                cube = pv.Cube(
                    center=(pos[1], pos[0], 0.1),
                    x_length=0.5, y_length=0.5, z_length=0.2
                )
                actor = plotter.add_mesh(cube, color="#ff8800", opacity=0.6, name=f"front_{pos}")
                frontier_actors[pos] = actor
                frontier_count += 1

        elif event_type == "path":
            path_coords = event["path"]
            final_metrics = event["metrics"]

            # Clear frontier visualization
            for actor in frontier_actors.values():
                plotter.remove_actor(actor)
            frontier_actors.clear()

            # Draw final path as connected tubes
            if len(path_coords) > 1:
                for i in range(len(path_coords) - 1):
                    p1 = path_coords[i]
                    p2 = path_coords[i + 1]
                    tube = pv.Tube(
                        pointa=(p1[1], p1[0], 0.25),
                        pointb=(p2[1], p2[0], 0.25),
                        radius=0.1
                    )
                    plotter.add_mesh(tube, color="#00ffaa", opacity=1.0)

            # Animate robot along final path
            plotter.remove_actor("robot")
            for coord in path_coords:
                robot_sphere = pv.Sphere(center=(coord[1], coord[0], 0.3), radius=0.25)
                plotter.remove_actor("robot")
                plotter.add_mesh(robot_sphere, color="#ff00ff", opacity=1.0, name="robot")
                plotter.update()
                time.sleep(step_delay * 3)

            break

        # Update metrics text
        plotter.remove_actor(metrics_text)
        metrics_text = plotter.add_text(
            f"Expanded: {nodes_expanded} | Frontier: {frontier_count}",
            position="upper_left",
            font_size=10,
            color="white"
        )

        plotter.update()
        time.sleep(step_delay)

    # Final metrics display
    if final_metrics:
        plotter.remove_actor(metrics_text)
        plotter.add_text(
            f"Complete! Path: {final_metrics.path_length} | Expanded: {final_metrics.nodes_expanded} | Time: {final_metrics.runtime:.4f}s",
            position="upper_left",
            font_size=10,
            color="#00ffaa"
        )
        plotter.update()

    # Save screenshot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    screenshot_path = os.path.join(save_dir, f"search_3d_{timestamp}.png")
    plotter.screenshot(screenshot_path)
    print(f"Saved 3D screenshot: {screenshot_path}")

    # Keep window open for interaction
    plotter.show(interactive=True)

    return path_coords, final_metrics
