"""3D live visualization of search progress using PyVista."""

import os
import time
import math
from datetime import datetime

import numpy as np
import pyvista as pv

# Robot model configuration
ROBOT_SCALE = 0.85  # Max dimension to fit within one grid cell
ROBOT_Z_PADDING = 0.02  # Small padding above ground
ROBOT_YAW_OFFSET = 0  # Additional yaw offset for forward direction alignment

# Model axis correction (applied once after loading to convert coordinate systems)
# Default: MODEL_ROT_X=90 converts Y-up models to Z-up
MODEL_ROT_X = 90
MODEL_ROT_Y = 0
MODEL_ROT_Z = 0

# Robot material (metallic appearance)
ROBOT_COLOR = "#4a90d9"  # Steel blue
ROBOT_METALLIC = 0.7
ROBOT_ROUGHNESS = 0.3

# Paths to robot models (relative to project root)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_SCRIPT_DIR)
ROBOT_MODEL_PRIMARY = os.path.join(_PROJECT_ROOT, "assets", "models", "robot_basic_pbr.glb")
ROBOT_MODEL_FALLBACK = os.path.join(_PROJECT_ROOT, "assets", "models", "robot_shaded.glb")

# Smooth movement configuration
INTERPOLATION_STEPS = 10  # Number of frames to interpolate between grid cells


def _extract_meshes_recursive(block, meshes):
    """Recursively extract all PolyData meshes from a MultiBlock structure."""
    if block is None:
        return
    if isinstance(block, pv.MultiBlock):
        for i in range(block.n_blocks):
            _extract_meshes_recursive(block.get_block(i), meshes)
    elif hasattr(block, 'n_points') and block.n_points > 0:
        meshes.append(block)


def _load_robot_model():
    """
    Load robot GLB model, apply axis correction, scaling, and ground alignment.

    The mesh is prepared once for efficient runtime transforms:
    - Axis rotation applied (MODEL_ROT_X/Y/Z) to convert Y-up to Z-up
    - Scaled so max dimension is ROBOT_SCALE
    - Centered on XY plane
    - Ground aligned (zmin = ROBOT_Z_PADDING)

    Returns:
        pv.PolyData: Prepared robot mesh, or None if loading fails.
    """
    model_path = None
    if os.path.exists(ROBOT_MODEL_PRIMARY):
        model_path = ROBOT_MODEL_PRIMARY
    elif os.path.exists(ROBOT_MODEL_FALLBACK):
        model_path = ROBOT_MODEL_FALLBACK

    if model_path is None:
        return None

    try:
        model = pv.read(model_path)

        # Handle MultiBlock (potentially nested) by extracting all meshes
        if isinstance(model, pv.MultiBlock):
            meshes = []
            _extract_meshes_recursive(model, meshes)
            if not meshes:
                return None
            model = meshes[0] if len(meshes) == 1 else meshes[0].merge(meshes[1:])

        if not hasattr(model, 'n_points') or model.n_points == 0:
            return None

        # Apply axis correction rotations (convert Y-up to Z-up typically)
        if MODEL_ROT_X != 0:
            model.rotate_x(MODEL_ROT_X, inplace=True)
        if MODEL_ROT_Y != 0:
            model.rotate_y(MODEL_ROT_Y, inplace=True)
        if MODEL_ROT_Z != 0:
            model.rotate_z(MODEL_ROT_Z, inplace=True)

        # Compute bounds after rotation
        bounds = model.bounds
        x_extent = bounds[1] - bounds[0]
        y_extent = bounds[3] - bounds[2]
        z_extent = bounds[5] - bounds[4]
        max_extent = max(x_extent, y_extent, z_extent)

        # Scale to fit within grid cell
        if max_extent > 0:
            scale_factor = ROBOT_SCALE / max_extent
            model.points *= scale_factor

        # Recompute bounds after scaling
        bounds = model.bounds

        # Center on XY plane
        x_center = (bounds[0] + bounds[1]) / 2
        y_center = (bounds[2] + bounds[3]) / 2
        model.points[:, 0] -= x_center
        model.points[:, 1] -= y_center

        # Ground alignment: translate so zmin = ROBOT_Z_PADDING
        z_min = model.bounds[4]
        model.points[:, 2] -= z_min
        model.points[:, 2] += ROBOT_Z_PADDING

        return model
    except Exception:
        return None


def _compute_yaw_from_direction(from_pos, to_pos):
    """
    Compute yaw angle (degrees) for robot to face direction of motion.

    Grid coordinates: (row, col) where row increases downward, col increases right.
    World coordinates: (x, y) where x=col, y=row.
    """
    if from_pos is None or to_pos is None:
        return 0

    dx = to_pos[1] - from_pos[1]  # col difference -> x direction
    dy = to_pos[0] - from_pos[0]  # row difference -> y direction

    if dx == 0 and dy == 0:
        return 0

    # atan2 gives angle from positive X axis, counterclockwise
    angle_rad = math.atan2(dy, dx)
    return math.degrees(angle_rad)


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

    Controls:
        Press 'f' to toggle follow-camera mode that tracks the robot.
    """
    os.makedirs(save_dir, exist_ok=True)

    grid = np.array(environment.grid)
    rows, cols = grid.shape

    # Load robot model once
    robot_mesh = _load_robot_model()
    use_robot_model = robot_mesh is not None

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

    # Goal marker - gold cone
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

    # Robot initialization - add mesh ONCE, then transform actor
    robot_actor = None
    robot_yaw = 0

    if use_robot_model:
        # Add robot mesh with metallic material
        try:
            robot_actor = plotter.add_mesh(
                robot_mesh,
                color=ROBOT_COLOR,
                pbr=True,
                metallic=ROBOT_METALLIC,
                roughness=ROBOT_ROUGHNESS,
                name="robot"
            )
        except Exception:
            # Fallback if PBR fails
            robot_actor = plotter.add_mesh(
                robot_mesh,
                color=ROBOT_COLOR,
                specular=0.8,
                specular_power=30,
                name="robot"
            )
        # Set initial position
        robot_actor.SetPosition(start[1], start[0], 0)
        robot_actor.SetOrientation(0, 0, ROBOT_YAW_OFFSET)
    else:
        # Fallback to sphere
        robot_sphere = pv.Sphere(center=(0, 0, 0.25), radius=0.25)
        robot_actor = plotter.add_mesh(
            robot_sphere,
            color=ROBOT_COLOR,
            specular=0.5,
            name="robot"
        )
        robot_actor.SetPosition(start[1], start[0], 0)

    # Tracking structures for dynamic updates
    explored_actors = {}
    frontier_actors = {}

    # Follow camera state
    follow_camera_enabled = False
    camera_offset = (2, -3, 4)  # Offset from robot position

    def toggle_follow_camera():
        nonlocal follow_camera_enabled
        follow_camera_enabled = not follow_camera_enabled

    plotter.add_key_event("f", toggle_follow_camera)

    # Set initial camera
    plotter.camera.position = (cols / 2, -rows, rows * 1.5)
    plotter.camera.focal_point = (cols / 2 - 0.5, rows / 2 - 0.5, 0)
    plotter.camera.up = (0, 0, 1)

    # Add text overlay for metrics
    metrics_text = plotter.add_text(
        "Nodes: 0 | Frontier: 0 | Press 'f' for follow-cam",
        position="upper_left",
        font_size=10,
        color="white"
    )

    def update_robot_transform(x, y, yaw):
        """Update robot actor position and orientation without recreating mesh."""
        robot_actor.SetPosition(x, y, 0)
        # Orientation is (pitch, roll, yaw) in VTK - we only rotate around Z
        robot_actor.SetOrientation(0, 0, yaw + ROBOT_YAW_OFFSET)

    def update_robot_position(new_pos, prev_pos=None, smooth=True):
        """Update robot position with optional smooth interpolation."""
        nonlocal robot_yaw

        # Compute yaw from movement direction
        if prev_pos is not None:
            robot_yaw = _compute_yaw_from_direction(prev_pos, new_pos)

        if smooth and prev_pos is not None:
            # Interpolate between positions
            for step in range(1, INTERPOLATION_STEPS + 1):
                t = step / INTERPOLATION_STEPS
                interp_x = prev_pos[1] + t * (new_pos[1] - prev_pos[1])
                interp_y = prev_pos[0] + t * (new_pos[0] - prev_pos[0])

                update_robot_transform(interp_x, interp_y, robot_yaw)

                # Update follow camera
                if follow_camera_enabled:
                    plotter.camera.position = (
                        interp_x + camera_offset[0],
                        interp_y + camera_offset[1],
                        camera_offset[2]
                    )
                    plotter.camera.focal_point = (interp_x, interp_y, 0)

                plotter.update()
                time.sleep(step_delay / INTERPOLATION_STEPS)
        else:
            # Instant movement
            update_robot_transform(new_pos[1], new_pos[0], robot_yaw)

            # Update follow camera
            if follow_camera_enabled:
                plotter.camera.position = (
                    new_pos[1] + camera_offset[0],
                    new_pos[0] + camera_offset[1],
                    camera_offset[2]
                )
                plotter.camera.focal_point = (new_pos[1], new_pos[0], 0)

    plotter.show(interactive_update=True, auto_close=False)

    # Process search events
    path_coords = []
    final_metrics = None
    nodes_expanded = 0
    frontier_count = 0
    last_robot_pos = start

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

            # Move robot to current expansion position (no smooth during search for speed)
            update_robot_position(pos, last_robot_pos, smooth=False)
            last_robot_pos = pos

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

            # Animate robot along final path with smooth movement
            if len(path_coords) > 0:
                for i, coord in enumerate(path_coords):
                    prev_coord = path_coords[i - 1] if i > 0 else None
                    update_robot_position(coord, prev_coord, smooth=(prev_coord is not None))
                    plotter.update()
                    if prev_coord is None:
                        time.sleep(step_delay)

            break

        # Update metrics text
        plotter.remove_actor(metrics_text)
        follow_status = " [FOLLOW]" if follow_camera_enabled else ""
        metrics_text = plotter.add_text(
            f"Expanded: {nodes_expanded} | Frontier: {frontier_count}{follow_status}",
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
