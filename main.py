"""Entry point for robot navigation with modular architecture."""

from agents.robot_agent import RobotAgent
from environment.grid_environment import GridEnvironment
from planners.astar import AStarPlanner
from planners.dijkstra import DijkstraPlanner
from planners.greedy import GreedyBestFirstPlanner
from planners.astar_stepper import AStarStepperPlanner
from visualization.plotter import visualize_path
from visualization.pro_plotter import visualize_path_pro
from visualization.viz3d_live import visualize_search_3d_live

PLANNER_TYPE = "astar"  # options: astar, dijkstra, greedy
VIZ_MODE = "3d_live"  # options: basic, pro, 3d_live

MAP = [
    [0, 0, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 0, 0, 0],
    [1, 0, 0, 0, 0, 0, 1],
    [0, 1, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0, 0],
    [0, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0],
    [0, 1, 0, 1, 0, 0, 0],
]

START = (0, 0)
GOAL = (7, 6)


def _create_planner():
    if PLANNER_TYPE == "astar":
        return AStarPlanner()
    elif PLANNER_TYPE == "dijkstra":
        return DijkstraPlanner()
    elif PLANNER_TYPE == "greedy":
        return GreedyBestFirstPlanner()
    else:
        raise ValueError(f"Unknown planner type: {PLANNER_TYPE}")


def _path_coords_to_action_path(path_coords):
    """Convert coordinate path to (action, state) format."""
    if not path_coords:
        return []

    result = [(None, path_coords[0])]
    for i in range(1, len(path_coords)):
        prev = path_coords[i - 1]
        curr = path_coords[i]
        dr = curr[0] - prev[0]
        dc = curr[1] - prev[1]

        if dr == -1:
            action = "Up"
        elif dr == 1:
            action = "Down"
        elif dc == -1:
            action = "Left"
        elif dc == 1:
            action = "Right"
        else:
            action = "Unknown"

        result.append((action, curr))
    return result


def run():
    environment = GridEnvironment(MAP)

    if VIZ_MODE == "3d_live":
        # Use stepper planner with 3D live visualization
        stepper = AStarStepperPlanner()
        search_iter = stepper.iterate(START, GOAL, environment)
        path_coords, metrics = visualize_search_3d_live(
            environment, search_iter, START, GOAL
        )
        path = _path_coords_to_action_path(path_coords)
        planner_name = stepper.name
    else:
        # Standard planner workflow
        planner = _create_planner()
        agent = RobotAgent(planner)
        path, metrics = agent.navigate(START, GOAL, environment)
        planner_name = planner.name

        # Add planner name to metrics for visualization
        metrics.planner_name = planner_name

        if VIZ_MODE == "pro":
            visualize_path_pro(environment, path, metrics)
        else:
            visualize_path(environment, path)

    print(path)
    print(
        {
            "planner": planner_name,
            "path_length": metrics.path_length,
            "nodes_expanded": metrics.nodes_expanded,
            "runtime": metrics.runtime,
            "heuristic_cost": metrics.heuristic_cost,
        }
    )


if __name__ == "__main__":
    run()
