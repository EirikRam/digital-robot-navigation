"""Entry point for robot navigation with modular architecture."""

from agents.robot_agent import RobotAgent
from environment.grid_environment import GridEnvironment
from planners.astar import AStarPlanner
from planners.dijkstra import DijkstraPlanner
from planners.greedy import GreedyBestFirstPlanner
from visualization.plotter import visualize_path
from visualization.pro_plotter import visualize_path_pro

PLANNER_TYPE = "astar"  # options: astar, dijkstra, greedy
VIZ_MODE = "pro"  # options: basic, pro

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


def run():
    environment = GridEnvironment(MAP)
    planner = _create_planner()
    agent = RobotAgent(planner)

    path, metrics = agent.navigate(START, GOAL, environment)

    # Add planner name to metrics for visualization
    metrics.planner_name = planner.name

    if VIZ_MODE == "pro":
        visualize_path_pro(environment, path, metrics)
    else:
        visualize_path(environment, path)

    print(path)
    print(
        {
            "planner": planner.name,
            "path_length": metrics.path_length,
            "nodes_expanded": metrics.nodes_expanded,
            "runtime": metrics.runtime,
            "heuristic_cost": metrics.heuristic_cost,
        }
    )


if __name__ == "__main__":
    run()
