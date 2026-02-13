"""Entry point for robot navigation with modular architecture."""

from agents.robot_agent import RobotAgent
from environment.grid_environment import GridEnvironment
from planners.astar import AStarPlanner
from visualization.plotter import visualize_path

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


def run():
    environment = GridEnvironment(MAP)
    planner = AStarPlanner()
    agent = RobotAgent(planner)

    path, metrics = agent.navigate(START, GOAL, environment)
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
