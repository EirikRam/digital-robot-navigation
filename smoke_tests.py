import time

from environment.grid_environment import GridEnvironment
from planners.astar import AStarPlanner

def run_smoke():
    grid = [
        [0,0,0,0,0,0,0,0],
        [1,1,1,1,0,1,1,0],
        [0,0,0,1,0,0,0,0],
        [0,1,0,0,0,1,1,0],
        [0,1,0,1,0,0,0,0],
        [0,0,0,1,0,1,0,0],
        [0,1,0,0,0,1,0,0],
        [0,1,1,1,0,0,0,0],
    ]

    env = GridEnvironment(grid)
    start, goal = (0,0), (7,6)

    planner = AStarPlanner()
    t0 = time.time()
    path, metrics = planner.plan(start, goal, env)
    dt = time.time() - t0

    print("Path:", path)
    print("Metrics:", metrics.to_dict())
    print("Execution Time:", dt)

    assert path, "Expected a valid path"
    assert metrics.path_length > 0, "Expected non-zero path length"
    assert metrics.nodes_expanded >= 0, "Expected nodes expanded metric"
    assert dt < 10, "This should not take forever"

    print("SMOKE OK:", metrics.to_dict())

if __name__ == "__main__":
    run_smoke()
