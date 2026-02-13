"""A* planner implementation."""

import time

from simpleai.search import SearchProblem, astar

from metrics.tracker import PlannerMetrics
from planners.base import Planner


class _RobotProblem(SearchProblem):
    def __init__(self, initial_state, goal_state, environment, metrics):
        super().__init__(initial_state)
        self.goal_state = goal_state
        self.environment = environment
        self.metrics = metrics

    def actions(self, state):
        self.metrics.nodes_expanded += 1
        return [action for action, _ in self.environment.neighbors(state)]

    def result(self, state, action):
        transitions = dict(self.environment.neighbors(state))
        return transitions[action]

    def is_goal(self, state):
        return state == self.goal_state

    def heuristic(self, state):
        x, y = state
        gx, gy = self.goal_state
        return abs(x - gx) + abs(y - gy)


class AStarPlanner(Planner):
    @property
    def name(self) -> str:
        return "A*"

    def plan(self, start, goal, environment):
        metrics = PlannerMetrics(heuristic_cost=abs(start[0] - goal[0]) + abs(start[1] - goal[1]))

        problem = _RobotProblem(start, goal, environment, metrics)

        start_time = time.perf_counter()
        result = astar(problem)
        metrics.runtime = time.perf_counter() - start_time

        path = result.path() if result else []
        metrics.path_length = max(len(path) - 1, 0)

        return path, metrics
