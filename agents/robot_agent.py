"""Robot agent orchestration."""


class RobotAgent:
    def __init__(self, planner):
        self.planner = planner

    def navigate(self, start, goal, environment):
        return self.planner.plan(start, goal, environment)
