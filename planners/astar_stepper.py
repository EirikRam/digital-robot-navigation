"""A* stepper planner with step-by-step iteration for visualization."""

import heapq
import time

from metrics.tracker import PlannerMetrics
from planners.base import Planner


class AStarStepperPlanner(Planner):
    """A* planner that can yield step events during search."""

    @property
    def name(self) -> str:
        return "A* Stepper"

    def _heuristic(self, pos, goal):
        """Manhattan distance heuristic."""
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def _reconstruct_path(self, came_from, current, start):
        """Reconstruct path from came_from map."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _to_action_path(self, path_coords):
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

    def iterate(self, start, goal, environment):
        """
        Generator that yields step events during A* search.

        Yields events with schema:
        - {"type": "expand", "pos": (r,c), "g": g, "h": h, "f": f}
        - {"type": "frontier", "pos": (r,c), "g": g, "h": h, "f": f}
        - {"type": "path", "path": [(r,c), ...]}

        Returns metrics via final "path" event's "metrics" field.
        """
        # Priority queue: (f, g, pos)
        # Using g as tiebreaker (prefer nodes closer to goal)
        open_set = []
        h_start = self._heuristic(start, goal)
        heapq.heappush(open_set, (h_start, 0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: h_start}

        open_set_hash = {start}
        closed_set = set()

        nodes_expanded = 0
        start_time = time.perf_counter()

        # Yield initial frontier
        yield {"type": "frontier", "pos": start, "g": 0, "h": h_start, "f": h_start}

        while open_set:
            _, current_g, current = heapq.heappop(open_set)
            open_set_hash.discard(current)

            if current in closed_set:
                continue

            closed_set.add(current)
            nodes_expanded += 1

            h = self._heuristic(current, goal)
            f = g_score[current] + h

            yield {"type": "expand", "pos": current, "g": g_score[current], "h": h, "f": f}

            if current == goal:
                # Found the goal
                runtime = time.perf_counter() - start_time
                path_coords = self._reconstruct_path(came_from, current, start)
                metrics = PlannerMetrics(
                    path_length=len(path_coords) - 1,
                    nodes_expanded=nodes_expanded,
                    runtime=runtime,
                    heuristic_cost=self._heuristic(start, goal)
                )
                yield {"type": "path", "path": path_coords, "metrics": metrics}
                return

            # Explore neighbors
            for action, neighbor in environment.neighbors(current):
                if neighbor in closed_set:
                    continue

                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h_neighbor = self._heuristic(neighbor, goal)
                    f_neighbor = tentative_g + h_neighbor
                    f_score[neighbor] = f_neighbor

                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_neighbor, tentative_g, neighbor))
                        open_set_hash.add(neighbor)
                        yield {"type": "frontier", "pos": neighbor, "g": tentative_g, "h": h_neighbor, "f": f_neighbor}

        # No path found
        runtime = time.perf_counter() - start_time
        metrics = PlannerMetrics(
            path_length=0,
            nodes_expanded=nodes_expanded,
            runtime=runtime,
            heuristic_cost=self._heuristic(start, goal)
        )
        yield {"type": "path", "path": [], "metrics": metrics}

    def plan(self, start, goal, environment):
        """
        Compute path from start to goal (standard Planner interface).

        Returns (path, metrics) where path is list of (action, state) tuples.
        """
        path_coords = []
        metrics = None

        for event in self.iterate(start, goal, environment):
            if event["type"] == "path":
                path_coords = event["path"]
                metrics = event["metrics"]

        action_path = self._to_action_path(path_coords)
        return action_path, metrics
