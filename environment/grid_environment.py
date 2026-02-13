"""Grid environment with obstacle and neighborhood utilities."""


class GridEnvironment:
    def __init__(self, grid):
        self.grid = [row[:] for row in grid]
        self._visualization_hooks = []

    def add_visualization_hook(self, hook):
        self._visualization_hooks.append(hook)

    def notify_visualization_hooks(self):
        for hook in self._visualization_hooks:
            hook(self)

    def update_cell(self, x, y, value):
        self.grid[x][y] = value
        self.notify_visualization_hooks()

    def add_obstacle(self, x, y):
        self.update_cell(x, y, 1)

    def remove_obstacle(self, x, y):
        self.update_cell(x, y, 0)

    def in_bounds(self, x, y):
        return 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0])

    def is_free(self, x, y):
        return self.in_bounds(x, y) and self.grid[x][y] == 0

    def neighbors(self, state):
        x, y = state
        candidates = [
            ("Up", (x - 1, y)),
            ("Left", (x, y - 1)),
            ("Down", (x + 1, y)),
            ("Right", (x, y + 1)),
        ]
        return [(action, nxt) for action, nxt in candidates if self.is_free(*nxt)]
