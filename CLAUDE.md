# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Digital Robot Navigation is a Python-based pathfinding system using the A* search algorithm to find optimal paths on 2D grid-based environments with obstacles.

## Commands

```bash
# Install dependencies
pip install -r requirements.txt

# Run the main demo
python main.py

# Run smoke tests
python smoke_tests.py
```

## Architecture

The project follows a modular, layered architecture:

```
agents/          → RobotAgent orchestrates navigation by binding a planner
planners/        → Pathfinding algorithms (abstract Planner base + AStarPlanner)
environment/     → GridEnvironment manages 2D grid with obstacles
metrics/         → PlannerMetrics dataclass tracks performance
visualization/   → Matplotlib-based path visualization
```

### Data Flow

```
main.py
  └─> GridEnvironment (grid setup with obstacles)
  └─> AStarPlanner (implements Planner ABC)
  └─> RobotAgent.navigate(start, goal, environment)
      └─> planner.plan() returns (path, PlannerMetrics)
  └─> visualize_path() displays results
```

### Key Abstractions

- **Planner ABC** (`planners/base.py`): Abstract base class with `plan(start, goal, environment)` method. Extend this to add new pathfinding algorithms.
- **GridEnvironment** (`environment/grid_environment.py`): Grid values `0` = open, `1` = obstacle. Provides `neighbors(state)` and `is_free(x, y)` for planners.
- **AStarPlanner** uses internal `_RobotProblem` class to adapt the environment to simpleai's SearchProblem interface.

### Movement Model

- 4-directional movement only (Up, Down, Left, Right)
- Manhattan distance heuristic for A*
- Path returned as list of direction strings
