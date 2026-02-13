"""Metrics tracking for planner runs."""

from dataclasses import dataclass


@dataclass
class PlannerMetrics:
    path_length: int = 0
    nodes_expanded: int = 0
    runtime: float = 0.0
    heuristic_cost: int = 0
