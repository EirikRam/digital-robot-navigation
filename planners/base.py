"""Planner abstractions for navigation algorithms."""

from abc import ABC, abstractmethod


class Planner(ABC):
    """Abstract base class for all planners."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable planner name."""

    @abstractmethod
    def plan(self, start, goal, environment):
        """Compute a path from start to goal within an environment."""
