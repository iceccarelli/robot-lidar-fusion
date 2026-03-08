"""Navigation Manager Module
===========================

This module defines the :class:`NavigationManager`, responsible for
computing paths and waypoints for the robot to reach its goals while
avoiding obstacles.  It interfaces with sensor processing to obtain
the robot's current pose and environment map, and produces high‑level
trajectory commands that are then converted to joint‑level
instructions by the task mapping layer.

Key responsibilities include:

* **Global path planning:** Compute an efficient path from the current
  position to a goal location using algorithms such as A*, RRT or
  Dijkstra on a map representation.
* **Local obstacle avoidance:** Adjust the path in real time based on
  dynamic obstacles detected by sensors.
* **Interface consistency:** Provide a stable API for updating state,
  setting goals and retrieving the current plan.

In stage A this class is a skeleton; algorithmic implementation will
be added in later stages.
"""

from __future__ import annotations

import logging
import random
from typing import Any

from robot_hw.robot_config import RobotConfig


class NavigationManager:
    """Plan and manage navigation tasks for the robot.

    Parameters
    ----------
    config : RobotConfig
        System configuration providing parameters such as maximum
        velocities, environment profile and sensor calibration.  See
        :mod:`robot_config` for details.
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        self._current_plan: list[tuple[float, float]] = []
        self._goal: tuple[float, float] | None = None
        # Planning algorithm selection (e.g. A_STAR, RRT)
        self._algorithm = getattr(config, 'path_planning_algorithm', 'A_STAR').upper()
        # Logger for diagnostics
        self._logger = logging.getLogger(self.__class__.__name__)

    def set_goal(self, goal: tuple[float, float]) -> None:
        """Set a new navigation goal.

        Parameters
        ----------
        goal : tuple[float, float]
            The target coordinates in the robot's reference frame.
        """
        self._goal = goal
        # Invalidate existing plan when a new goal is set
        self._current_plan.clear()

    def update(self, state: Any) -> list[tuple[float, float]]:
        """Update the navigation plan based on current state.

        This method should be called each control cycle.  It inspects
        the robot's current state (pose, obstacles) and computes or
        refines a path to the current goal.  The implementation
        provided here is intentionally simple: it creates a direct path
        consisting of the current position and the goal.  Future
        versions should implement algorithms such as A* or RRT and
        incorporate obstacle avoidance.

        Parameters
        ----------
        state : Any
            A dictionary containing at least a ``positions`` entry
            mapping joint IDs to positions.  Only the first two joint
            positions are used to derive a 2D current position; if
            unavailable, the origin is assumed.

        Returns
        -------
        list[tuple[float, float]]
            A list of waypoints representing the current plan.  An
            empty list indicates no plan (e.g. no goal set).
        """
        if self._goal is None:
            self._current_plan.clear()
            return []
        # Derive current position from joint positions (joint1→x, joint2→y)
        x = 0.0
        y = 0.0
        try:
            if isinstance(state, dict):
                pos = state.get("positions", {})
                if pos:
                    vals = list(pos.values())
                    if len(vals) >= 1:
                        x = float(vals[0])
                    if len(vals) >= 2:
                        y = float(vals[1])
        except Exception as exc:
            self._logger.debug("Error computing current position: %s", exc)
        current = (x, y)
        goal = self._goal
        # Compute a new path using the configured algorithm
        plan: list[tuple[float, float]]
        plan = self._compute_rrt(current, goal) if self._algorithm == "RRT" else self._compute_a_star(current, goal)
        # Incorporate simple hazard avoidance by shifting path if hazards present
        try:
            hazard_flags = state.get("hazard_flags", {}) if isinstance(state, dict) else {}
            # If any dynamic obstacle hazard is active, offset the path in y
            dynamic_keys = {"train", "car", "human", "pedestrian"}
            if any(h in hazard_flags for h in dynamic_keys):
                plan = [(px, py + 1.0) for px, py in plan]
        except Exception as exc:
            self._logger.debug("Error applying hazard avoidance: %s", exc)
        self._current_plan = list(plan)
        return list(self._current_plan)

    # ------------------------------------------------------------------
    # Path planning algorithms (simplified for demonstration)
    # ------------------------------------------------------------------
    def _compute_a_star(self, start: tuple[float, float], goal: tuple[float, float]) -> list[tuple[float, float]]:
        """Compute a simple straight‑line path with an intermediate waypoint.

        A placeholder for a grid‑based A* algorithm.  Returns a list of
        waypoints from start to goal via the midpoint.  A real implementation
        would compute optimal paths over a map with obstacles.
        """
        sx, sy = start
        gx, gy = goal
        # Midpoint for demonstration
        mx = (sx + gx) / 2.0
        my = (sy + gy) / 2.0
        return [start, (mx, my), goal]

    def _compute_rrt(self, start: tuple[float, float], goal: tuple[float, float]) -> list[tuple[float, float]]:
        """Compute a randomised path using a rudimentary RRT‐like method.

        This demonstration generates random intermediate waypoints between
        start and goal.  The first waypoint is offset randomly from the
        start, and the second is offset randomly from the goal.  Real RRT
        builds a tree exploring the space; this is merely illustrative.
        """
        sx, sy = start
        gx, gy = goal
        # Generate two random intermediate waypoints
        def rand_offset(a: float, b: float) -> float:
            return random.uniform(min(a, b), max(a, b))
        p1 = (rand_offset(sx, gx), rand_offset(sy, gy))
        p2 = (rand_offset(sx, gx), rand_offset(sy, gy))
        return [start, p1, p2, goal]

    def get_plan(self) -> list[tuple[float, float]]:
        """Return the current navigation plan.

        Returns
        -------
        list[tuple[float, float]]
            A possibly empty list of waypoints from the robot's current
            position to the goal.
        """
        return list(self._current_plan)
