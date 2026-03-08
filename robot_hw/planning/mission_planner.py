"""Mission Planner Module
=========================

The mission planner transforms high‑level goals into concrete tasks
that can be executed by the robot.  Goals may originate from
operators, remote servers or autonomous decision logic.  The planner
coordinates navigation, locomotion and manipulation primitives to
achieve these goals while respecting hazards, power constraints and
thermal limits.

In this simplified implementation the planner maintains a queue of
waypoint goals (x, y coordinates) and generates navigation tasks
(``Task`` objects) for each.  It consults the battery, thermal and
hazard managers before issuing tasks, deferring execution if
conditions are unsafe.  The planner can be extended to support more
complex missions, dynamic prioritisation and learning‑based
replanning.
"""

from __future__ import annotations

import logging
from typing import Any

from robot_hw.core.hazard_manager import HazardManager
from robot_hw.planning.task_hardware_mapping import Task
from robot_hw.power.battery_management import BatteryManager
from robot_hw.power.thermal_management import ThermalManager
from robot_hw.robot_config import RobotConfig


class MissionPlanner:
    """Convert high‑level goals into executable tasks.

    Parameters
    ----------
    config : RobotConfig
        System configuration used for parameter tuning and environment
        awareness.
    battery_manager : BatteryManager
        Manager used to query power availability.  The planner defers
        tasks when available energy falls below a threshold.
    thermal_manager : ThermalManager
        Manager used to monitor thermal state and decide whether to
        throttle or defer tasks.
    hazard_manager : HazardManager
        Manager providing hazard flags.  Tasks will not be issued
        while any hazard or fault is active.
    """

    def __init__(self,
                 config: RobotConfig,
                 battery_manager: BatteryManager,
                 thermal_manager: ThermalManager,
                 hazard_manager: HazardManager) -> None:
        self._config = config
        self._battery_manager = battery_manager
        self._thermal_manager = thermal_manager
        self._hazard_manager = hazard_manager
        # Queue of pending mission goals.  Each goal is a dict that may
        # contain coordinates or other parameters.  In this
        # implementation goals are simple waypoints with ``x`` and ``y``.
        self._goal_queue: list[dict[str, Any]] = []
        # Logger for debug and diagnostic information
        self._logger = logging.getLogger(self.__class__.__name__)

    def add_goal(self, goal: dict[str, Any]) -> None:
        """Add a new mission goal to the queue.

        Parameters
        ----------
        goal : dict[str, Any]
            A dictionary describing the goal.  Keys ``x`` and ``y``
            denote target coordinates.  Additional keys may be
            supported in future extensions (e.g. ``z`` for 3D, ``task``
            type).
        """
        if isinstance(goal, dict) and "x" in goal and "y" in goal:
            self._goal_queue.append({"x": goal["x"], "y": goal["y"]})

    def clear_goals(self) -> None:
        """Remove all pending goals from the queue."""
        self._goal_queue.clear()

    def plan_tasks(self, current_state: dict[str, Any]) -> list[Task]:
        """Generate a list of tasks for the next planning cycle.

        The planner consults the battery and thermal managers and the
        hazard manager to determine whether tasks can be executed
        safely.  If conditions are acceptable and there is at least one
        goal in the queue, a navigation task is created for the next
        waypoint.  Goals are processed sequentially.

        Parameters
        ----------
        current_state : dict[str, Any]
            The most recent fused sensor state.  Currently unused but
            retained for future enhancements (e.g. dynamic replanning).

        Returns
        -------
        list[Task]
            A list of high‑level tasks ready to be passed to the
            hardware mapper.  The list may be empty if no goals are
            pending or if conditions are unsafe.
        """
        tasks: list[Task] = []
        # Inspect hazards and skip tasks if any active hazard has high risk
        hazards = self._hazard_manager.current_hazards()
        for info in hazards.values():
            try:
                risk = info.get("risk_level")
                if risk == "high":
                    # Defer tasks due to high‑severity hazard
                    self._logger.debug("Deferring tasks due to hazard with high risk: %s", info)
                    return tasks
            except Exception:
                continue
        # Skip task issuance if the battery or thermal state is unsafe
        if not self._battery_manager.is_ok() or not self._thermal_manager.is_within_limits():
            return tasks
        # Pop the next goal and generate a navigation task
        if self._goal_queue:
            goal = self._goal_queue.pop(0)
            try:
                x = float(goal["x"])
                y = float(goal["y"])
            except Exception:
                # Invalid goal; skip and continue
                return tasks
            # Create a navigation task with optional priority field
            task = Task(id="navigate_to", parameters={"x": x, "y": y, "priority": len(tasks)})
            tasks.append(task)
        return tasks
