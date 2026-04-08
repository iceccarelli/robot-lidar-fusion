"""Navigation manager with occupancy mapping, costmaps, replanning, and Nav2-style outputs.

This module upgrades the repository's placeholder navigation path into a
planning-ready component that consumes fused perception, builds deterministic
map products, computes global plans over an inflated costmap, and exposes a
small compatibility surface for ROS 2 / Nav2-style integration.
"""

from __future__ import annotations

import heapq
import logging
from math import atan2, hypot
from typing import Any

from robot_hw.mapping.occupancy_mapping import GridMap, OccupancyMapper
from robot_hw.robot_config import RobotConfig


class NavigationManager:
    """Plan and manage navigation tasks for the robot."""

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        self._current_plan: list[tuple[float, float]] = []
        self._local_plan: list[tuple[float, float]] = []
        self._goal: tuple[float, float] | None = None
        self._algorithm = getattr(config, "path_planning_algorithm", "A_STAR").upper()
        self._logger = logging.getLogger(self.__class__.__name__)
        self._mapper = OccupancyMapper(
            width=80,
            height=80,
            resolution_m=0.1,
            origin_xy=(-4.0, -4.0),
            inflation_radius_m=0.4,
            obstacle_cost=100,
        )
        self._latest_report: dict[str, Any] = {
            "status": "idle",
            "planner": self._algorithm,
        }
        self._latest_velocity_command: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_plan_signature: tuple[Any, ...] | None = None

    def set_goal(self, goal: tuple[float, float]) -> None:
        self._goal = (float(goal[0]), float(goal[1]))
        self._current_plan.clear()
        self._local_plan.clear()
        self._latest_velocity_command = (0.0, 0.0, 0.0)
        self._latest_report = {
            "status": "goal_updated",
            "planner": self._algorithm,
            "goal": self._goal,
        }

    def update(self, state: Any) -> list[tuple[float, float]]:
        if self._goal is None:
            self._current_plan.clear()
            self._local_plan.clear()
            self._latest_velocity_command = (0.0, 0.0, 0.0)
            self._latest_report = {
                "status": "no_goal",
                "planner": self._algorithm,
            }
            return []

        current = self._extract_current_position(state)
        occupancy = self._mapper.build_occupancy_grid(state if isinstance(state, dict) else {})
        costmap = self._mapper.build_costmap(occupancy)
        summary = self._mapper.summarize(occupancy, costmap)
        signature = self._build_plan_signature(state, current, self._goal, summary)

        replanning_reason = (
            "map_or_goal_change" if signature != self._last_plan_signature else "tracking"
        )
        if signature != self._last_plan_signature or not self._current_plan:
            self._current_plan = self._compute_plan(current, self._goal, costmap)
            self._last_plan_signature = signature
        self._local_plan = self._current_plan[: min(10, len(self._current_plan))]
        self._latest_velocity_command = self._compute_velocity_command(current, self._local_plan)

        status = "planned" if self._current_plan else "blocked"
        self._latest_report = {
            "status": status,
            "planner": self._algorithm,
            "replanning_reason": replanning_reason,
            "goal": self._goal,
            "current_position": current,
            "global_plan": list(self._current_plan),
            "local_plan": list(self._local_plan),
            "map": summary,
            "nav2": self._mapper.to_nav2_costmap_dict(costmap),
            "velocity_command": self._latest_velocity_command,
        }
        return list(self._current_plan)

    def get_plan(self) -> list[tuple[float, float]]:
        return list(self._current_plan)

    def get_local_plan(self) -> list[tuple[float, float]]:
        return list(self._local_plan)

    def get_latest_velocity_command(self) -> tuple[float, float, float]:
        return tuple(self._latest_velocity_command)

    def get_latest_report(self) -> dict[str, Any]:
        return dict(self._latest_report)

    def _extract_current_position(self, state: Any) -> tuple[float, float]:
        if isinstance(state, dict):
            position = state.get("position")
            if isinstance(position, (list, tuple)) and len(position) >= 2:
                try:
                    return (float(position[0]), float(position[1]))
                except Exception:
                    pass
            positions = state.get("positions", {})
            if isinstance(positions, dict) and positions:
                values = list(positions.values())
                try:
                    x = float(values[0]) if len(values) >= 1 else 0.0
                    y = float(values[1]) if len(values) >= 2 else 0.0
                    return (x, y)
                except Exception:
                    return (0.0, 0.0)
        return (0.0, 0.0)

    def _build_plan_signature(
        self,
        state: Any,
        current: tuple[float, float],
        goal: tuple[float, float],
        map_summary: dict[str, Any],
    ) -> tuple[Any, ...]:
        hazard_flags = {}
        if isinstance(state, dict) and isinstance(state.get("hazard_flags"), dict):
            hazard_flags = state["hazard_flags"]
        risk_markers = tuple(sorted(str(key) for key in hazard_flags))
        return (
            round(current[0], 2),
            round(current[1], 2),
            round(goal[0], 2),
            round(goal[1], 2),
            map_summary.get("occupied_cells", 0),
            map_summary.get("inflated_cells", 0),
            risk_markers,
        )

    def _compute_plan(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        costmap: GridMap,
    ) -> list[tuple[float, float]]:
        if self._algorithm == "RRT":
            return self._compute_rrt_like(start_xy, goal_xy, costmap)
        return self._compute_a_star(start_xy, goal_xy, costmap)

    def _compute_a_star(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        costmap: GridMap,
    ) -> list[tuple[float, float]]:
        start = costmap.world_to_grid(*start_xy)
        goal = costmap.world_to_grid(*goal_xy)
        if not costmap.in_bounds(*start) or not costmap.in_bounds(*goal):
            return []
        if costmap.data[goal[1]][goal[0]] >= 100:
            return []

        open_heap: list[tuple[float, tuple[int, int]]] = []
        heapq.heappush(open_heap, (0.0, start))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                return self._reconstruct_world_path(came_from, current, costmap)

            for neighbour in self._neighbors(current, costmap):
                nx, ny = neighbour
                cell_cost = costmap.data[ny][nx]
                if cell_cost >= 100:
                    continue
                tentative = g_score[current] + 1.0 + (cell_cost / 100.0)
                if tentative < g_score.get(neighbour, float("inf")):
                    came_from[neighbour] = current
                    g_score[neighbour] = tentative
                    f_score = tentative + self._heuristic(neighbour, goal)
                    heapq.heappush(open_heap, (f_score, neighbour))
        return []

    def _compute_rrt_like(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        costmap: GridMap,
    ) -> list[tuple[float, float]]:
        direct = self._compute_a_star(start_xy, goal_xy, costmap)
        if direct:
            return direct
        mid = ((start_xy[0] + goal_xy[0]) / 2.0, (start_xy[1] + goal_xy[1]) / 2.0)
        first_leg = self._compute_a_star(start_xy, mid, costmap)
        second_leg = self._compute_a_star(mid, goal_xy, costmap)
        if first_leg and second_leg:
            return first_leg[:-1] + second_leg
        return []

    def _neighbors(self, cell: tuple[int, int], costmap: GridMap) -> list[tuple[int, int]]:
        cx, cy = cell
        neighbours: list[tuple[int, int]] = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx = cx + dx
            ny = cy + dy
            if costmap.in_bounds(nx, ny):
                neighbours.append((nx, ny))
        return neighbours

    def _heuristic(self, current: tuple[int, int], goal: tuple[int, int]) -> float:
        return abs(goal[0] - current[0]) + abs(goal[1] - current[1])

    def _reconstruct_world_path(
        self,
        came_from: dict[tuple[int, int], tuple[int, int]],
        current: tuple[int, int],
        costmap: GridMap,
    ) -> list[tuple[float, float]]:
        cells = [current]
        while current in came_from:
            current = came_from[current]
            cells.append(current)
        cells.reverse()
        return [
            (
                costmap.origin_xy[0] + (ix + 0.5) * costmap.resolution_m,
                costmap.origin_xy[1] + (iy + 0.5) * costmap.resolution_m,
            )
            for ix, iy in cells
        ]

    def _compute_velocity_command(
        self,
        current: tuple[float, float],
        local_plan: list[tuple[float, float]],
    ) -> tuple[float, float, float]:
        if not local_plan:
            return (0.0, 0.0, 0.0)
        target = local_plan[1] if len(local_plan) > 1 else local_plan[0]
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        distance = hypot(dx, dy)
        vx = min(distance, 0.5)
        yaw_rate = atan2(dy, dx) if distance > 1e-9 else 0.0
        return (vx, 0.0, yaw_rate)
