from __future__ import annotations

from robot_hw.robot_orchestrator import RobotOrchestrator


def test_orchestrator_emits_navigation_state_after_goal_submission() -> None:
    orchestrator = RobotOrchestrator(
        cycle_time=0.001, total_memory_bytes=1024 * 1024, battery_capacity_wh=100.0
    )
    orchestrator.submit_goal({"x": 1.5, "y": 0.0})

    orchestrator.run(num_cycles=1)

    navigation = orchestrator.current_state.get("navigation")
    assert isinstance(navigation, dict)
    assert navigation["status"] in {"planned", "blocked"}
    assert tuple(navigation["goal"]) == (1.5, 0.0)
    assert "map" in navigation
    assert "nav2" in navigation
    assert isinstance(orchestrator.current_state.get("global_plan"), list)
    assert isinstance(orchestrator.current_state.get("local_plan"), list)
    assert isinstance(orchestrator.current_state.get("nav2_costmap"), dict)
    assert isinstance(orchestrator.current_state.get("map"), dict)
