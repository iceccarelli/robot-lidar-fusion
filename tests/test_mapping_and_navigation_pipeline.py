from __future__ import annotations

from robot_hw.core.consistency_verification import ConsistencyVerifier
from robot_hw.mapping.occupancy_mapping import OccupancyMapper
from robot_hw.planning.navigation_manager import NavigationManager
from robot_hw.robot_config import load as load_config

FUSED_STATE = {
    "timestamp": 1.0,
    "position": (0.0, 0.0),
    "positions": {"joint1": 0.0, "joint2": 0.0},
    "fusion": {
        "objects": [
            {"centroid_camera_xyz": [0.2, 0.0, 1.6], "confidence": 0.9},
            {"centroid_camera_xyz": [-0.3, 0.0, 2.2], "confidence": 0.8},
        ]
    },
    "hazard_flags": {},
}


def test_occupancy_mapper_builds_grid_and_nav2_payload() -> None:
    mapper = OccupancyMapper(width=50, height=50, resolution_m=0.1, origin_xy=(-2.0, -2.0))

    occupancy = mapper.build_occupancy_grid(FUSED_STATE)
    costmap = mapper.build_costmap(occupancy)
    summary = mapper.summarize(occupancy, costmap)
    nav2_payload = mapper.to_nav2_costmap_dict(costmap)

    assert summary["occupied_cells"] >= 2
    assert summary["inflated_cells"] >= 1
    assert nav2_payload["metadata"]["width"] == 50
    assert len(nav2_payload["data"]) == 50 * 50
    assert nav2_payload["encoding"] == "costmap_2d"


def test_navigation_manager_emits_global_local_and_nav2_outputs() -> None:
    cfg = load_config()
    manager = NavigationManager(cfg)
    manager.set_goal((2.5, 0.0))

    plan = manager.update(FUSED_STATE)
    report = manager.get_latest_report()
    velocity = manager.get_latest_velocity_command()

    assert isinstance(plan, list)
    assert len(plan) >= 2
    assert report["status"] == "planned"
    assert report["planner"] in {"A_STAR", "RRT"}
    assert isinstance(report["global_plan"], list)
    assert isinstance(report["local_plan"], list)
    assert report["map"]["occupied_cells"] >= 2
    assert report["nav2"]["metadata"]["resolution"] == 0.1
    assert velocity[0] > 0.0


def test_navigation_manager_replans_when_map_signature_changes() -> None:
    cfg = load_config()
    manager = NavigationManager(cfg)
    manager.set_goal((2.5, 0.0))

    manager.update(FUSED_STATE)
    first_report = manager.get_latest_report()

    changed_state = {
        **FUSED_STATE,
        "fusion": {
            "objects": [
                {"centroid_camera_xyz": [0.2, 0.0, 1.6], "confidence": 0.9},
                {"centroid_camera_xyz": [0.0, 0.0, 0.8], "confidence": 0.95},
                {"centroid_camera_xyz": [0.1, 0.0, 1.1], "confidence": 0.92},
            ]
        },
    }
    manager.update(changed_state)
    second_report = manager.get_latest_report()

    assert first_report["map"]["occupied_cells"] != second_report["map"]["occupied_cells"]
    assert second_report["replanning_reason"] == "map_or_goal_change"


def test_consistency_verifier_accepts_stage5_navigation_fields() -> None:
    verifier = ConsistencyVerifier()
    state = {
        **FUSED_STATE,
        "map": {"occupied_cells": 2, "lethal_cells": 2, "inflated_cells": 8},
        "nav2_costmap": {
            "metadata": {
                "width": 10,
                "height": 10,
                "resolution": 0.1,
                "origin": {"x": 0.0, "y": 0.0},
            },
            "data": [0] * 100,
            "encoding": "costmap_2d",
        },
        "global_plan": [(0.0, 0.0), (1.0, 0.0)],
        "local_plan": [(0.0, 0.0), (0.5, 0.0)],
        "navigation": {"status": "planned"},
        "locomotion_commands": {"joint1": 0.1, "joint2": 0.1},
    }

    assert verifier.verify(state)
    assert verifier.errors == []
