from __future__ import annotations

import json
from pathlib import Path

import pytest

from robot_hw.perception.lidar_utils import compute_proximity
from robot_hw.perception.sensor_frames import CameraFrame, LidarFrame
from robot_hw.perception.time_sync import TimeSync

DATASET_PATH = (
    Path(__file__).resolve().parent.parent
    / "datasets"
    / "sample_bags"
    / "fusion_demo_sequence.json"
)
METADATA_PATH = (
    Path(__file__).resolve().parent.parent / "datasets" / "sample_bags" / "metadata.yaml"
)


def _load_dataset() -> dict:
    return json.loads(DATASET_PATH.read_text(encoding="utf-8"))


def _build_lidar_frame(payload: dict) -> LidarFrame:
    return LidarFrame(
        timestamp=float(payload["timestamp"]),
        frame_id=str(payload["frame_id"]),
        points_xyz=[tuple(map(float, point)) for point in payload["points_xyz"]],
        intensities=[float(value) for value in payload["intensities"]],
        metadata=dict(payload.get("metadata", {})),
    )


def _build_camera_frame(payload: dict) -> CameraFrame:
    return CameraFrame(
        timestamp=float(payload["timestamp"]),
        frame_id=str(payload["frame_id"]),
        image=payload["image"],
        intrinsics=dict(payload.get("intrinsics", {})),
        metadata=dict(payload.get("metadata", {})),
    )


def test_demo_dataset_files_exist() -> None:
    assert DATASET_PATH.is_file()
    assert METADATA_PATH.is_file()


def test_demo_dataset_has_expected_frame_count() -> None:
    dataset = _load_dataset()
    assert dataset["dataset"] == "fusion_demo_sequence"
    assert len(dataset["frames"]) == 3


def test_demo_dataset_frames_can_be_synchronized() -> None:
    dataset = _load_dataset()
    matcher = TimeSync(max_offset=0.05)

    offsets: list[float] = []
    for item in dataset["frames"]:
        camera_frame = _build_camera_frame(item["camera"])
        lidar_frame = _build_lidar_frame(item["lidar"])
        matcher.add_camera_frame(camera_frame)
        result = matcher.match(lidar_frame)
        assert result.camera_frame is not None
        assert result.offset is not None
        offsets.append(result.offset)

    assert offsets == pytest.approx([0.01, 0.01, 0.01], abs=1e-9)


def test_demo_dataset_yields_forward_proximity_values() -> None:
    dataset = _load_dataset()
    proximities = []
    for item in dataset["frames"]:
        lidar_frame = _build_lidar_frame(item["lidar"])
        proximities.append(compute_proximity(lidar_frame.points_xyz, fov_degrees=90.0))

    assert proximities[0] is not None
    assert proximities[1] is not None
    assert proximities[2] == 1.6
    assert proximities[0] > proximities[1] > proximities[2]


def test_demo_dataset_camera_intrinsics_are_present() -> None:
    dataset = _load_dataset()
    for item in dataset["frames"]:
        intrinsics = item["camera"]["intrinsics"]
        assert intrinsics["fx"] > 0
        assert intrinsics["fy"] > 0
        assert "cx" in intrinsics
        assert "cy" in intrinsics
        assert len(intrinsics["D"]) == 5


def test_demo_dataset_image_payload_shape_is_consistent() -> None:
    dataset = _load_dataset()
    for item in dataset["frames"]:
        image = item["camera"]["image"]
        assert image["width"] == 8
        assert image["height"] == 6
        assert len(image["data"]) == image["height"]
        assert all(len(row) == image["width"] for row in image["data"])
        assert all(len(pixel) == 3 for row in image["data"] for pixel in row)
