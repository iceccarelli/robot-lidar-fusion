#!/usr/bin/env python3
"""Replay the bundled LiDAR-camera demo dataset.

This script is intentionally lightweight so the repository has a
hardware-free reproducible demo path even before full ROS 2 bag replay is
available in every environment. When ROS 2 is present, launch files can
wrap this script as part of a broader RViz workflow. When ROS 2 is not
present, the script still validates dataset structure, timestamp
synchronisation, and early fusion outputs locally.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from robot_hw.perception.lidar_utils import compute_proximity
from robot_hw.perception.sensor_frames import CameraFrame, LidarFrame
from robot_hw.perception.time_sync import TimeSync

DEFAULT_DATASET = REPO_ROOT / "datasets" / "sample_bags" / "fusion_demo_sequence.json"


def load_dataset(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def build_lidar_frame(payload: dict[str, Any]) -> LidarFrame:
    points_xyz = [tuple(map(float, point)) for point in payload["points_xyz"]]
    intensities = payload.get("intensities")
    if intensities is not None:
        intensities = [float(value) for value in intensities]
    return LidarFrame(
        timestamp=float(payload["timestamp"]),
        frame_id=str(payload["frame_id"]),
        points_xyz=points_xyz,
        intensities=intensities,
        metadata=dict(payload.get("metadata", {})),
    )


def build_camera_frame(payload: dict[str, Any]) -> CameraFrame:
    return CameraFrame(
        timestamp=float(payload["timestamp"]),
        frame_id=str(payload["frame_id"]),
        image=payload["image"],
        intrinsics=dict(payload.get("intrinsics", {})),
        metadata=dict(payload.get("metadata", {})),
    )


def summarise_pair(lidar_frame: LidarFrame, camera_frame: CameraFrame) -> dict[str, Any]:
    proximity = compute_proximity(lidar_frame.points_xyz, fov_degrees=90.0)
    return {
        "lidar_timestamp": lidar_frame.timestamp,
        "camera_timestamp": camera_frame.timestamp,
        "sync_offset_seconds": abs(lidar_frame.timestamp - camera_frame.timestamp),
        "lidar_points": len(lidar_frame.points_xyz),
        "camera_resolution": (
            (
                camera_frame.image.get("width"),
                camera_frame.image.get("height"),
            )
            if isinstance(camera_frame.image, dict)
            else None
        ),
        "proximity_m": proximity,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay the bundled robot-lidar-fusion demo dataset"
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=DEFAULT_DATASET,
        help="Path to the JSON replay dataset",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Replay rate multiplier. Values above 1.0 replay faster.",
    )
    parser.add_argument(
        "--max-offset",
        type=float,
        default=0.05,
        help="Maximum LiDAR-camera timestamp offset in seconds for matching.",
    )
    parser.add_argument(
        "--no-sleep",
        action="store_true",
        help="Disable realtime sleeping and print all frames immediately.",
    )
    args = parser.parse_args()

    dataset = load_dataset(args.dataset)
    frames = dataset.get("frames", [])
    matcher = TimeSync(max_offset=float(args.max_offset))

    print(f"replay: dataset={dataset.get('dataset')} frames={len(frames)} rate={args.rate}")

    previous_time: float | None = None
    for item in frames:
        lidar_frame = build_lidar_frame(item["lidar"])
        camera_frame = build_camera_frame(item["camera"])
        matcher.add_camera_frame(camera_frame)
        match = matcher.match(lidar_frame)

        if previous_time is not None and not args.no_sleep:
            delta = max(lidar_frame.timestamp - previous_time, 0.0)
            time.sleep(delta / max(args.rate, 1e-6))
        previous_time = lidar_frame.timestamp

        if match is None:
            print(
                "replay: unmatched frame "
                f"seq={item.get('sequence_id')} lidar_ts={lidar_frame.timestamp:.3f}"
            )
            continue

        summary = summarise_pair(match.lidar_frame, match.camera_frame)
        print(
            "replay: matched "
            f"seq={item.get('sequence_id')} "
            f"offset={summary['sync_offset_seconds']:.3f}s "
            f"points={summary['lidar_points']} "
            f"proximity={summary['proximity_m']}"
        )

    print("replay: complete")


if __name__ == "__main__":
    main()
