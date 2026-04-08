# Datasets

This directory contains the reproducible demo assets for **robot-lidar-fusion**.

The immediate purpose of these assets is to let a new user validate the perception pipeline **without custom hardware**. The dataset format is intentionally simple and repository-local so it can be versioned, inspected, and extended while the ROS 2-native bag replay workflow is being hardened.

## Contents

| Path | Purpose |
|---|---|
| `sample_bags/fusion_demo_sequence.json` | Small deterministic LiDAR-camera replay sequence used for local smoke tests and future ROS 2 replay adapters |
| `sample_bags/metadata.yaml` | Dataset provenance, topic naming, frame IDs, and expected replay properties |

## Dataset design

The sample dataset is not meant to be a benchmark-quality public corpus. It is meant to be a **small, inspectable, version-controlled replay fixture** that supports these workflows:

| Workflow | Outcome |
|---|---|
| Local Python replay | Validates parsing, synchronisation, and fusion smoke tests |
| Future ROS 2 replay adapter | Publishes deterministic LiDAR and camera topics from repository-local assets |
| CI smoke validation | Confirms demo artifacts are present and structurally valid |
| Visual debug workflow | Supports projected overlays and synchronisation inspection |

## Data model

Each replay frame stores a timestamp, LiDAR point list, camera intrinsics, and lightweight image payload metadata. The format mirrors the repository's internal `LidarFrame` and `CameraFrame` contracts closely enough that adapters can be implemented without guessing field meanings.

## Usage

The next release stage adds launch files and a replay utility that treat this dataset as the default no-hardware demo path. Until then, contributors should keep the schema stable and append only demonstrably useful fields.
