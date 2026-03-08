# Integration Guide

This guide explains how to integrate the Robot LiDAR Fusion control stack with your specific robot hardware, sensors, and deployment environment. It covers hardware backends, sensor drivers, ROS2 integration, and deployment strategies.

## Connecting Your Robot Hardware

The control stack communicates with hardware through the `HardwareSynchronizer` class in `robot_hw/core/hardware_synchronization.py`. The default implementation uses a `MockHardware` backend that simulates joint positions and sensor readings. To connect your robot, you need to implement a hardware backend that translates between the control stack's abstract commands and your robot's SDK.

Your hardware backend must provide two methods. `read_state()` returns a dictionary containing joint positions, velocities, and torques keyed by joint name. `write_commands(commands)` accepts a dictionary of joint commands and dispatches them to the physical actuators.

```python
class MyRobotBackend:
    def read_state(self) -> dict:
        # Read from your robot's SDK
        return {"j1": {"position": 0.0, "velocity": 0.0, "torque": 0.0}, ...}

    def write_commands(self, commands: dict) -> None:
        # Send commands to your robot's SDK
        ...
```

Pass your backend to the `HardwareSynchronizer` during orchestrator construction.

## Connecting LiDAR Sensors

The perception subsystem supports two ingestion modes for LiDAR data.

**ROS2 Mode** subscribes to the `/ouster/points` topic (or any topic publishing `sensor_msgs/PointCloud2`). This is the recommended mode for production deployments where you have a ROS2 driver running for your LiDAR.

**Direct SDK Mode** uses the Ouster Python SDK (`ouster-sdk`) to connect directly to the sensor over the network. This mode is useful for lightweight testing without a full ROS2 installation.

Both modes produce `LidarFrame` dataclasses containing the timestamp, frame ID, 3D point cloud, optional intensities, and metadata.

## Connecting Cameras

Camera ingestion follows the same dual-mode pattern.

**ROS2 Mode** subscribes to `/camera/image_raw` (or any topic publishing `sensor_msgs/Image`).

**Direct SDK Mode** uses OpenCV (`cv2.VideoCapture`) for USB cameras or `pyrealsense2` for Intel RealSense depth cameras.

Both modes produce `CameraFrame` dataclasses containing the timestamp, frame ID, image data, intrinsics, and metadata.

## Calibration

Before running with real sensors, you need to provide calibration files in the `calibration/` directory.

**camera_intrinsics.yaml** contains the camera's intrinsic matrix (K) and distortion coefficients. Use OpenCV's camera calibration tools or the manufacturer's calibration utility.

**extrinsics.yaml** contains the rigid-body transforms between the LiDAR, camera, and robot base frames. These transforms are essential for projecting LiDAR points into the camera image and for fusing sensor data in a common coordinate frame.

## Deployment

The project includes a `Dockerfile` and `docker-compose.yml` for containerised deployment. The Docker image includes all Python dependencies and runs the control loop as the default entrypoint.

For production deployments, we recommend running the control stack in a Docker container with the `--privileged` flag to access USB devices and network interfaces for direct sensor communication.

```bash
docker compose up -d
```

For ROS2 deployments, mount the ROS2 workspace into the container and source the setup script before starting the control loop.

## Environment Variables

All configuration is loaded from environment variables via `robot_hw/robot_config.py`. See `.env.example` for the full list of available settings. Key variables include `ROBOT_CYCLE_TIME` (control loop period in seconds), `ROBOT_BATTERY_CAPACITY_WH` (battery capacity), and `ROBOT_LIDAR_ENABLED` / `ROBOT_CAMERA_ENABLED` (sensor enable flags).
