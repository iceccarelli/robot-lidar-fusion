# Calibration Files for LiDAR and Camera

This directory contains calibration artefacts required to align the
external sensors (LiDAR and camera) with the robot’s coordinate
frames.  Accurate calibration is critical for sensor fusion and
navigation.  The files provided here serve as templates; you should
replace the example values with measurements from your actual setup.

## Camera Intrinsics

The file `camera_intrinsics.yaml` stores the intrinsic parameters
for the camera.  It follows the ROS `sensor_msgs/CameraInfo` format.
Important fields include:

* `width` and `height`: image resolution.
* `K`: 3×3 intrinsic matrix `[fx, 0, cx, 0, fy, cy, 0, 0, 1]`.
* `D`: distortion coefficients `[k1, k2, p1, p2, k3]` for a plumb–bob
  model.
* `distortion_model`: typically `plumb_bob` for pinhole cameras.

Adjust these values based on a calibration target (e.g. a chessboard)
and use standard tools such as `camera_calibration` to generate
accurate intrinsics.

## Extrinsics

The file `extrinsics.yaml` defines static transforms between the
robot’s frames (e.g. `base_link`) and the sensor frames (`os_sensor`,
`camera_link`).  Each transform specifies a translation and rotation
quaternion.  At runtime these transforms are broadcast as static
TFs via ROS2 or consumed by the perception pipeline.

Example fields:

```yaml
transforms:
  - parent_frame: base_link
    child_frame: os_sensor
    translation: [0.0, 0.0, 1.2]        # metres
    rotation: [0.0, 0.0, 0.0, 1.0]      # quaternion (x, y, z, w)
  - parent_frame: base_link
    child_frame: camera_link
    translation: [0.0, 0.0, 1.25]
    rotation: [0.0, 0.0, 0.0, 1.0]
```

### Collecting Calibration Data

1. **Camera Intrinsics**: Place a checkerboard pattern in view of the
   camera and use the ROS `camera_calibration` tool or OpenCV’s
   calibration functions to compute `fx`, `fy`, `cx`, `cy` and
   distortion coefficients.  Save the results into
   `camera_intrinsics.yaml`.

2. **Extrinsics between LiDAR and Camera**: Capture a scene visible
   to both sensors.  Use tools such as `Kalibr` or `lidar_camera_calibration` to
   compute the rigid transform between the LiDAR and camera frames.
   Ensure the base frame (`base_link`) is defined consistently in
   your robot’s URDF.  Update `extrinsics.yaml` accordingly.

3. **Verification**: After updating the files, run the demo script
   (`demo_os1_camera_live.py`) to check timestamp alignment and
   overlay LiDAR points onto the camera image.  Misalignment
   indicates calibration errors.