"""Robot Hardware Control Stack with LiDAR-Camera Fusion.

This package provides a modular, hardware-agnostic control stack for
humanoid and mobile robots.  It integrates LiDAR point-cloud processing,
camera frame ingestion, sensor fusion, hazard detection, locomotion
control, mission planning and power management into a single
deterministic control loop.

Subpackages
-----------
core
    Foundational services: hardware synchronisation, memory management,
    fault detection, hazard monitoring, concurrency and communication.
control
    Joint synchronisation and locomotion controllers.
perception
    Sensor I/O (ROS2 and direct SDK), LiDAR utilities, time
    synchronisation and sensor fusion.
planning
    Mission planning, navigation and task-to-hardware mapping.
power
    Battery state-of-charge tracking and thermal management.
ai
    Predictive state estimation and trajectory forecasting.
"""

__version__ = "0.1.0"
