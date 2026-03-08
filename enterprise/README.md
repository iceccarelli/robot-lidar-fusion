# Enterprise Extensions

This directory is reserved for enterprise-grade extensions that build on top of the open-source robot control stack. While the core framework provides everything you need to get started, production deployments in regulated industries often require additional capabilities that go beyond what a community project can maintain.

## Planned Enterprise Modules

### Certified Robot Connectors

Production-hardened drivers for industrial robot platforms, each validated against the manufacturer's safety certification requirements.

| Connector | Platform | Status |
|:---|:---|:---|
| KUKA KR C5 | KUKA industrial arms | Planned |
| ABB OmniCore | ABB collaborative and industrial robots | Planned |
| Fanuc R-30iB | Fanuc CNC and robotic systems | Planned |
| Universal Robots e-Series | UR3e, UR5e, UR10e, UR16e, UR20, UR30 | Planned |
| Boston Dynamics Spot | Spot quadruped platform | Planned |
| Agility Robotics Digit | Humanoid logistics platform | Planned |

### Advanced Algorithms

High-performance algorithms optimised for real-time operation on industrial hardware.

| Algorithm | Description | Status |
|:---|:---|:---|
| EKF/UKF Fusion | Extended and Unscented Kalman Filters for multi-sensor state estimation | Planned |
| RRT* Path Planning | Sampling-based motion planning with asymptotic optimality | Planned |
| MPC Locomotion | Model Predictive Control for dynamic walking and balancing | Planned |
| SLAM Integration | Simultaneous Localisation and Mapping with LiDAR and visual odometry | Planned |
| Reinforcement Learning | Learned locomotion policies for unstructured terrain | Planned |

### Compliance and Safety

Modules that help meet regulatory requirements for deploying autonomous robots in industrial environments.

| Module | Standard | Status |
|:---|:---|:---|
| ISO 10218 Compliance | Industrial robot safety | Planned |
| ISO 13849 PL-d | Safety-related control systems | Planned |
| ISO 15066 | Collaborative robot operation | Planned |
| CE Marking Support | European conformity documentation | Planned |

## How to Access

Enterprise extensions will be available under a separate licence. If you are interested in early access or have specific requirements, please open an issue on the main repository or reach out directly.

## Contributing

If you have experience with any of the platforms or algorithms listed above and would like to contribute, we welcome your involvement. Please see the main [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines.
