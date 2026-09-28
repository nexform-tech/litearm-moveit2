# litearm-moveit2

MoveIt 2 integration on ROS 2 for the **LiteArm robotic manipulator series**.

## Packages

| Package | Type | Role |
| --- | --- | --- |
| `litearm_moveit_config` | ament_cmake | MoveIt 2 configuration for the LiteArm 7-axis arm: SRDF, joint limits, kinematics, OMPL and Pilz planning pipelines, MoveIt controllers and an RViz layout |

## Scope

| | |
| --- | --- |
| Product | LiteArm robotic manipulator series |
| Repository role | MoveIt 2 integration on ROS 2 |
| Status | Active — MoveIt 2 configuration package landed |

## Related repositories

| Repository | Role |
| --- | --- |
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK |
| [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) | C++ SDK |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | Product documentation |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | ROS 2 driver |

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and
automated semantic-release versioning on every merge to `main`.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).
