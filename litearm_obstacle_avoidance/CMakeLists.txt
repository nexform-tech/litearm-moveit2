cmake_minimum_required(VERSION 3.10)
project(litearm_obstacle_avoidance)

find_package(ament_cmake REQUIRED)

# 本包是纯配置包（组合 URDF/SRDF / launch / yaml），不含编译产物。
install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# 静态环境、测试夹具与验收：
#   scene_builder.py   把 scene.yaml 写进 PlanningScene
#   synthetic_cloud.py 无相机时打通 Octomap 链路
#   scene_probe.py     阶段 1 场景验收
#   gazebo_robot_description.py  只给 gazebo_ros2_control 提供 Gazebo 侧描述
#   acceptance_fake.sh 一键干跑验收
install(PROGRAMS
  scripts/scene_builder.py
  scripts/scene_probe.py
  scripts/margin_probe.py
  scripts/avoid_supervisor.py
  scripts/gazebo_robot_description.py
  scripts/replan_probe.py
  scripts/safety_probe.py
  scripts/timing_probe.py
  scripts/spawn_robot.py
  scripts/synthetic_cloud.py
  scripts/acceptance_fake.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
