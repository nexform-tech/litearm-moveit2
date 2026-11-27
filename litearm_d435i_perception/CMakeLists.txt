cmake_minimum_required(VERSION 3.10)
project(litearm_d435i_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# 可切换的机械臂 3cm 包络点云过滤（现有 MoveIt padding 2cm + 额外 1cm）。
add_executable(cloud_envelope_filter src/cloud_envelope_filter.cpp)
target_link_libraries(cloud_envelope_filter
  moveit_ros_perception::moveit_point_containment_filter
  moveit_ros_planning::moveit_robot_model_loader
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  Eigen3::Eigen
)
ament_target_dependencies(cloud_envelope_filter
  rclcpp sensor_msgs tf2 tf2_ros visualization_msgs interactive_markers
  moveit_core moveit_ros_planning moveit_ros_perception
)
install(TARGETS cloud_envelope_filter
  DESTINATION lib/${PROJECT_NAME}
)

# 纯配置 + Python 演示包：模型、launch、Octomap 配置、RViz 布局与节点脚本。
install(DIRECTORY config launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_tf.py
  scripts/spawn_model.py
  scripts/gazebo_robot_description.py
  scripts/movable_block.py
  scripts/move_block.py
  scripts/dynamic_obstacle.py
  scripts/d435i_supervisor.py
  scripts/d435i_probe.py
  scripts/wrist_scan.py
  scripts/wrist_runtime_probe.py
  scripts/acceptance_d435i.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
