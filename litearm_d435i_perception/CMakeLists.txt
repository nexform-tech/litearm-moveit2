cmake_minimum_required(VERSION 3.10)
project(litearm_d435i_perception)

find_package(ament_cmake REQUIRED)

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
  scripts/acceptance_d435i.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
