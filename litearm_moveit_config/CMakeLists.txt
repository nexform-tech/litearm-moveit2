cmake_minimum_required(VERSION 3.16)
project(litearm_moveit_config)

find_package(ament_cmake REQUIRED)

# This package is pure configuration (SRDF / yaml / launch / rviz); it produces
# no build artifacts.
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# On-site acceptance probes (require a running move_group)
install(PROGRAMS
  scripts/moveit_probe.py
  scripts/acceptance_moveit.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Tool scripts: collision matrix generation, deriving joint limits from the
# firmware parameter tables. Installing them under lib/${PROJECT_NAME} is what
# makes the invocation paths given in the messages valid.
install(PROGRAMS
  tools/compute_collision_matrix.py
  tools/gen_from_firmware_config.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
