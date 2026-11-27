cmake_minimum_required(VERSION 3.16)
project(litearm_dual_arm)

find_package(ament_cmake REQUIRED)

# 纯配置 + Python 脚本包：模型生成、launch、RViz、验收探针。
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_dual_arm_model.py
  scripts/dual_arm_sim.py
  scripts/dual_arm_bridge.py
  scripts/dual_arm_scene.py
  scripts/dual_arm_probe.py
  scripts/dual_arm_preflight.py
  scripts/dual_arm_task_client.py
  scripts/dual_carry_demo.py
  scripts/dual_alternating_demo.py
  scripts/acceptance_dual_arm.sh
  scripts/acceptance_carry.sh
  scripts/acceptance_alternating.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 本机的默认 python3 可能来自 conda（3.11），而 Humble 的 rclpy/rclpy_pybind11
  # 是 3.10 ABI；用 conda 解释器跑 pytest 会在 import rclpy 时失败。优先用
  # /usr/bin 下的系统 Python 3.10，找不到时再退回 CMake 找到的解释器。
  find_program(LITEARM_PYTEST_PYTHON NAMES python3.10 python3
    PATHS /usr/bin NO_DEFAULT_PATH)
  if(NOT LITEARM_PYTEST_PYTHON)
    set(LITEARM_PYTEST_PYTHON Python3::Interpreter)
  endif()
  # 本机 pytest 9.x 与 ROS Humble 的 launch_testing 入口点不兼容（hook 签名
  # 变更）。这里只跑纯 Python 测试，不需要自动加载 ROS 插件，所以显式关掉
  # entry-point 自动加载，避免 colcon test 被 launch_testing 的旧 hook 卡住。
  ament_add_pytest_test(test_model test/test_model.py
    PYTHON_EXECUTABLE ${LITEARM_PYTEST_PYTHON}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_configs test/test_configs.py
    PYTHON_EXECUTABLE ${LITEARM_PYTEST_PYTHON}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_launch test/test_launch.py
    PYTHON_EXECUTABLE ${LITEARM_PYTEST_PYTHON}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_bridge test/test_bridge.py
    PYTHON_EXECUTABLE ${LITEARM_PYTEST_PYTHON}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_real_launch test/test_real_launch.py
    PYTHON_EXECUTABLE ${LITEARM_PYTEST_PYTHON}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_carry test/test_carry.py
    PYTHON_EXECUTABLE ${LITEARM_PYTEST_PYTHON}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_alternating test/test_alternating.py
    PYTHON_EXECUTABLE ${LITEARM_PYTEST_PYTHON}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
