[English](README.md) | 简体中文

`litearm_obstacle_avoidance` 让 LiteArm 七轴机械臂在装上 LiteGrip 夹爪后进行规划，
使夹爪、工位与周围障碍物都参与 MoveIt 碰撞检查。它面向希望在真机运行前验证避障行为的
工程师：包内提供基于假硬件的干跑栈、Gazebo Classic 仿真栈、数据驱动的静态场景、
执行监督节点，以及覆盖四个阶段的验收探针。它本身不实现规划；运动学、关节限位与
OMPL 规划管线来自 `litearm_moveit_config`。

以下命令都在工作区根目录 `~/litearm_ws` 下执行。

## 环境要求

| 项目 | 要求 |
| --- | --- |
| 系统 | Ubuntu 22.04 (jammy)，x86_64 |
| ROS 2 | Humble Hawksbill |
| 工作区内的包 | `litearm_moveit_config`、`litearm_ros2_control`、`litearm_description`、`litegrip_description`、`litearm-cpp` |
| 机械臂 | 干跑栈不需要 |
| 显示 | 可选；RViz 需要，探针不需要 |
| Gazebo | Classic 与 `gazebo_ros2_control`，仅 `obstacle_gazebo.launch.py` 需要 |

## 1. 编译工作区

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

**不要**加 `--symlink-install`。工作区通过 `EXCLUDE_FROM_ALL` 把 `litearm-cpp` SDK
引入 `litearm_ros2_control`，SDK 的 CLI 可执行文件因此不会被编译，但它的安装规则仍然
执行；符号链接安装随后会报
`ament_cmake_symlink_install_targets() can't find`。普通 `colcon build` 会编译工作区
内的全部包。

## 2. 运行干跑栈

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py
```

这条命令启动假硬件控制栈、MoveIt、静态场景和 RViz。等到日志出现
`Configured and activated joint_trajectory_controller` 后再下发目标：action server
要等 spawner 激活控制器后才存在。

在 RViz 里用 **MotionPlanning** 面板设置目标，然后规划或执行。无显示环境可以加
`use_rviz:=false`，再用探针驱动这套栈：

```bash
ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py use_rviz:=false
```

另开一个终端：

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
python3 "$(ros2 pkg prefix litearm_moveit_config)/lib/litearm_moveit_config/moveit_probe.py" --execute
```

`obstacle_fake.launch.py` 支持的参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `use_rviz` | `true` | 启动带 MoveIt 布局的 RViz |
| `use_synthetic_cloud` | `false` | 发布 `/synthetic_cloud` 并启用 Octomap |
| `scene_file` | `config/scene.yaml` | 静态工位；传 `''` 不加载任何世界物体 |
| `start_supervisor` | `false` | 启动 `avoid_supervisor.py` |
| `gripper_envelope_margin` | `0.01` | 手指包络额外外扩量，单位米 |
| `ros_domain_id` | `42` | 所有节点使用的 ROS domain |
| `ros_localhost_only` | `true` | 只在本机发现 |

## 3. 运行 Gazebo 仿真

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litearm_obstacle_avoidance obstacle_gazebo.launch.py
```

无显示环境：

```bash
ros2 launch litearm_obstacle_avoidance obstacle_gazebo.launch.py \
    gui:=false use_rviz:=false start_supervisor:=true
```

Gazebo 栈把 `mock_components` 换成 Gazebo Classic 物理引擎。控制器名称、JTC action
与探针接口都保持不变，因此 `acceptance_fake.sh` 里的探针可以直接复用它。

`worlds/obstacle_avoidance.world` 里的世界几何必须与 `config/scene.yaml` 一致。
在一个文件里改了夹具位姿，就要同步改另一个；否则 Gazebo 与 MoveIt 对障碍物位置的
判断会不一致。

## Launch 文件

| Launch 文件 | 启动内容 | 适用场景 |
| --- | --- | --- |
| `fake_control.launch.py` | `robot_state_publisher`、基于 `mock_components/GenericSystem` 的 `ros2_control_node`、joint state broadcaster、joint trajectory controller | 只需要执行链；不含 MoveIt 与 RViz |
| `obstacle_demo.launch.py` | `move_group` 与可选的 RViz，使用机械臂加夹爪的组合模型 | 只做规划，或控制栈已在别处运行 |
| `obstacle_fake.launch.py` | `fake_control.launch.py` 加 `obstacle_demo.launch.py`；可选合成点云与监督节点 | 完整干跑：规划、执行、重规划，不动真机 |
| `obstacle_gazebo.launch.py` | Gazebo Classic、`robot_state_publisher`、`spawn_robot.py`、控制器 spawner、`move_group`、可选 RViz | 物理仿真 |

`obstacle_fake.launch.py` 会包含另外两个 launch 文件，因此假硬件与 MoveIt 共用同一份
机器人描述、同一份 SRDF 和同一套规划参数。`obstacle_gazebo.launch.py` 是独立的，
因为 Gazebo 模型额外加了固定的 `world` link，MoveIt 不能重复看到它。

## 验收探针

包内提供四个分阶段探针。每个探针都针对正在运行的栈，只有全部检查通过才返回 `0`。

| 探针 | 阶段 | 检查内容 |
| --- | --- | --- |
| `scene_probe.py` | 1 | 场景对象已进入 PlanningScene；规划轨迹的每个点都通过碰撞检查；用盒子包住目标后规划必须失败 |
| `margin_probe.py` | 2 | 手指包络覆盖夹爪张开行程；在包络外缘之外放小方块仍会碰撞，证明 margin 已生效 |
| `replan_probe.py` | 3 | 执行中注入障碍会触发取消与重规划；目标被挡住时触发低速撤退或安全停靠 |
| `safety_probe.py` | 4 | `/joint_states` 过期时监督节点拒绝新运动，且不发布任何轨迹 |

辅助程序：

| 程序 | 作用 |
| --- | --- |
| `scene_builder.py` | 读取 `scene.yaml`，通过 `/apply_planning_scene` 把对象写进 PlanningScene |
| `synthetic_cloud.py` | 没有相机时在 `/synthetic_cloud` 发布长方体表面点云 |
| `avoid_supervisor.py` | 规划、执行、取消与重规划；失败时撤退或停住 |
| `spawn_robot.py` | 把机器人插入 Gazebo，避开 `gazebo_ros/spawn_entity.py` 在 conda 环境下的 `lxml` 问题 |
| `acceptance_fake.sh` | 启动干跑栈、等待就绪、运行选定探针、收栈 |

一条命令跑完四个阶段：

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run litearm_obstacle_avoidance acceptance_fake.sh --scene --margin --replan --safety
```

| 参数 | 增加的内容 |
| --- | --- |
| 无 | 只规划并执行到 ready 位形 |
| `--cloud` | 合成点云与 Octomap 就绪检查 |
| `--scene` | 阶段 1 场景探针 |
| `--margin` | 阶段 2 裕度探针 |
| `--replan` | 阶段 3 重规划探针；会启动监督节点 |
| `--safety` | 阶段 4 数据过期探针；会启动监督节点 |

**不要**在另一个栈占用同一 ROS domain 时运行 `acceptance_fake.sh`。脚本会启动自己的
栈并在退出时收栈；同一个 domain 里出现两套 `/controller_manager` 或 `/move_group`
会让探针报出假失败。脚本日志在 `/tmp/litearm_acceptance_fake.log`。

## 静态场景

`config/scene.yaml` 用固定的 `base_link` 坐标系描述工位。它是数据，不是代码：
改这个文件就能改工位，然后重启 launch 或重新运行 `scene_builder.py`。

| 对象 | 形状 | 尺寸 | `base_link` 下的位姿 |
| --- | --- | --- | --- |
| `workbench` | box | 0.90 x 0.90 x 0.10 m | (0.10, 0.0, -0.07) |
| `fixture_pillar` | cylinder | 半径 0.04 m，高 0.28 m | (0.30, -0.25, 0.12) |
| `fixture_block` | box | 0.12 x 0.12 x 0.12 m | (-0.30, 0.15, 0.04) |

每个对象都把 `padding` 设为 0.01 m。该字段会放大图元，因为 Humble 的
`moveit_msgs/CollisionObject` 没有 padding 字段。工作台顶面在
z = -0.02 m，加 padding 后为 z = -0.01 m，与底座下表面留出 1 cm，避免起始状态
被判为碰撞。

这是测试工位，不是实测的生产工位。对真实工位下结论前，请把尺寸与位姿替换成实测值。
如果夹具挡住了验收目标，规划会（正确地）失败。

## 夹爪模型与包络

组合模型由三个入口文件构建：

| 文件 | 硬件块 |
| --- | --- |
| `config/litearm_litegrip.urdf.xacro` | 真机，通过 `litearm_ros2_control` |
| `config/litearm_litegrip_fake.urdf.xacro` | 干跑用的 `mock_components/GenericSystem` |
| `config/litearm_litegrip_gazebo.urdf.xacro` | `gazebo_ros2_control/GazeboSystem`，并追加固定的 `world` link |

三个文件都包含 `config/litearm_litegrip.geometry.urdf.xacro`，因此机械臂几何、夹爪
安装变换与包络只维护一份。夹爪 link 统一使用 `litegrip_` 前缀，避免与机械臂的
`base_link` 重名。

默认安装偏移是相对 `ee_link` 的 0.0538 m。它假定 10.3 mm 端盖已拆掉，夹爪尾部凸台
占据该空间。如果你的硬件保留端盖或使用转接板，覆盖这个值：

```bash
xacro config/litearm_litegrip.geometry.urdf.xacro \
    gripper_mount_xyz:="0 0 0.0641" gripper_mount_rpy:="0 0 0"
```

这套栈里夹爪没有状态来源。MoveIt 因此把 `gripper_opening_joint` 当作 0，即闭合位，
而物理夹爪可能处于张开状态。`config/litearm_litegrip.geometry.urdf.xacro` 用只参与
碰撞的包络来补偿：包络覆盖手指全行程，并额外扩出 `gripper_envelope_margin`，默认
0.01 m。`config/litearm_litegrip.srdf` 禁用了结构接触对，以及包络按设计包住的手指对。

**不要**在不修改 SRDF 的情况下传 `include_gripper_envelope:=false`。SRDF 的
`disable_collisions` 引用了包络 link；MoveIt 会拒绝引用不存在的 link。

规划组有两个：`litearm_arm` 从 `base_link` 到 `ee_link`，`gripper` 只含主动关节
`gripper_opening_joint`。

## 执行监督

`avoid_supervisor.py` 执行「停、重规划、继续」的循环。它不是 MoveIt Servo：它会取消
当前轨迹，从当前状态重新规划，然后继续。重规划失败时，它以低速撤退到 ready 位形；
撤退也失败就停在原地。

| 接口 | 类型 | 方向 |
| --- | --- | --- |
| `/avoid_supervisor/goal` | `sensor_msgs/JointState` | 输入；七个臂关节 |
| `/avoid_supervisor/status` | `std_msgs/String` JSON | 输出；`state`、`replans`、`busy`、`reason` |
| `/avoid_supervisor/planned_trajectory` | `trajectory_msgs/JointTrajectory` | 输出；每次新规划 |
| `/plan_kinematic_path` | `moveit_msgs/GetMotionPlan` | service 客户端 |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | action 客户端 |

状态机依次经过 `idle`、`planning`、`executing`、`succeeded`；场景变化时进入
`canceling` 与 `replanning`；出错路径进入 `retreating`、`retreated`、`safe_stop` 或
`failed`。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `group_name` | `litearm_arm` | 规划组 |
| `velocity_scaling` | `0.15` | 正常执行的速度缩放 |
| `acceleration_scaling` | `0.15` | 正常执行的加速度缩放 |
| `planning_time` | `2.0` s | 每次尝试的规划预算 |
| `plan_attempts` | `5` | 每次规划请求的尝试次数 |
| `max_replans` | `3` | 放弃前的最大重规划次数 |
| `goal_tolerance` | `0.06` rad | 收敛容差 |
| `max_state_age` | `1.0` s | `/joint_states` 超过该时长即拒绝运动 |
| `sensor_topic` | 空 | 可选的传感器时效检查话题 |
| `max_sensor_age` | `0.0` s | 0 表示关闭传感器时效检查 |
| `retreat_enabled` | `true` | 允许失败后低速撤退 |
| `retreat_positions` | ready 位形 | 撤退的七个关节值 |
| `retreat_velocity_scaling` | `0.1` | 撤退速度缩放 |

随干跑栈启动：

```bash
ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py start_supervisor:=true
```

## 环境隔离

所有 launch 文件都为节点设置 `ROS_DOMAIN_ID=42` 与 `ROS_LOCALHOST_ONLY=1`。这是刻意
的：让测试栈远离 domain 0 和网络上的其它机器人。用 `ros_domain_id:=` 或
`ros_localhost_only:=` 可以改变它。

在与这套栈通信的终端里，运行任何 `ros2` 命令前先设置同样的值：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

`acceptance_fake.sh` 使用相同的默认值，并把它们传给 launch 文件。

## 已知限制

- 干跑栈没有动力学。状态等于指令，因此只能验证执行链，不能验证控制品质。真实行为
  必须在机械臂上验证。
- Gazebo Classic 不支持 URDF mimic。两个夹爪手指在仿真里是被动关节；避障依赖
  MoveIt 侧的包络。
- 夹爪没有状态来源。碰撞检查使用保守包络，而不是实际开口。
- `scene.yaml` 是测试工位，没有按生产工位标定。
- 监督节点的语义是停、重规划、继续，不是实时避障控制器。
- 在 MoveIt 2.5.10 上，`move_group` 收栈时可能崩溃。这是上游问题，不影响收栈前的
  检查结果。
- 第一条合成点云可能打出一条
  `shape_mask: Missing transform for shape mesh`。此时自滤波的变换还没就绪；该消息
  不会重复出现。
