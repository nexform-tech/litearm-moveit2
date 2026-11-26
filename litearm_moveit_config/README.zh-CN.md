[English](README.md) | 简体中文

# litearm_moveit_config

LiteArm 七轴机械臂的 MoveIt 2 配置：SRDF、KDL 运动学、关节限位、OMPL 规划
流水线、控制器映射和 launch 文件。它规划的是不含夹爪的臂模型，并附带真机
眼在手外 D435i 避障栈。要在真机 LiteArm 上规划，或需要真机栈使用的
手眼标定时，读这个包。

## 1. 编译

在工作区根目录编译：

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

**不要加 `--symlink-install`。** `litearm-cpp` SDK 以 `EXCLUDE_FROM_ALL`
被拉进 `litearm_ros2_control`，它的 CLI 不会编译，但 install 规则仍会执行，
symlink install 会失败。

## 2. 手眼标定

真机栈使用眼在手外的 D435i 标定。`config/handeye/` 里的文件是**某一台
相机、某一个安装位置**的参考测量，不是通用标定。

| 文件 | 用途 |
| --- | --- |
| `handeye_eye_to_hand_TSAI_20261126_102300_fixed.json` | 针对 D435i 序列号 243222074636 和 2026-11-26 安装位置的修正 TSAI 结果。launch 默认使用它。只允许在同一物理工位、做完下面的验证后复用。 |
| `handeye_eye_to_hand_TSAI_20261126_102300.json` | 原始采集，留作追溯。它给 1280x720 彩色图用了 `fx=615.2`（深度/IR）内参，平移差约 15 cm。**不要用它驱动机器人。** |

**不要把包里的标定复用到另一台机器人、另一台相机或另一个安装位置。**
手眼变换是相机和机器人基座之间的物理测量。基座位置变了、换了 D435i
（彩色内参是逐台的）、安装支架变了，或者相机被碰过，文件就失效。需要
重新标定，再做验证。

### 重新标定

1. 相机刚性固定，标定后不要移动。移动相机或机器人基座都会让结果失效。
2. 用 6x6 ChArUco 板（方格 30 mm、标记 22.5 mm）刚性固定在机器人法兰上，
   采集至少 16 个眼在手外位姿。使用 1280x720 彩色图像，记录每个位姿的
   `T_base_flange`。
3. 从 `/camera/color/camera_info` 读取实际的 D435i 彩色内参。1280x720 下
   彩色 `fx` 约为 910，不是 615。在 HandEye Studio 里用彩色内参跑眼在手外
   TSAI 或 PARK，检查重投影误差和残差，保存修正后的 JSON。
4. 保持约定：`T_base_cam` 把相机坐标系变换到 `base_link`。相机坐标系是
   彩色光学帧（x 右、y 下、z 前）。`camera_calibration_tf.py` 会把它和驱动
   发布的 `camera_link -> camera_color_optical_frame` 出厂变换复合，然后
   发布 `base_link -> camera_link`。
5. 把新文件传给 launch：

   ```bash
   ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py \
     calibration_file:=$HOME/litearm_ws/handeye.json
   ```

   除非标定板是通过深度/IR 流观测的，否则保持
   `calibration_frame:=camera_color_optical_frame`。接了多台 RealSense 时
   用 `camera_serial:=...` 指定。

### 验证标定

1. 启动栈并打开 RViz：

   ```bash
   ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py \
     start_supervisor:=false
   ```

2. 打开原始点云，和机器人模型对比。手臂和底座必须重合；差几厘米就是
   标定缺陷。
3. 检查 `/camera/depth/color/points_filtered`。手臂表面应该被滤掉；如果
   还有手臂点，说明自滤波或外参有问题。
4. 看 MotionPlanning > Scene Geometry。Octomap 不能长到手臂、底座或
   台面接触区域上。如果出现，垫高底座或裁掉台面那一块。
5. 在急停可触达的情况下先发一个小目标，确认后再做大幅度运动。

## 3. 真机 D435i 避障

在工作区根目录、已 source `install/setup.bash` 的环境里：

```bash
ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py
```

默认启动直连 USB 控制栈、move_group、D435i、标定 TF、supervisor、目标
marker 和 RViz。机械臂模型不含夹爪。launch 不启动 Gazebo，也不使用
仿真时间。

控制栈已经在别处运行时加 `start_control:=false`；相机驱动在别处运行时加
`start_camera:=false`；只看感知时加 `start_supervisor:=false`。

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `start_control` | `true` | 启动直连 USB 控制栈 |
| `start_camera` | `true` | 启动 `realsense2_camera_node` |
| `camera_serial` | 空 | 按序列号选择 RealSense |
| `start_supervisor` | `true` | 规划、执行并复检剩余轨迹 |
| `start_goal_marker` | `true` | 启动 RViz 6-DOF 目标 marker |
| `retreat_enabled` | `false` | 重规划预算用完后原地停；`true` 则退回 ready |
| `max_replans` | `20` | 被挡恢复次数；每次恢复开始执行后归零 |
| `max_no_path_retries` | `3` | 连续多少次返回空轨迹才判定“无路径” |
| `replan_budget_reset_on_success` | `true` | `false` = 每个目标硬上限 `max_replans` |
| `velocity_scaling` | `0.08` | supervisor 速度；首轮保持 0.05 到 0.1 |
| `calibration_file` | `_fixed.json` 文件 | 手眼标定 JSON |
| `calibration_frame` | `camera_color_optical_frame` | 手眼 JSON 对应的相机帧 |
| `shape_tf_cache_wait` | `0.5` | ShapeMask 查 TF 的等待时间（秒） |
| `allow_move_group_execution` | `false` | 是否允许 move_group/RViz Execute 直接执行；真机默认只走 supervisor |
| `scene_file` | 空 | 可选的静态场景 YAML |
| `ros_domain_id` | `42` | 所有节点使用的 ROS 域 |
| `ros_localhost_only` | `true` | 只在本机发现 |

真机 launch 默认把传感器心跳设为深度和彩色两路 CameraInfo
（`sensor_topics`），`max_state_age=0.3`、`max_sensor_age=0.5`、
`max_octomap_age=1.5`；任一数据源或 Octomap 过期都会拒绝运动。规划显式
使用当前关节值，并拒绝被 `FixStartStateCollision` 挪动过的起点；执行中
实际跟踪误差超过 `max_tracking_error=0.1` rad 直接 `safe_stop`。
如果现场出现 `stale_octomap` 误报，先看 `/d435i_supervisor/status` 里的
`octomap_age` 和 `/monitored_planning_scene` 的实际频率，再调整
`max_octomap_age`；设为 0 可关闭该检查。
`allow_move_group_execution=false` 时 RViz Execute 和
`moveit_probe.py --execute` 都不能绕过 supervisor；需要手动调试时显式
加 `allow_move_group_execution:=true`。

传感器调参在 `config/sensors_3d_real.yaml`。`padding_offset` 是自滤波的
膨胀量，不是规划安全裕度。外参准确时首轮 `0.02` m 可用；手臂表面漏进
Octomap 时提到 `0.03`。规划裕度靠 `octomap_resolution`、静态场景和低速。

supervisor 在场景更新后规划、执行并复检剩余轨迹。被挡住的点会取消 JTC
目标，从当前状态重规划。两个预算决定什么时候放弃：`max_replans`（20）
统计“被挡恢复”的次数，每次恢复的轨迹开始执行后归零，所以它限制的是
**连续恢复失败**，而不是整趟任务的总次数；`max_no_path_retries`（3）统计
连续返回空轨迹的次数，只有它才会以“无路径”结束重规划。要按目标做硬上限，
设 `replan_budget_reset_on_success:=false`。默认 `retreat_enabled=false`，
放弃时原地 `safe_stop`；加 `retreat_enabled:=true` 则以 0.05 缩放退回 ready。
`ignore_base_link_contacts` 不会把固定底座/台面的安装接触当成运动障碍。

## 4. 目标 marker 和命令行工具

RViz marker 放在当前 `ee_link`。拖动蓝色方块平移，拖动 x/y/z 三个圆环
旋转。拖动时节点以 10 Hz 求 IK，并把一条幽灵臂发到
`/display_planned_path`；幽灵臂只是预览，不会动机器人。右键 marker 菜单：

- Send goal：把位姿发到 `/d435i_supervisor/goal_pose`；
- Home：把七个关节的零位发到 `/d435i_supervisor/goal_joints`；
- Preview：只做 IK 和 OMPL，把轨迹发出来但不执行；
- Reset：把 marker 放回当前 `ee_link`；
- Cancel：调用 `/d435i_supervisor/cancel`。

Home 菜单发送的是零位。想回 ready，把 launch 里的 `home_positions` 改成
`[0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]`。

命令行工具打印当前位姿和关节角，也可以发位姿、关节目标或零位：

```bash
ros2 run litearm_moveit_config goal_tool.py
ros2 run litearm_moveit_config goal_tool.py --home
ros2 run litearm_moveit_config goal_tool.py \
  --pose 0.25 0.0 0.25 3.14159 0.0 0.0
ros2 run litearm_moveit_config goal_tool.py \
  --joints 0 0 0 -1.5708 0 0 0
```

执行请走 supervisor 的入口。真机 launch 默认
`allow_move_group_execution=false`，RViz MotionPlanning 的 **Execute**
按钮和 `/move_action` 都不能直接驱动机器人；那里只用 **Plan** 看路径。
确实需要手动执行做 bring-up 时，显式加 `allow_move_group_execution:=true`。

## 5. 安全和已知限制

- 这不是安全认证的避障栈。物理急停必须在手边；固件 watchdog 和控制器
  容差是最后一道保护。
- supervisor 是停—重规划，不是 MoveIt Servo。场景变化到新轨迹之间可能
  有几百毫秒。
- 单台固定相机看不到手臂背面和被物体挡住的部分。自滤波只去掉机器人，
  不会消除遮挡；相机看不到的地方用静态场景和保守地图兜底。
- 手眼误差几厘米就会让手臂漏进 Octomap。要检查点云和模型是否重合；
  相机碰过之后必须重新标定。
- MoveIt 2.5.10 的 `move_group` 在关闭时可能崩溃；不影响关闭前的运行。
- 控制栈退出时机械臂由固件保持；如果重力不是唯一负载，先支撑住。
