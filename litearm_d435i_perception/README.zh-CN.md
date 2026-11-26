[English](README.md) | 简体中文

`litearm_d435i_perception` 在 Gazebo Classic 中模拟一台固定外置的 Intel
RealSense D435i，用真实深度点云为机械臂 + LiteGrip 夹爪构建环境。点云经
MoveIt 2 的 Octomap 进入 PlanningScene，支持位姿/关节目标规划、执行中障碍
更新后的“停 → 复检 → 重规划”，并默认提供一个可通过 ROS 接口实时改位的
演示物块（另有可选的 Gazebo 动态障碍，默认关闭）。

# litearm_d435i_perception

本包不重复机械臂模型：组合 URDF/SRDF、夹爪包络、Gazebo 控制器、工位 world
都来自 `litearm_obstacle_avoidance`，规划参数来自 `litearm_moveit_config`。

## 数据链路

```text
Gazebo depth camera (libgazebo_ros_camera.so)
  -> /camera/depth/color/points        (frame_id=camera_depth_optical_frame)
  -> PointCloudOctomapUpdater          (config/sensors_3d.yaml)
     · ShapeMask 自滤波：排除落在机器人碰撞体内部的点
     · padding_offset 外扩，降低“擦边”规划
  -> Octomap (frame_id=base_link)
  -> /monitored_planning_scene
  -> OMPL 碰撞检查 / d435i_supervisor 剩余轨迹复检
```

话题名刻意对齐真实 `realsense2_camera`：

| 话题 | 内容 |
| --- | --- |
| `/camera/depth/color/points` | 深度点云（MoveIt 消费） |
| `/camera/depth/color/depth/image_raw` | 深度图 float32（仿真侧插件附带） |
| `/camera/color/image_raw` | 彩色图 |
| `/camera/color/camera_info` | 彩色相机内参 |

## 前置依赖

```bash
sudo apt install ros-humble-gazebo-plugins
```

`libgazebo_ros_camera.so` 在这个包里；没装时 Gazebo 不会出点云，
`d435i_supervisor` 会以 `no_sensor/no_octomap` 拒绝运动。

其余依赖（MoveIt、Gazebo、ros2_control、旧的组合模型包）与
`litearm_obstacle_avoidance` 相同。

## 构建与启动

从工作区根目录构建：

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

有显示环境：

```bash
ros2 launch litearm_d435i_perception d435i_avoidance.launch.py
```

无显示（验收/服务器）：

```bash
ros2 launch litearm_d435i_perception d435i_avoidance.launch.py \
  gui:=false use_rviz:=false
```

默认 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`。**每个新终端**（运行
`ros2 topic`、`ros2 run`、RViz 等）都要先执行：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

否则终端和仿真不在同一个 ROS 域，会表现为
`Waiting for at least 1 matching subscription(s)...` 一直等待，或
`✗ 没有节点订阅 /movable_block/target_pose`——节点其实在跑，只是互相看不见。

关键 launch 参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 启动 gzclient |
| `use_rviz` | `true` | 启动 RViz2（含点云、图像、Octomap 场景） |
| `scene_file` | 空 | 可选加载旧包 `scene.yaml` 静态碰撞对象；默认只用相机 Octomap |
| `camera_mount_file` | `config/camera_mount.yaml` | 相机安装位姿（Gazebo 与 TF 共用） |
| `start_camera` | `true` | 插入 Gazebo D435i；接真机/外部相机时设 `false` |
| `sensor_heartbeat_topic` | `/camera/color/camera_info` | supervisor 判传感器存活用的小消息话题 |
| `start_movable_block` | `true` | 插入可实时改位的演示物块 |
| `movable_block_pose` | `0.30 0.55 -0.06` | 物块初始位置 `x y z`（m，world/base_link）；默认在工位外的地面上，相机可见但不挡机械臂 |
| `movable_block_yaw` | `0.0` | 物块初始绕 Z 轴朝向（rad） |
| `start_dynamic_obstacle` | `false` | 默认不插入动态障碍；`true` 才插入并驱动 `dynamic_obstacle` |
| `dynamic_mode` | `sweep` | `sweep` 往复；`once` 走一遍就停 |
| `dynamic_speed` | `0.20` | m/s |
| `extra_gazebo_args` | 空 | 追加给 gzserver，例如 `--verbose` |

## 性能

这套栈在集显 + 桌面后台较重时容易卡。实测（i5-14450HX、集显，同时开着
VSCode/Firefox）：

| 配置 | 主要 CPU | RTF |
| --- | --- | --- |
| 640×480/15 Hz、subsample=2、Octomap 5 Hz、GUI+RViz | move_group 65–99%、rviz2 53%、gzserver 34%、gzclient 15% | 0.745 |
| 当前默认（424×240、深度 10 Hz/彩色 5 Hz、subsample=2、Octomap 10 Hz/0.02 m、RViz 15 FPS） | 明显下降 | 接近 1.0 |

瓶颈不是规划器本身，而是：

- `PointCloudOctomapUpdater` 的 **ShapeMask 自滤波**：每个点都要对机器人
  碰撞体做包含判断，点云越密、更新越频繁，`move_group` 越吃 CPU；
- **RViz + Gazebo GUI** 的场景/点云/图像渲染共用集显；
- supervisor 之前为了判断传感器存活，订阅并反序列化了整份 `PointCloud2`
  （Python 每帧几百 KB）；现在改订阅 CameraInfo 心跳。真机 launch 同时
  订阅深度和彩色两路（`sensor_topics`），任一停发都会拒绝运动。

如果仍然卡，按顺序试：

1. **只留一个界面**：`gui:=false use_rviz:=true`，或
   `gui:=false use_rviz:=false` 看纯仿真。
2. 在 RViz Displays 里关掉「相机彩色图」；「相机点云」默认已关闭，需要看
   原始点云时再打开（Octomap 在 MotionPlanning 的 Scene Geometry 里）。
3. 继续降 `models/d435i_camera.sdf` 的分辨率/帧率，或把
   `config/sensors_3d.yaml` 的 `point_subsample` 提到 4，或把
   `octomap_resolution` 调到 0.03。**不要**把 `max_update_rate` 降到 5 Hz 以下：手臂在动，点云和自滤波的
   机器人位姿有时间差，更新太慢会把旧手臂点留成幽灵障碍，supervisor 会
   反复重规划。10 Hz（与深度相机同频）是默认值。
4. 用 `top` 和 `gz stats` 区分：`gzserver` 高 = 相机渲染/物理；`move_group`
   高 = 点云/Octomap；`rviz2` 高 = 显示内容。

## 验收

```bash
ros2 run litearm_d435i_perception acceptance_d435i.sh
```

或分步：

```bash
# 终端 1：起栈（无显示）
ros2 launch litearm_d435i_perception d435i_avoidance.launch.py \
  gui:=false use_rviz:=false

# 终端 2：先进入与终端 1 相同的 ROS 域
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

ros2 run litearm_d435i_perception d435i_probe.py
```

`d435i_probe.py` 依次检查：

1. 相机点云在发，`/monitored_planning_scene` 出现非空 Octomap，
   supervisor 确认 `octomap_seen=true`；
2. 回到 `ready`，再用 `/compute_fk` 把一个已知可达关节位形换算成
   `ee_link` 位姿，作为 `PoseStamped` 目标；经 `/compute_ik` + OMPL
   规划并执行，位置误差 < 5 cm、姿态误差 < 0.2 rad；
3. 回 `ready`；
4. 再发位姿目标；从 supervisor 刚发布的轨迹中取中点附近（50%）的
   `ee_link` 位姿，把 Gazebo 里的 `dynamic_obstacle` 直接移到那里
   （相机能看到，Octomap 会更新）；
5. 期望 supervisor 复检剩余轨迹 → 取消 → 重规划，`replans >= 1`，
   最终仍 `succeeded`。

第 4 步是刻意的**确定性注入**：固定往复 sweep 不一定真的穿过当前规划
路径，验收需要可重复触发。**动态障碍默认关闭**，所以默认跑验收时第 4/5
步会自动跳过，只覆盖“感知 + 静态位姿”；要看动态重规划，用
`start_dynamic_obstacle:=true` 启动。

## 可移动物块（默认开启）

启动后工位外的地面上有一个橙红色物块。它只有 visual、没有 collision，
`<static>true</static>`，由 `movable_block.py` 通过
`/gazebo/set_entity_state` 改世界位姿；移动后 D435i 立刻在新位置看到它，
`/camera/depth/color/points`、Octomap 和 `/monitored_planning_scene` 实时更新。

无 GUI（`gui:=false`）时也可以改位置。另开终端，先进入同一个 ROS 域，再执行：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

# 摆到工作台上方（world 与 base_link 在本仿真中重合）
ros2 run litearm_d435i_perception move_block.py 0.25 0.10 0.20

# 只改位置、顺带绕 Z 轴转 45 度
ros2 run litearm_d435i_perception move_block.py 0.25 0.10 0.20 --yaw 0.785

# 回初始停车位
ros2 run litearm_d435i_perception move_block.py --reset
```

也可以直接用 ROS 接口输入（`PoseStamped` 的 `header.frame_id` 接受
`world` 或 `base_link`）：

```bash
# 只改位置：最短写法
ros2 topic pub --once /movable_block/target_point \
  geometry_msgs/msg/Point "{x: 0.25, y: 0.10, z: 0.20}"

# 改位置 + 朝向
ros2 topic pub --once /movable_block/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.25, y: 0.10, z: 0.20}, orientation: {w: 1.0}}}"

# 回初始位姿
ros2 service call /movable_block/reset std_srvs/srv/Trigger
```

查看实时更新：

- 有 RViz 时，打开 Displays 里的「相机点云」（默认关闭以省性能），
  或直接看 MotionPlanning 的 Scene Geometry：物块移动后 Octomap 里的
  橙色体素会跟着走；
- 纯 headless 时，观察 `/camera/depth/color/points` 的 `width/height`
  或截取一帧，比较物块位置附近的点：

```bash
ros2 topic hz /camera/depth/color/points
ros2 topic echo --once /movable_block/pose
```

物块位置没有范围限制；z 是几何中心，默认停车位在地面上（中心
`z=-0.06`）。放到工作台面上时中心约 `z=0.02`（台面 `z=-0.04`，半高
0.06 m）。它出现在 Octomap 后会被 MoveIt 当成障碍物，规划会自动绕开。

## 动态障碍演示（默认关闭）

需要动态障碍时显式启动：

```bash
ros2 launch litearm_d435i_perception d435i_avoidance.launch.py \
  start_dynamic_obstacle:=true dynamic_mode:=sweep
```

方块先停在 park，不会自己动；再手动使能：

```bash
# 开始往复（once 模式则走到终点后停）
ros2 service call /dynamic_obstacle/enable std_srvs/srv/SetBool "{data: true}"
# 暂停
ros2 service call /dynamic_obstacle/enable std_srvs/srv/SetBool "{data: false}"
# 回到 park
ros2 service call /dynamic_obstacle/reset std_srvs/srv/Trigger
```

在 RViz 里能同时看到彩色/深度点云和 Octomap 随方块更新。固定 sweep 不一定
穿过当前规划路径；要现场看重规划，可在机械臂执行中发一个会穿过 sweep 的
位姿目标，或直接用 `d435i_probe.py` 的确定性注入。

## 节点与接口

| 节点/脚本 | 作用 |
| --- | --- |
| `spawn_model.py` | 从 SDF/URDF 文件插入 Gazebo；启动顺序里相机/障碍先于机器人 |
| `camera_tf.py` | `base_link -> camera_link -> *_optical_frame` 静态 TF |
| `gazebo_robot_description.py` | 只提供 Gazebo 侧 `robot_description` 参数给 `gazebo_ros2_control` |
| `movable_block.py` | 驱动默认可改位物块；`/movable_block/target_pose`、`/movable_block/target_point`、`/movable_block/reset` |
| `move_block.py` | 命令行客户端：`move_block.py X Y Z [--yaw R]` / `--reset` |
| `dynamic_obstacle.py` | 通过 `/gazebo/set_entity_state` 驱动方块；`/dynamic_obstacle/enable`、`/dynamic_obstacle/reset` |
| `d435i_supervisor.py` | 位姿/关节目标、规划执行、剩余轨迹复检、停-重规划、失败撤退 |
| `wrist_scan.py` | 腕装相机多视角扫描：IK、规划执行、停稳采集、Octomap 融合、覆盖率报告 |
| `wrist_runtime_probe.py` | 在轨迹中点注入障碍，验证运行时复检/重规划 |
| `d435i_probe.py` | 验收探针 |

supervisor 接口：

| 类型 | 名称 |
| --- | --- |
| 目标 | `/d435i_supervisor/goal_pose` (`geometry_msgs/PoseStamped`) |
| 目标 | `/d435i_supervisor/goal_joints` (`sensor_msgs/JointState`) |
| 状态 | `/d435i_supervisor/status` (`std_msgs/String`, JSON) |
| 轨迹 | `/d435i_supervisor/planned_trajectory` |
| 服务 | `/d435i_supervisor/cancel` (`std_srvs/Trigger`) |

执行节拍：规划走 `/plan_kinematic_path`，执行走
`/joint_trajectory_controller/follow_joint_trajectory`。

## 几个 Gazebo Classic 的坑（本包已处理）

这些不是 MoveIt 的问题，但会直接让仿真“看起来没反应”，因此写在这里：

1. **link/joint 同名冲突**：sdformat9 把 link、joint、frame 放在同一个
   命名空间；litearm 描述里 link 和 joint 都叫 `joint1..joint7`，直接
   插入 Gazebo 会报 `Joint ... has a name collision`，模型插件不加载。
   launch 只在送进 Gazebo 的那份 URDF 里把冲突 link 改成 `gz_joint1..`；
   MoveIt 模型保持原名。
2. **`package://` 网格**：Gazebo Classic 不认 ROS 的 package URI，会去
   `models.gazebosim.org` 找，世界加载可能卡几分钟。launch 把 Gazebo 侧
   网格改写成 `file://<package_share>/...` 绝对路径。
3. **空几何 dummy link**：夹爪的 `litegrip_gripper_opening_link` 没有几何/
   惯性，URDF→SDF 时连同 `gripper_opening_joint` 一起被丢弃，导致
   `/joint_states` 缺关节、MoveIt 报 “Missing gripper_opening_joint”。
   launch 给这类关节子 link 补一个极小惯性。
4. **TF 与 Gazebo 模型分离**：`robot_state_publisher` 用 MoveIt 模型发
   `joint1..` 的 TF，Octomap 的 ShapeMask 才能做机器人自滤波；
   `gazebo_ros2_control` 通过 `gazebo_robot_description` 节点读带
   `gz_joint*` 的 Gazebo 模型。
5. **插件搜索路径**：不用 `gazebo_ros/gzserver.launch.py`，直接起
   gzserver 并显式给全 `GAZEBO_PLUGIN_PATH` / `LD_LIBRARY_PATH`，
   否则 `libgazebo_ros2_control.so` 静默不加载，`controller_manager`
   永远不出现。
6. **夹爪 link 前缀**：`litearm_litegrip.geometry.urdf.xacro` include
   `litegrip_description` 时没有把 `link_prefix` 传进去，被包含文件的空
   默认值生效，夹爪的 `base_link` 与机械臂的 `base_link` 重名
   （`link 'base_link' is not unique`）。两个 launch 都会在展开后把夹爪
   link 改名为 `litegrip_*`（joint 名保持不变）。改模型后请重新检查是否有
   重名 link。

## 接真实 D435i

1. 安装并启动 `realsense2_camera`，确认它发
   `/camera/depth/color/points`，frame 为 `camera_depth_optical_frame`。
2. 停掉 Gazebo 相机：不启动 `spawn_model.py` 的相机模型，或使用
   `start_camera:=false` 的变体；`sensors_3d.yaml` 不用改。
3. 把 `camera_mount.yaml` 的 `xyz/rpy` 换成实测标定值，仍然发布
   `base_link -> camera_link` 静态 TF。
4. 机械臂换成真实控制栈（`litearm_moveit_config` 的直连 USB 启动），
   `d435i_supervisor` 可以原样使用；注意真实 D435i 的近距噪声和
   `padding_offset` 需要按现场重新调。

真机避障 launch 现在在 `litearm_moveit_config`：
`litearm_real_avoidance.launch.py` 使用
`litearm_moveit_config/config/handeye/` 里的手眼 JSON，不再用
`camera_mount.yaml`；`camera_mount.yaml` 和 `camera_tf.py` 留在 Gazebo
仿真路径。

真机 launch 还会启动 C++ 节点 `cloud_envelope_filter`：它把原始
`/camera/depth/color/points` 变成 `/camera/depth/color/points_envelope`
再给 Octomap。默认直通；在 RViz 的 `/cloud_envelope_filter` 交互标记里
点击绿色/灰色小球切换“机器人碰撞模型周围 3cm”包络（内置 2cm + 额外
1cm）。绿色开、灰色关；这是模型包络，不是手动框选线缆；线缆离模型超过
3cm 时需要 cable_guard 或手动区域过滤。

**包里的手眼文件只对应这一台相机和这一个安装位置。** 它对应 D435i
序列号 243222074636 和 2026-11-26 的安装。换机器人、换相机、换安装位置，
或相机被碰过，都需要重新标定；原始文件的彩色内参是错的，必须用
`_fixed.json` 或重新标定。流程见
[`../litearm_moveit_config/README.zh-CN.md`](../litearm_moveit_config/README.zh-CN.md)。

当前包的固定外置假设是 `base_link` 不随臂运动。若以后改成腕部安装，
Octomap 的 `octomap_frame` 仍应填固定的 `base_link`（或 `world`），
不要填相机 link。

## 腕装相机多视角扫描（可选）

上面的固定外置相机 launch 仍然保留。腕装扫描是另一个 launch，两者都会发
`/camera/...`，不要同时启动：

```bash
# 有显示
ros2 launch litearm_d435i_perception d435i_wrist_scan.launch.py

# 无显示
ros2 launch litearm_d435i_perception d435i_wrist_scan.launch.py \
  gui:=false use_rviz:=false

# 只做 IK/可达性检查，不动机械臂
ros2 launch litearm_d435i_perception d435i_wrist_scan.launch.py \
  gui:=false use_rviz:=false scan_dry_run:=true
```

相机是挂在 `ee_link` 上的固定机器人 link（`config/wrist_camera.xacro`），
`robot_state_publisher` 会发布 `base_link -> ... -> camera_link ->
camera_*_optical_frame` 的动态 TF。默认是**侧视**安装：
`camera_mount_xyz="0 0.075 0.02"`、`camera_mount_rpy="0 0 0"`
（相机 +X = ee_link +X）。把光轴装成工具轴方向（rpy `0 -pi/2 0`）会让
顶视/环视的朝下位形在本臂上不可达——实测顶视和 55° 位置 KDL 全种子无解。
改安装位姿后先用 `scan_dry_run:=true` 验证。

`wrist_scan.py` 生成顶视 + 55° 环视（默认 8 个方位角），可选 30° 前侧
低环；用多组种子解 IK 并逐个校验状态；**顶视优先、低环最后**；每个环视
先到上方 8 cm 的预视角，再短距离下降；每段用 OMPL 规划、JTC 执行；每个
视角停稳 dwell 秒，让 `PointCloudOctomapUpdater` 把点云融进同一张
`base_link` Octomap。跨工位转移加 `ee_link` 最低高度路径约束
（`scan_safe_transfer_z`，默认 0.30 m）；约束规划失败会回退无约束并打
warning。覆盖率统计用 `/camera/depth/color/points_filtered`。

launch 用 `config/sensors_3d_wrist.yaml` 把**两台深度相机融进同一张
Octomap**：固定外置 D435i 用 `/fixed_camera/depth/color/points` 和
`fixed_camera_*` 帧，腕装 D435i 用 `/camera/...`。固定相机连续看整个
工作台，腕装相机补遮挡和近场细节。默认不再预置已知静态工位，桌子/夹具
由这两台相机观测；需要额外已知场景时可传 `scene_file:=...`。

视角按**顶视优先、仰角从高到低**排序，30° 低环最后；顶视所有方位角重合，
只生成一个视角。每个环视先到上方 8 cm 的预视角，再短距离下降；到预视角
的转移带 `ee_link` 最低高度路径约束（`scan_safe_transfer_z`，默认
0.30 m）。约束规划失败时会回退无约束并打 warning，保证扫描继续。这对
未观测几何仍是尽力而为：固定相机覆盖了大部分工位，但两台相机都看不到的
物体、或扫描中途才出现的物体，只受当前 Octomap 保护。默认规划预算为
6 s / 12 次，因为工位附近的路径可能较紧。

执行期间扫描节点还监听 `/monitored_planning_scene`：地图或碰撞对象更新时，
对**剩余轨迹点**调用 `/check_state_validity` 复检（用 `scan_check_period`
限频，从 `scan_check_lookahead` 之后开始采样）。发现被挡就取消 JTC goal、
等 `scan_cancel_settle` 停稳，再从当前状态重规划，单段最多
`scan_max_exec_replans` 次。每条规划轨迹都会发布到
`/wrist_scan/planned_trajectory`。这是“停-重规划”，不是 MoveIt Servo：
极快的动态障碍仍可能在两次复检之间碰到手臂，但剩余路径上的新障碍不会
再被忽略。

确定性验证：通过 `/apply_planning_scene` 在第一条轨迹中点注入一个 6 cm
方块，扫描报告 `runtime_replans=1`、`validity_checks=361`，2/2 视角
全部完成且无失败。可用下面命令复现：

```bash
ros2 run litearm_d435i_perception wrist_runtime_probe.py
```

它等待 `/wrist_scan/planned_trajectory`，在第一条足够长的轨迹中点注入
6 cm 方块，期望 `runtime_replans >= 1`，结束后移除方块。退出码 0 表示
复检触发；2 表示没有触发。

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `scan_targets` | `0.10,0.0,0.10` | 注视点，分号分隔 |
| `scan_radius` | `0.40` | 相机到注视点距离（m） |
| `scan_elevations` | `90,55` | 仰角（deg）；加 `30` 得到低环 |
| `scan_azimuths` | `8` | 每圈方位角数量 |
| `scan_dwell` | `0.60` | 每视角停稳采集秒数 |
| `scan_coverage_target` | `0.90` | 达到后提前结束（实测约 40–57%，默认会跑完整个图案） |
| `scan_max_views` | `0` | 最多执行视角数（0=全部） |
| `scan_clear_octomap` | `false` | 扫描前清空地图；false 保留启动期间建好的地图，避免扫描中桌子重新生长 |
| `scan_top_first` | `true` | 顶视优先，按仰角从高到低，低环最后 |
| `scan_safe_transfer` | `true` | 每个环视从上方 8 cm 预视角接近；跨工位转移加最低高度路径约束 |
| `scan_safe_transfer_z` | `0.30` | 转移时 `ee_link` 最低高度（m） |
| `scan_approach_height` | `0.08` | 预视角相对目标视角的抬升高度（m） |
| `scan_runtime_check` | `true` | 场景更新时复检剩余轨迹 |
| `scan_check_period` | `0.50` | 复检限频（s） |
| `scan_check_lookahead` | `0.40` | 跳过已承诺路径的时间（s） |
| `scan_check_stride` | `2` | 剩余轨迹采样步长（轨迹点） |
| `scan_max_exec_replans` | `3` | 单段轨迹被挡时最多重规划次数 |
| `scan_cancel_settle` | `0.50` | 取消 JTC goal 后等待停稳（s） |
| `scene_file` | 空 | 可选已知静态场景；空 = 只用两台相机 |
| `start_fixed_camera` | `true` | 启动固定外置 D435i，与腕装相机喂同一张 Octomap；false 时使用 `sensors_3d_wrist_only.yaml` |
| `fixed_camera_mount_file` | `config/fixed_camera_mount.yaml` | 固定相机安装位姿/帧名 |
| `scan_roi_min` / `scan_roi_max` | `-0.40,-0.50,-0.05` / `0.60,0.50,0.45` | 覆盖率 ROI |
| `scan_dry_run` | `false` | 只做 IK/校验，不运动 |
| `camera_mount_xyz` / `camera_mount_rpy` | `0 0.075 0.02` / `0 0 0` | 腕装位姿 |

接口：`/wrist_scan/status`（JSON：状态、视角数、覆盖率、点云帧数、
`runtime_replans`、`validity_checks`、`scene_objects_seen`）、
`/wrist_scan/planned_trajectory`（每条 `JointTrajectory`）、
`/wrist_scan/start`、`/wrist_scan/cancel`（`std_srvs/Trigger`）。
默认 `scan_auto_start=true`，等待 `scan_start_delay` 后自动开始。

覆盖率是**代理指标**，不代表“看到了所有表面”：它统计配置 ROI 内、沿相机
射线被观测到的体素（`scan_coverage_resolution`）。实测默认 9 个视角
（1 个顶视 + 8 个 55°）覆盖约 40–57%（随采集时序有波动），加可选 30° 低环
会更多；剩余部分是物体背面、桌面下方和可达视角以外的体积。约束转移在
路径很紧时可能回退无约束（会打 warning）。实测 9/9 和 14/14 视角全部
执行成功。残余盲区仍要靠 Octomap + 可见表面补全/安全
margin 兜底。扫描默认**不**清空 Octomap——两台相机从启动起就在建图；清空
（`scan_clear_octomap:=true`）会让桌子在扫描过程中重新长出来，远侧视角
可能因此无法规划，直到桌子被重新观测到。

## 已知限制

- supervisor 是“停 → 复检 → 重规划”，不是 MoveIt Servo 的实时避障；
  从发现障碍到新轨迹生效有秒级延迟。
- Octomap 在规划期间仍在更新，OMPL 返回的路径可能被规划后校验判为无效。
  返回空轨迹时按 `max_no_path_retries` 重试，只有连续这么多次都为空才以
  “无路径”结束重规划；“被挡恢复”用另一本 `max_replans` 预算，每次恢复
  开始执行后归零。验收日志里 `replans` 因此可能大于“障碍事件数”，这是
  预期行为。
- 点云自滤波只能排除机器人当前位形；快速运动时的拖影可能短时进入地图。
- `padding_offset` 默认 0.01 m，是为了避免工作台栅格膨胀到机器人底座；
  要更大安全裕度时优先抬工位/降台面，不要只调这个值。
- Gazebo Classic 已 EOL；本包只针对 Humble + Gazebo 11。
- 用 `SIGKILL`（或 `timeout`）结束 launch 可能留下 `scene_builder`、
  `gazebo_robot_description`、`movable_block` 或 Gazebo 进程。之后启动的
  `move_group` 可能收到旧的 planning-scene 对象，TF/仿真时间也会被污染
  （症状：`TF_OLD_DATA` 刷屏、机器人自身点云变成障碍、规划报 `99999`）。
  请用 Ctrl-C 正常停止，或在重启前清掉所有残留的 `litearm`/`gazebo` 进程。
