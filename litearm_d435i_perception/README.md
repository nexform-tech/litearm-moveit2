# litearm_d435i_perception

在 Gazebo Classic 中模拟一台固定外置的 Intel RealSense D435i，用真实深度点云
构建机械臂 + LiteGrip 夹爪周围的环境；点云经 MoveIt 2 的 Octomap 进入
PlanningScene，支持位姿/关节目标规划、执行中障碍更新后的“停 → 复检 →
重规划”，以及可选的 Gazebo 动态障碍（默认关闭）。

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

默认 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`。别的终端要 export 同样的值。

关键 launch 参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 启动 gzclient |
| `use_rviz` | `true` | 启动 RViz2（含点云、图像、Octomap 场景） |
| `scene_file` | 空 | 可选加载旧包 `scene.yaml` 静态碰撞对象；默认只用相机 Octomap |
| `camera_mount_file` | `config/camera_mount.yaml` | 相机安装位姿（Gazebo 与 TF 共用） |
| `start_camera` | `true` | 插入 Gazebo D435i；接真机/外部相机时设 `false` |
| `sensor_heartbeat_topic` | `/camera/color/camera_info` | supervisor 判传感器存活用的小消息话题 |
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
  （Python 每帧几百 KB）；现在改订阅 `/camera/color/camera_info`。

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

# 终端 2：同一 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY
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
| `dynamic_obstacle.py` | 通过 `/gazebo/set_entity_state` 驱动方块；`/dynamic_obstacle/enable`、`/dynamic_obstacle/reset` |
| `d435i_supervisor.py` | 位姿/关节目标、规划执行、剩余轨迹复检、停-重规划、失败撤退 |
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

当前包的固定外置假设是 `base_link` 不随臂运动。若以后改成腕部安装，
Octomap 的 `octomap_frame` 仍应填固定的 `base_link`（或 `world`），
不要填相机 link。

## 已知限制

- supervisor 是“停 → 复检 → 重规划”，不是 MoveIt Servo 的实时避障；
  从发现障碍到新轨迹生效有秒级延迟。
- Octomap 在规划期间仍在更新，OMPL 返回的路径可能被规划后校验判为无效。
  supervisor 会在 `max_replans` 预算内重试；验收日志里 `replans` 因此可能
  大于“障碍事件数”，这是预期行为。
- 点云自滤波只能排除机器人当前位形；快速运动时的拖影可能短时进入地图。
- `padding_offset` 默认 0.01 m，是为了避免工作台栅格膨胀到机器人底座；
  要更大安全裕度时优先抬工位/降台面，不要只调这个值。
- Gazebo Classic 已 EOL；本包只针对 Humble + Gazebo 11。
