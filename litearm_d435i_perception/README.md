English | [简体中文](README.zh-CN.md)

`litearm_d435i_perception` simulates a fixed external Intel RealSense D435i in
Gazebo Classic. It builds the environment around the seven-axis arm and
LiteGrip gripper from a real depth point cloud, which enters the MoveIt
PlanningScene through an Octomap. The package plans and executes pose and joint
goals, re-checks the remaining trajectory when obstacles change during
execution, and offers an optional Gazebo moving obstacle (disabled by default).

# litearm_d435i_perception

The package does not duplicate the robot model: the composed URDF/SRDF, gripper
envelope, Gazebo controllers, and workcell world all come from
`litearm_obstacle_avoidance`; planning parameters come from
`litearm_moveit_config`.

## Data flow

```text
Gazebo depth camera (libgazebo_ros_camera.so)
  -> /camera/depth/color/points        (frame_id=camera_depth_optical_frame)
  -> PointCloudOctomapUpdater          (config/sensors_3d.yaml)
     - ShapeMask self-filter: drops points inside the robot's collision bodies
     - padding_offset inflates the map to reduce grazing plans
  -> Octomap (frame_id=base_link)
  -> /monitored_planning_scene
  -> OMPL collision checking / d435i_supervisor remaining-trajectory re-check
```

Topic names deliberately match the real `realsense2_camera`:

| Topic | Content |
| --- | --- |
| `/camera/depth/color/points` | Depth point cloud (consumed by MoveIt) |
| `/camera/depth/color/depth/image_raw` | Depth image, float32 (published by the simulation plugin) |
| `/camera/color/image_raw` | Color image |
| `/camera/color/camera_info` | Color camera intrinsics |

## Prerequisites

```bash
sudo apt install ros-humble-gazebo-plugins
```

`libgazebo_ros_camera.so` lives in that package. Without it, Gazebo publishes no
point cloud and `d435i_supervisor` refuses motion with `no_sensor/no_octomap`.

The remaining dependencies (MoveIt, Gazebo, ros2_control, and the composed model
package) are the same as for `litearm_obstacle_avoidance`.

## Build and launch

Build from the workspace root:

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

With a display:

```bash
ros2 launch litearm_d435i_perception d435i_avoidance.launch.py
```

Without a display (acceptance or server):

```bash
ros2 launch litearm_d435i_perception d435i_avoidance.launch.py \
  gui:=false use_rviz:=false
```

The defaults are `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1`. Export the same
values in every other terminal.

Key launch arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `gui` | `true` | Start gzclient |
| `use_rviz` | `true` | Start RViz2 with point cloud, image, and Octomap scene |
| `scene_file` | empty | Optionally load the static collision objects from the older package's `scene.yaml`; by default only the camera Octomap is used |
| `camera_mount_file` | `config/camera_mount.yaml` | Camera mount pose, shared by Gazebo and TF |
| `start_camera` | `true` | Insert the Gazebo D435i; set `false` for a real or external camera |
| `sensor_heartbeat_topic` | `/camera/color/camera_info` | Small topic the supervisor uses to decide whether the sensor is alive |
| `start_dynamic_obstacle` | `false` | Insert and drive `dynamic_obstacle`; off by default |
| `dynamic_mode` | `sweep` | `sweep` moves back and forth; `once` stops at the end |
| `dynamic_speed` | `0.20` | m/s |
| `extra_gazebo_args` | empty | Appended to gzserver, for example `--verbose` |

## Performance

This stack struggles on integrated graphics when the desktop has a heavy
background load. Measured on an i5-14450HX with integrated graphics while
VSCode and Firefox were open:

| Configuration | Main CPU | RTF |
| --- | --- | --- |
| 640×480/15 Hz, subsample=2, Octomap 5 Hz, GUI+RViz | move_group 65–99%, rviz2 53%, gzserver 34%, gzclient 15% | 0.745 |
| Current default (424×240, depth 10 Hz/color 5 Hz, subsample=2, Octomap 10 Hz/0.02 m, RViz 15 FPS) | Clearly lower | Close to 1.0 |

The planner itself is not the bottleneck. The cost comes from:

- **ShapeMask self-filtering** in `PointCloudOctomapUpdater`: every point is
  tested against the robot's collision bodies, so denser clouds and faster
  updates cost `move_group` more CPU;
- **RViz + Gazebo GUI** sharing integrated graphics for scene, cloud, and image
  rendering;
- The supervisor used to subscribe to and deserialize the whole `PointCloud2`
  to check sensor liveness (hundreds of KB per frame in Python). It now
  subscribes to `/camera/color/camera_info` instead.

If the stack is still slow, try these in order:

1. **Keep only one interface**: `gui:=false use_rviz:=true`, or
   `gui:=false use_rviz:=false` for pure simulation.
2. In RViz Displays, turn off the color image. The raw camera point cloud is
   off by default; enable it only when you need it (the Octomap is under
   MotionPlanning, Scene Geometry).
3. Lower the resolution or frame rate in `models/d435i_camera.sdf`, raise
   `point_subsample` to 4 in `config/sensors_3d.yaml`, or set
   `octomap_resolution` to 0.03. **Do not** drop `max_update_rate` below 5 Hz:
   the arm moves, so the cloud and the self-filtered robot pose differ in time.
   Slow updates leave ghost obstacles from the old arm pose, and the supervisor
   replans repeatedly. 10 Hz (the depth camera rate) is the default.
4. Use `top` and `gz stats` to separate the causes: high `gzserver` means
   camera rendering or physics; high `move_group` means cloud or Octomap; high
   `rviz2` means display content.

## Acceptance

```bash
ros2 run litearm_d435i_perception acceptance_d435i.sh
```

Or step by step:

```bash
# Terminal 1: start the stack without a display
ros2 launch litearm_d435i_perception d435i_avoidance.launch.py \
  gui:=false use_rviz:=false

# Terminal 2: same ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY
ros2 run litearm_d435i_perception d435i_probe.py
```

`d435i_probe.py` checks the following in order:

1. The camera point cloud is publishing, a non-empty Octomap appears in
   `/monitored_planning_scene`, and the supervisor reports `octomap_seen=true`.
2. Return to `ready`, then use `/compute_fk` to convert a known reachable joint
   configuration into an `ee_link` pose and send it as a `PoseStamped` goal.
   `/compute_ik` plus OMPL plans and executes it; the position error must stay
   below 5 cm and the orientation error below 0.2 rad.
3. Return to `ready`.
4. Send another pose goal. Take the `ee_link` pose near the midpoint (50%) of
   the trajectory the supervisor just published, and move the Gazebo
   `dynamic_obstacle` there directly. The camera sees it, and the Octomap
   updates.
5. Expect the supervisor to re-check the remaining trajectory, cancel, and
   replan with `replans >= 1`, while the final state remains `succeeded`.

Step 4 is a deliberate **deterministic injection**: a fixed sweep does not
necessarily cross the current planned path, and acceptance needs a repeatable
trigger. **The dynamic obstacle is off by default**, so the default acceptance
run skips steps 4 and 5 and covers only perception plus static pose goals. To
exercise dynamic replanning, start with `start_dynamic_obstacle:=true`.

## Dynamic obstacle demo (disabled by default)

Start the stack with the dynamic obstacle explicitly enabled:

```bash
ros2 launch litearm_d435i_perception d435i_avoidance.launch.py \
  start_dynamic_obstacle:=true dynamic_mode:=sweep
```

The block waits at the park position and does not move until you enable it:

```bash
# Start the sweep (once mode stops at the end)
ros2 service call /dynamic_obstacle/enable std_srvs/srv/SetBool "{data: true}"
# Pause
ros2 service call /dynamic_obstacle/enable std_srvs/srv/SetBool "{data: false}"
# Return to park
ros2 service call /dynamic_obstacle/reset std_srvs/srv/Trigger
```

RViz shows the color and depth clouds and the Octomap updating as the block
moves. A fixed sweep does not necessarily cross the current planned path. To
see replanning live, send a pose goal that crosses the sweep while the arm is
executing, or use the deterministic injection in `d435i_probe.py`.

## Nodes and interfaces

| Node or script | Purpose |
| --- | --- |
| `spawn_model.py` | Insert an SDF/URDF file into Gazebo; the start sequence inserts the camera and obstacle before the robot |
| `camera_tf.py` | Static TF from `base_link` to `camera_link` and the two optical frames |
| `gazebo_robot_description.py` | Serve the Gazebo-side `robot_description` parameter to `gazebo_ros2_control` |
| `dynamic_obstacle.py` | Drive the block through `/gazebo/set_entity_state`; `/dynamic_obstacle/enable`, `/dynamic_obstacle/reset` |
| `d435i_supervisor.py` | Pose and joint goals, planning and execution, remaining-trajectory re-check, stop-and-replan, retreat on failure |
| `d435i_probe.py` | Acceptance probe |

Supervisor interfaces:

| Type | Name |
| --- | --- |
| Goal | `/d435i_supervisor/goal_pose` (`geometry_msgs/PoseStamped`) |
| Goal | `/d435i_supervisor/goal_joints` (`sensor_msgs/JointState`) |
| Status | `/d435i_supervisor/status` (`std_msgs/String`, JSON) |
| Trajectory | `/d435i_supervisor/planned_trajectory` |
| Service | `/d435i_supervisor/cancel` (`std_srvs/Trigger`) |

Execution path: planning uses `/plan_kinematic_path`; execution uses
`/joint_trajectory_controller/follow_joint_trajectory`.

## Gazebo Classic pitfalls (handled by this package)

These are not MoveIt problems, but they make the simulation appear unresponsive,
so they are documented here:

1. **link/joint name collisions**: sdformat9 puts links, joints, and frames in
   one namespace. The litearm description names both links and joints
   `joint1..joint7`, so inserting it directly into Gazebo reports
   `Joint ... has a name collision` and model plugins do not load. The launch
   file renames the colliding links to `gz_joint1..` only in the URDF sent to
   Gazebo; the MoveIt model keeps the original names.
2. **`package://` meshes**: Gazebo Classic does not understand ROS package URIs.
   It looks on `models.gazebosim.org`, and world loading can stall for minutes.
   The launch file rewrites Gazebo-side meshes to absolute
   `file://<package_share>/...` paths.
3. **Empty geometry on dummy links**: the gripper's
   `litegrip_gripper_opening_link` has no geometry or inertia, so the URDF to
   SDF conversion drops it together with `gripper_opening_joint`. Then
   `/joint_states` lacks the joint and MoveIt reports
   "Missing gripper_opening_joint". The launch file gives such child links a
   minimal inertia.
4. **TF and the Gazebo model are separate**: `robot_state_publisher` publishes
   TF for `joint1..` from the MoveIt model, which the Octomap ShapeMask needs
   for self-filtering. `gazebo_ros2_control` reads the Gazebo model with
   `gz_joint*` through the `gazebo_robot_description` node.
5. **Plugin search path**: the launch file does not use
   `gazebo_ros/gzserver.launch.py`. It starts gzserver directly and sets the
   full `GAZEBO_PLUGIN_PATH` and `LD_LIBRARY_PATH`; otherwise
   `libgazebo_ros2_control.so` silently fails to load and `controller_manager`
   never appears.

## Connecting a real D435i

1. Install and start `realsense2_camera`, and confirm it publishes
   `/camera/depth/color/points` with frame `camera_depth_optical_frame`.
2. Stop the Gazebo camera: do not start the camera model in `spawn_model.py`,
   or use a variant with `start_camera:=false`. `sensors_3d.yaml` does not need
   to change.
3. Replace the `xyz/rpy` values in `camera_mount.yaml` with your calibration.
   The package still publishes the static `base_link -> camera_link` TF.
4. Switch the arm to the real control stack (the direct-USB launch in
   `litearm_moveit_config`). `d435i_supervisor` works unchanged. Note that the
   real D435i's near-range noise and `padding_offset` need retuning on site.

This package assumes a fixed external camera where `base_link` does not move
with the arm. If you later mount the camera on the wrist, keep the Octomap
`octomap_frame` at the fixed `base_link` (or `world`), not at the camera link.

## Known limitations

- The supervisor implements stop, re-check, and replan, not MoveIt Servo's
  real-time avoidance. Several seconds can pass between detecting an obstacle
  and executing a new trajectory.
- The Octomap keeps updating during planning, so a path returned by OMPL may
  fail the post-planning validation. The supervisor retries within its
  `max_replans` budget. `replans` in acceptance logs can therefore exceed the
  number of obstacle events; this is expected.
- Point cloud self-filtering only removes the robot's current configuration.
  Fast motion can leave brief trails in the map.
- `padding_offset` defaults to 0.01 m so the workbench grid does not inflate
  into the robot base. For more safety margin, raise the workcell or lower the
  table first; do not only increase this value.
- Gazebo Classic is end-of-life. This package targets Humble and Gazebo 11 only.
