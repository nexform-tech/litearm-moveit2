English | [简体中文](README.zh-CN.md)

`litearm_d435i_perception` simulates a fixed external Intel RealSense D435i in
Gazebo Classic. It builds the environment around the seven-axis arm and
LiteGrip gripper from a real depth point cloud, which enters the MoveIt
PlanningScene through an Octomap. The package plans and executes pose and joint
goals, re-checks the remaining trajectory when obstacles change during
execution, and ships a demo block that can be repositioned at runtime through
ROS (plus an optional Gazebo moving obstacle, disabled by default).

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

The defaults are `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1`. In **every new
terminal** (`ros2 topic`, `ros2 run`, RViz, ...), export them first:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

Otherwise the terminal and the simulation are in different ROS domains. The
symptoms are `Waiting for at least 1 matching subscription(s)...` hanging
forever, or `no node subscribes to /movable_block/target_pose`; the nodes are
running, they just cannot see each other.

Key launch arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `gui` | `true` | Start gzclient |
| `use_rviz` | `true` | Start RViz2 with point cloud, image, and Octomap scene |
| `scene_file` | empty | Optionally load the static collision objects from the older package's `scene.yaml`; by default only the camera Octomap is used |
| `camera_mount_file` | `config/camera_mount.yaml` | Camera mount pose, shared by Gazebo and TF |
| `start_camera` | `true` | Insert the Gazebo D435i; set `false` for a real or external camera |
| `sensor_heartbeat_topic` | `/camera/color/camera_info` | Small topic the supervisor uses to decide whether the sensor is alive |
| `start_movable_block` | `true` | Insert the block that can be repositioned at runtime |
| `movable_block_pose` | `0.30 0.55 -0.06` | Initial block position `x y z` (m, world/base_link); the default is on the floor outside the workcell, visible to the camera but out of the arm's way |
| `movable_block_yaw` | `0.0` | Initial block yaw around Z (rad) |
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
  subscribes to CameraInfo heartbeats instead. The real launch watches both
  the depth and the color topics (`sensor_topics`), so either stream stopping
  refuses motion.

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

# Terminal 2: enter the same ROS domain as terminal 1 first
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

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

## Repositionable demo block (enabled by default)

After launch there is an orange-red block on the floor outside the workcell.
It is visual-only and `<static>true</static>`; `movable_block.py` changes its
world pose through `/gazebo/set_entity_state`. Once it moves, the D435i sees it
at the new position and `/camera/depth/color/points`, the Octomap, and
`/monitored_planning_scene` update in real time.

No GUI is needed (`gui:=false` works). In another terminal, enter the same ROS
domain first, then run:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

# Place it above the workbench (world and base_link coincide in this sim)
ros2 run litearm_d435i_perception move_block.py 0.25 0.10 0.20

# Change position and yaw by 45 degrees
ros2 run litearm_d435i_perception move_block.py 0.25 0.10 0.20 --yaw 0.785

# Return to the initial park pose
ros2 run litearm_d435i_perception move_block.py --reset
```

You can also use the ROS interfaces directly (`PoseStamped.header.frame_id`
accepts `world` or `base_link`):

```bash
# Position only: shortest form
ros2 topic pub --once /movable_block/target_point \
  geometry_msgs/msg/Point "{x: 0.25, y: 0.10, z: 0.20}"

# Position and orientation
ros2 topic pub --once /movable_block/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.25, y: 0.10, z: 0.20}, orientation: {w: 1.0}}}"

# Back to the initial pose
ros2 service call /movable_block/reset std_srvs/srv/Trigger
```

To watch the update:

- With RViz, enable the "相机点云" display (off by default to save CPU), or
  watch MotionPlanning's Scene Geometry: the Octomap voxels follow the block;
- Headless: observe the cloud dimensions or sample a frame and compare points
  around the block:

```bash
ros2 topic hz /camera/depth/color/points
ros2 topic echo --once /movable_block/pose
```

There is no position range check; `z` is the geometric center. The default
park pose sits on the floor (center `z=-0.06`). To put it on the workbench top
(`z=-0.04`) with a 0.12 m cube, use center `z≈0.02`. Once the block enters the
Octomap, MoveIt treats it as an obstacle and plans around it.

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
| `movable_block.py` | Drive the default repositionable block; `/movable_block/target_pose`, `/movable_block/target_point`, `/movable_block/reset` |
| `move_block.py` | Command-line client: `move_block.py X Y Z [--yaw R]` / `--reset` |
| `dynamic_obstacle.py` | Drive the block through `/gazebo/set_entity_state`; `/dynamic_obstacle/enable`, `/dynamic_obstacle/reset` |
| `d435i_supervisor.py` | Pose and joint goals, planning and execution, remaining-trajectory re-check, stop-and-replan, retreat on failure |
| `wrist_scan.py` | Wrist-camera multi-view scan: IK, plan/execute, stop-and-stare, Octomap fusion, coverage report |
| `wrist_runtime_probe.py` | Inject a box mid-trajectory and verify the runtime re-check/replan |
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
6. **Gripper link prefix**: `litearm_litegrip.geometry.urdf.xacro` includes
   `litegrip_description` without passing `link_prefix`, so the included file's
   empty default wins and the gripper's `base_link` collides with the arm's
   (`link 'base_link' is not unique`). Both launches post-process the expanded
   URDF and rename the gripper links to `litegrip_*`; joint names are unchanged.
   If you change the model, re-check for duplicate link names.

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

The real-arm avoidance launch now lives in `litearm_moveit_config`:
`litearm_real_avoidance.launch.py` uses the hand-eye JSON in
`litearm_moveit_config/config/handeye/`, not `camera_mount.yaml`.
`camera_mount.yaml` and `camera_tf.py` remain the Gazebo path.

The real launch also starts the C++ `cloud_envelope_filter` node: it turns
`/camera/depth/color/points` into `/camera/depth/color/points_envelope` for the
Octomap. It always runs a flying-pixel filter: each point is projected back to
a pixel with `/camera/depth/camera_info`, and pixels whose neighbourhood depth
spread exceeds `flying_pixel_depth_jump` (0.03 m, radius 1 px) are dropped.
That removes the fan of intermediate depth the D435i generates between the arm
silhouette and the table — the source of the staircase-shaped ghost voxels that
survive the 2 cm self-filter. The cloud does not need to be organized. Click
the green/gray sphere in the `/cloud_envelope_filter` interactive marker to
additionally toggle a 3 cm envelope around the robot collision model (2 cm
built-in + 1 cm extra). Green = on, gray = off. This is a model envelope, not a
manual cable selection; if the cable is more than 3 cm from the model, use
cable_guard or a manual region filter.

**The included hand-eye calibration is a reference for one camera and one
mount.** A different robot, camera, mount, or a moved or bumped camera needs a
new calibration. See
[`../litearm_moveit_config/README.md`](../litearm_moveit_config/README.md).

This package assumes a fixed external camera where `base_link` does not move
with the arm. If you later mount the camera on the wrist, keep the Octomap
`octomap_frame` at the fixed `base_link` (or `world`), not at the camera link.

## Wrist-mounted camera scan (optional)

The fixed external camera launch above remains available. The wrist scan is a
separate launch and must not run at the same time (both publish
`/camera/...`):

```bash
# GUI
ros2 launch litearm_d435i_perception d435i_wrist_scan.launch.py

# Headless
ros2 launch litearm_d435i_perception d435i_wrist_scan.launch.py \
  gui:=false use_rviz:=false

# Only solve/validate the viewpoints, never move the arm
ros2 launch litearm_d435i_perception d435i_wrist_scan.launch.py \
  gui:=false use_rviz:=false scan_dry_run:=true
```

The camera is a fixed robot link (`config/wrist_camera.xacro`) attached to
`ee_link`, so `robot_state_publisher` publishes the dynamic chain
`base_link -> ... -> camera_link -> camera_*_optical_frame`. The default mount
is **side-looking**: `camera_mount_xyz="0 0.075 0.02"`,
`camera_mount_rpy="0 0 0"` (camera +X = ee_link +X). Pointing the optical axis
along the tool axis (rpy `0 -pi/2 0`) makes the downward scan poses
kinematically unreachable on this arm — at the top/55° positions KDL returned
no solution for all tested seeds. Change the mount only after verifying with
`scan_dry_run:=true`.

`wrist_scan.py` generates a top view and a 55° ring (8 azimuths by default),
optionally a 30° front ring; solves IK with several seeds; validates each state;
orders views **top first and low ring last**; reaches each ring view through a
pre-view 8 cm above it and then descends; plans each leg with OMPL and executes
it on the JTC; and dwells at every view while `PointCloudOctomapUpdater` fuses
the cloud into the same `base_link` Octomap. Cross-workspace transfers carry a
path constraint keeping `ee_link` above `scan_safe_transfer_z` (0.30 m); if the
constrained plan fails, it retries unconstrained and logs a warning. Coverage
statistics use `/camera/depth/color/points_filtered`.

The launch runs **two depth cameras into one Octomap** using
`config/sensors_3d_wrist.yaml`: the fixed external D435i on
`/fixed_camera/depth/color/points` with `fixed_camera_*` frames, and the wrist
D435i on `/camera/...`. The fixed camera keeps a continuous global view of the
workcell; the wrist camera fills occlusions and near-field detail. No known
static scene is preloaded by default — the table and fixtures are observed by
the two cameras. `scene_file:=...` can still load an optional known scene.

Viewpoints are ordered **top first, then descending elevation**, so the 30° ring
is last; the top elevation produces a single view because all azimuths coincide.
Each ring view is approached through a pre-view 8 cm above it, then a short
descent. The transfer to a pre-view carries a path constraint keeping `ee_link`
at or above `scan_safe_transfer_z` (0.30 m). If that constrained plan fails, the
node retries without the constraint and logs a warning, so the scan still
progresses. This is best-effort for unknown geometry: the fixed camera covers
most of the workcell continuously, but objects outside both fields of view or
first seen mid-scan are only handled by the current Octomap. The default
planning budget is 6 s / 12 attempts because paths near the workcell can be
tight.

During execution the scan also listens to `/monitored_planning_scene`. When the
Octomap or collision objects update, it re-checks the **remaining** trajectory
points with `/check_state_validity` (throttled by `scan_check_period`, starting
after `scan_check_lookahead`). If any sampled point is blocked, it cancels the
JTC goal, waits `scan_cancel_settle`, and replans from the current state, up to
`scan_max_exec_replans` times. Every planned trajectory is published on
`/wrist_scan/planned_trajectory`. This is stop-and-replan, not MoveIt Servo: a
very fast obstacle can still reach the arm between checks, but a new obstacle
in the remaining path no longer goes unnoticed.

Deterministic validation: a 6 cm box was injected at the midpoint of the first
trajectory via `/apply_planning_scene`; the scan reported
`runtime_replans=1`, `validity_checks=361`, and completed 2/2 views with no
failures. Run the same check with:

```bash
ros2 run litearm_d435i_perception wrist_runtime_probe.py
```

It waits for `/wrist_scan/planned_trajectory`, injects a 6 cm box at the first
long trajectory's midpoint, expects `runtime_replans >= 1`, then removes the
box. Exit code 0 means the recheck fired; 2 means it did not.

| Argument | Default | Purpose |
| --- | --- | --- |
| `scan_targets` | `0.10,0.0,0.10` | look-at points, `;` separated |
| `scan_radius` | `0.40` | camera-to-target distance (m) |
| `scan_elevations` | `90,55` | view elevations (deg); add `30` for a low front ring |
| `scan_azimuths` | `8` | views per ring |
| `scan_dwell` | `0.60` | settle time per view (s) |
| `scan_coverage_target` | `0.90` | early stop threshold (runs reach ~40–57%, so the full pattern runs) |
| `scan_max_views` | `0` | cap on executed views (0 = all) |
| `scan_clear_octomap` | `false` | clear the map before scanning; false keeps the startup map and avoids mid-scan table regrowth |
| `scan_top_first` | `true` | top view first, descending elevations; low ring last |
| `scan_safe_transfer` | `true` | approach each ring view from an 8 cm pre-view; add a minimum-height path constraint on transfers |
| `scan_safe_transfer_z` | `0.30` | minimum `ee_link` height for constrained transfers (m) |
| `scan_approach_height` | `0.08` | pre-view height above the target view (m) |
| `scan_runtime_check` | `true` | re-check the remaining trajectory when the scene updates |
| `scan_check_period` | `0.50` | re-check throttle (s) |
| `scan_check_lookahead` | `0.40` | skip this much of the already-committed path (s) |
| `scan_check_stride` | `2` | sample every Nth remaining trajectory point |
| `scan_max_exec_replans` | `3` | replans per blocked trajectory segment |
| `scan_cancel_settle` | `0.50` | wait after canceling the JTC goal (s) |
| `scene_file` | empty | optional known static scene; empty uses the two cameras only |
| `start_fixed_camera` | `true` | run the fixed external D435i into the same Octomap; false selects `sensors_3d_wrist_only.yaml` |
| `fixed_camera_mount_file` | `config/fixed_camera_mount.yaml` | fixed camera mount pose/frames |
| `scan_roi_min` / `scan_roi_max` | `-0.40,-0.50,-0.05` / `0.60,0.50,0.45` | coverage ROI |
| `scan_dry_run` | `false` | IK/validity only, no motion |
| `camera_mount_xyz` / `camera_mount_rpy` | `0 0.075 0.02` / `0 0 0` | wrist mount |

Interfaces: `/wrist_scan/status` (JSON: state, views, coverage, cloud frames,
`runtime_replans`, `validity_checks`, `scene_objects_seen`),
`/wrist_scan/planned_trajectory` (each planned `JointTrajectory`),
`/wrist_scan/start` and `/wrist_scan/cancel` (`std_srvs/Trigger`). Auto-start is
on by default after `scan_start_delay`.

Coverage is a **proxy**, not a guarantee of "all surfaces": it counts voxels in
the configured ROI that were observed along camera rays at
`scan_coverage_resolution`. In testing, the default 9-view pattern observed
roughly 40–57% of the configured ROI (the number varies with capture timing),
and the optional 30° ring adds more; the rest is occluded volume behind objects,
under the table, or outside the reachable viewpoints. Constrained transfers can
fall back to unconstrained planning on a tight path; that is logged. In testing,
9/9 and 14/14 views executed. Use the Octomap plus the object-completion/padding
measures for the residual. The scan does **not** clear the Octomap by default
because both cameras have already been building it during startup; clearing it
(`scan_clear_octomap:=true`) makes the table reappear during the scan and far
views can become unplannable until it is re-observed.

## Known limitations

- The supervisor implements stop, re-check, and replan, not MoveIt Servo's
  real-time avoidance. Several seconds can pass between detecting an obstacle
  and executing a new trajectory.
- The Octomap keeps updating during planning, so a path returned by OMPL may
  fail the post-planning validation. Empty plans are retried up to
  `max_no_path_retries`, and only that consecutive-empty-plan budget ends
  replanning as "no path". Blocked-path recoveries use the separate
  `max_replans` budget, which resets after each recovery that starts
  executing. `replans` in acceptance logs can therefore exceed the number of
  obstacle events; this is expected.
- Point cloud self-filtering only removes the robot's current configuration.
  Fast motion can leave brief trails in the map.
- `padding_offset` defaults to 0.01 m so the workbench grid does not inflate
  into the robot base. For more safety margin, raise the workcell or lower the
  table first; do not only increase this value.
- Gazebo Classic is end-of-life. This package targets Humble and Gazebo 11 only.
- Killing a launch with `SIGKILL` (or `timeout`) can leave `scene_builder`,
  `gazebo_robot_description`, `movable_block`, or Gazebo processes alive; a
  later `move_group` may then receive stale planning-scene objects, and TF/sim
  time can be polluted (symptoms: `TF_OLD_DATA` floods, self-filtered robot
  points becoming obstacles, planning failing with `99999`). Stop with Ctrl-C,
  or kill all stale `litearm`/`gazebo` processes before restarting.
