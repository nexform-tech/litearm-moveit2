English | [简体中文](README.zh-CN.md)

# litearm_moveit_config

MoveIt 2 configuration for the LiteArm seven-axis arm: the SRDF, KDL
kinematics, joint limits, OMPL pipeline, controller mapping, and the launch
files. It plans the arm-only model, with no gripper, and ships an eye-to-hand
D435i obstacle avoidance stack for the real arm. Read this package when you
plan on the real LiteArm or need the hand-eye calibration that the real stack
uses.

## 1. Build

Build from the workspace root:

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

**Do not add `--symlink-install`.** The `litearm-cpp` SDK is pulled into
`litearm_ros2_control` with `EXCLUDE_FROM_ALL`, so its CLI is never built while
its install rule still runs, and the symlink install fails.

## 2. Hand-eye calibration

The real stack uses an eye-to-hand D435i calibration. The files in
`config/handeye/` are reference measurements for one camera and one mount, not
a portable calibration.

| File | Use |
| --- | --- |
| `handeye_eye_to_hand_TSAI_20261126_102300_fixed.json` | Corrected TSAI result for D435i serial 243222074636 and the 2026-11-26 mount. The launch uses it by default. Reuse it only on the same physical cell, after the verification below. |
| `handeye_eye_to_hand_TSAI_20261126_102300.json` | Original capture, kept for traceability. It used `fx=615.2` (depth/IR) for 1280x720 color images, so the transform is about 15 cm off. **Do not use it for motion.** |

**Do not reuse the included calibration on another robot, camera, or mount.**
The hand-eye transform is a physical measurement between the camera and the
robot base. A different base position, a different D435i (the color intrinsics
are per device), a different mount, or a bumped camera makes the file wrong.
Recalibrate, then verify.

### Recalibrate

1. Mount the camera rigidly and do not move it after calibration. Moving the
   camera or the robot base invalidates the result.
2. Capture at least 16 eye-to-hand poses with a 6x6 ChArUco board (30 mm
   squares, 22.5 mm markers) fixed rigidly to the robot flange. Use 1280x720
   color images and record `T_base_flange` for each pose.
3. Read the actual D435i color intrinsics from `/camera/color/camera_info`.
   At 1280x720 the color `fx` is about 910, not 615. Run the eye-to-hand TSAI
   or PARK solve in HandEye Studio with the color intrinsics, check the
   reprojection error and the residuals, and save the corrected JSON.
4. Keep the convention: `T_base_cam` maps the camera frame into `base_link`.
   The camera frame is the color optical frame (x right, y down, z forward).
   `camera_calibration_tf.py` composes it with the driver's
   `camera_link -> camera_color_optical_frame` factory transform and publishes
   `base_link -> camera_link`.
5. Pass the new file to the launch:

   ```bash
   ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py \
     calibration_file:=$HOME/litearm_ws/handeye.json
   ```

   Keep `calibration_frame:=camera_color_optical_frame` unless the board was
   observed through the depth/IR stream. Set `camera_serial:=...` when more
   than one RealSense is attached.

### Verify the calibration

1. Start the stack and open RViz:

   ```bash
   ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py \
     start_supervisor:=false
   ```

2. Enable the raw point cloud and compare it with the robot model. The arm and
   the base must line up; a few centimetres of offset is a calibration defect.
3. Check `/camera/depth/color/points_filtered`. The arm surface should be gone;
   leftover points on the arm mean the self-filter or the extrinsic is wrong.
4. Look at MotionPlanning > Scene Geometry. The Octomap must not grow over the
   arm, the base, or the table contact. Raise the base or crop the table region
   if it does.
5. Send a small goal with the e-stop in reach before trusting the calibration
   for larger motions.

## 3. Run the real arm with D435i avoidance

From the workspace root with `install/setup.bash` sourced:

```bash
ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py
```

The default starts the direct-USB control stack, move_group, the D435i, the
calibration TF, the supervisor, the goal marker, and RViz. The arm model has no
gripper. The launch does not start Gazebo and does not use sim time.

Pass `start_control:=false` when a control stack already runs,
`start_camera:=false` when the driver runs elsewhere, and
`start_supervisor:=false` for a perception-only run.

| Argument | Default | Effect |
| --- | --- | --- |
| `start_control` | `true` | start the direct-USB control stack |
| `start_camera` | `true` | start `realsense2_camera_node` |
| `camera_serial` | empty | pick the RealSense by serial |
| `start_supervisor` | `true` | plan, execute, and recheck the remaining trajectory |
| `start_goal_marker` | `true` | start the RViz 6-DOF goal marker |
| `retreat_enabled` | `false` | after `max_replans`, stop in place; `true` retreats to ready |
| `velocity_scaling` | `0.08` | supervisor speed; keep 0.05 to 0.1 for first runs |
| `calibration_file` | the `_fixed.json` file | hand-eye JSON |
| `calibration_frame` | `camera_color_optical_frame` | frame the hand-eye JSON uses |
| `shape_tf_cache_wait` | `0.5` | ShapeMask TF lookup wait, in seconds |
| `scene_file` | empty | optional static scene YAML |
| `ros_domain_id` | `42` | ROS domain for every node |
| `ros_localhost_only` | `true` | keep discovery on this machine |

The sensor tuning is in `config/sensors_3d_real.yaml`. `padding_offset` is the
self-filter inflation, not the planning clearance. The first run at `0.02` m
works with a good extrinsic; raise it to `0.03` when the arm surface leaks into
the Octomap. The planning margin comes from `octomap_resolution`, the static
scene, and the low speed.

The supervisor plans, executes, and rechecks the remaining trajectory after a
scene update. A blocked point cancels the JTC goal and replans from the current
state, up to `max_replans` (5). With the default `retreat_enabled=false`, a
failed replan ends in a `safe_stop` where the arm is; with
`retreat_enabled:=true`, it retreats to the ready pose at 0.05 scaling.
`ignore_base_link_contacts` keeps the fixed base/table mounting contact from
being treated as a moving obstacle.

## 4. Goal marker and goal tool

The RViz marker is placed at the current `ee_link`. Drag the blue box to move
and the x/y/z rings to rotate. While you drag, the node solves IK at 10 Hz and
publishes a ghost arm on `/display_planned_path`; the ghost is a preview and
does not move the arm. Right-click the marker for:

- Send goal: publish the pose to `/d435i_supervisor/goal_pose`;
- Home: send seven zero joint angles to `/d435i_supervisor/goal_joints`;
- Preview: run IK and OMPL and publish the trajectory without executing;
- Reset: move the marker back to the current `ee_link`;
- Cancel: call `/d435i_supervisor/cancel`.

The Home entry sends zeros. Change `home_positions` in the launch to
`[0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]` if you prefer the ready pose.

The command line tool prints the current pose and joint angles, and can send a
pose, a joint goal, or the zero pose:

```bash
ros2 run litearm_moveit_config goal_tool.py
ros2 run litearm_moveit_config goal_tool.py --home
ros2 run litearm_moveit_config goal_tool.py \
  --pose 0.25 0.0 0.25 3.14159 0.0 0.0
ros2 run litearm_moveit_config goal_tool.py \
  --joints 0 0 0 -1.5708 0 0 0
```

Use the supervisor paths for execution. The RViz MotionPlanning **Execute**
button calls move_group directly and bypasses the supervisor's recheck; use
**Plan** there only to inspect a path.

## 5. Safety and known limitations

- This is not a safety-rated avoidance stack. Keep the physical e-stop within
  reach; the firmware watchdog and the controller tolerances are the last
  layer.
- The supervisor stops and replans; it is not MoveIt Servo. Several hundred
  milliseconds can pass between a scene change and a new trajectory.
- One fixed camera cannot see behind the arm or behind objects. The self-filter
  removes the robot, not the occlusion; use a static scene and a conservative
  map where the camera cannot see.
- A hand-eye error of a few centimetres leaks the arm into the Octomap. Verify
  the point cloud/model alignment and recalibrate after any camera bump.
- `move_group` can crash while shutting down on MoveIt 2.5.10. This does not
  affect the run before shutdown.
- The arm is held by the firmware when the control stack deactivates; support
  it if gravity is not the only load.
