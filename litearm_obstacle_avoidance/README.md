English | [简体中文](README.zh-CN.md)

`litearm_obstacle_avoidance` plans the LiteArm seven-axis arm with a LiteGrip
gripper attached, so the gripper, the workcell, and the surrounding obstacles
take part in MoveIt collision checking. It is for engineers validating
avoidance behaviour before running on the real arm: the package ships a dry-run
stack on mock hardware, a Gazebo Classic stack, a data-driven static scene, an
execution supervisor, and acceptance probes for four stages. It does not
implement planning itself; kinematics, joint limits, and the OMPL pipeline come
from `litearm_moveit_config`.

All commands below run from the workspace root, `~/litearm_ws`.

## Requirements

| Item | Requirement |
| --- | --- |
| OS | Ubuntu 22.04 (jammy), x86_64 |
| ROS 2 | Humble Hawksbill |
| Workspace packages | `litearm_moveit_config`, `litearm_ros2_control`, `litearm_description`, `litegrip_description`, `litearm-cpp` |
| Arm | not required for the dry-run stack |
| Display | optional; RViz needs one, the probes do not |
| Gazebo | Classic and `gazebo_ros2_control`, for `obstacle_gazebo.launch.py` only |

## 1. Build the workspace

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

**Do not** add `--symlink-install`. The workspace pulls the `litearm-cpp` SDK
into `litearm_ros2_control` with `EXCLUDE_FROM_ALL`, so the SDK's CLI executable
is never built, while its install rule still runs; the symlink install then
fails with `ament_cmake_symlink_install_targets() can't find`. Plain
`colcon build` builds every package in the workspace.

## 2. Run the dry-run stack

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py
```

This starts the mock control stack, MoveIt, the static scene, and RViz. Wait
until the log reports
`Configured and activated joint_trajectory_controller` before sending a goal:
the action server exists only after the spawner activates it.

In RViz, use the **MotionPlanning** panel to set a goal and plan or execute.
For a headless run, add `use_rviz:=false` and drive the stack with the probe:

```bash
ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py use_rviz:=false
```

In a second terminal:

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
python3 "$(ros2 pkg prefix litearm_moveit_config)/lib/litearm_moveit_config/moveit_probe.py" --execute
```

Arguments understood by `obstacle_fake.launch.py`:

| Argument | Default | Effect |
| --- | --- | --- |
| `use_rviz` | `true` | start RViz with the MoveIt layout |
| `use_synthetic_cloud` | `false` | publish `/synthetic_cloud` and enable Octomap |
| `scene_file` | `config/scene.yaml` | static workcell; pass `''` to load no world objects |
| `start_supervisor` | `false` | start `avoid_supervisor.py` |
| `gripper_envelope_margin` | `0.01` | extra margin around the finger envelope, in metres |
| `ros_domain_id` | `42` | ROS domain for every node |
| `ros_localhost_only` | `true` | keep discovery on this machine |

## 3. Run the Gazebo simulation

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litearm_obstacle_avoidance obstacle_gazebo.launch.py
```

For a headless run:

```bash
ros2 launch litearm_obstacle_avoidance obstacle_gazebo.launch.py \
    gui:=false use_rviz:=false start_supervisor:=true
```

The Gazebo stack replaces `mock_components` with the Gazebo Classic physics
engine. Controller names, the JTC action, and the probe interfaces stay the
same, so `acceptance_fake.sh` probes can be reused against it.

The world geometry in `worlds/obstacle_avoidance.world` must match
`config/scene.yaml`. If you move a fixture in one file, move it in the other;
otherwise Gazebo and MoveIt disagree about where the obstacles are.

## Launch files

| Launch file | Starts | Use it when |
| --- | --- | --- |
| `fake_control.launch.py` | `robot_state_publisher`, `ros2_control_node` on `mock_components/GenericSystem`, joint state broadcaster, joint trajectory controller | you only need the execution chain; no MoveIt, no RViz |
| `obstacle_demo.launch.py` | `move_group` and optional RViz with the combined arm and gripper model | you want planning only, or a control stack is already running |
| `obstacle_fake.launch.py` | `fake_control.launch.py` plus `obstacle_demo.launch.py`; optional synthetic cloud and supervisor | full dry run: plan, execute, replan, no arm |
| `obstacle_gazebo.launch.py` | Gazebo Classic, `robot_state_publisher`, `spawn_robot.py`, controller spawners, `move_group`, optional RViz | physics simulation |

`obstacle_fake.launch.py` includes the other two launch files, so its mock
hardware and MoveIt share one robot description, one SRDF, and one set of
planning parameters. `obstacle_gazebo.launch.py` is standalone because the
Gazebo model adds a fixed `world` link that MoveIt must not see twice.

## Acceptance probes

The package has four staged probes. Each one runs against a live stack and
exits `0` only when every check passes.

| Probe | Stage | Checks |
| --- | --- | --- |
| `scene_probe.py` | 1 | scene objects are in the PlanningScene; every planned trajectory point passes collision checking; a goal wrapped in a box makes planning fail |
| `margin_probe.py` | 2 | the finger envelope covers the open-gripper travel; a small box just outside the envelope edge still collides, proving the margin is applied |
| `replan_probe.py` | 3 | an obstacle injected mid-execution triggers cancel and replan; a blocked goal triggers a low-speed retreat or a safe stop |
| `safety_probe.py` | 4 | stale `/joint_states` makes the supervisor refuse new motion and publish no trajectory |

Supporting programs:

| Program | Role |
| --- | --- |
| `scene_builder.py` | reads `scene.yaml` and writes the objects into the PlanningScene through `/apply_planning_scene` |
| `synthetic_cloud.py` | publishes a box-surface point cloud on `/synthetic_cloud` when no camera is available |
| `avoid_supervisor.py` | plans, executes, cancels, and replans; retreats or stops on failure |
| `spawn_robot.py` | inserts the robot into Gazebo without the conda `lxml` problem in `gazebo_ros/spawn_entity.py` |
| `acceptance_fake.sh` | starts the dry-run stack, waits for readiness, runs the selected probes, and stops the stack |

Run all four stages with one command:

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run litearm_obstacle_avoidance acceptance_fake.sh --scene --margin --replan --safety
```

| Flag | Adds |
| --- | --- |
| none | plan and execute to the ready pose only |
| `--cloud` | synthetic point cloud and Octomap readiness check |
| `--scene` | stage 1 scene probe |
| `--margin` | stage 2 margin probe |
| `--replan` | stage 3 replan probe; starts the supervisor |
| `--safety` | stage 4 stale-data probe; starts the supervisor |

**Do not** run `acceptance_fake.sh` while another stack uses the same ROS
domain. The script starts its own stack and stops it on exit; two
`/controller_manager` or `/move_group` instances in one domain make the probes
report false failures. The script log is `/tmp/litearm_acceptance_fake.log`.

## Static scene

`config/scene.yaml` describes the workcell in the fixed `base_link` frame. It
is data, not code: edit the file to change the cell, then restart the launch or
re-run `scene_builder.py`.

| Object | Shape | Size | Pose in `base_link` |
| --- | --- | --- | --- |
| `workbench` | box | 0.90 x 0.90 x 0.10 m | (0.10, 0.0, -0.07) |
| `fixture_pillar` | cylinder | radius 0.04 m, height 0.28 m | (0.30, -0.25, 0.12) |
| `fixture_block` | box | 0.12 x 0.12 x 0.12 m | (-0.30, 0.15, 0.04) |

Each object sets `padding` to 0.01 m. The field inflates the primitive, because
`moveit_msgs/CollisionObject` has no padding field in Humble. The workbench top
sits at z = -0.02 m and the padded top at z = -0.01 m, which leaves 1 cm below
the base so the start state is not in collision.

This is a test cell, not a measured production cell. Replace the dimensions
and poses with your own before drawing conclusions about a real workcell. If a
fixture blocks the acceptance goal, planning fails correctly.

## Gripper model and envelope

The combined model is built from three entry points:

| File | Hardware block |
| --- | --- |
| `config/litearm_litegrip.urdf.xacro` | real arm through `litearm_ros2_control` |
| `config/litearm_litegrip_fake.urdf.xacro` | `mock_components/GenericSystem` for the dry run |
| `config/litearm_litegrip_gazebo.urdf.xacro` | `gazebo_ros2_control/GazeboSystem` plus a fixed `world` link |

All three include `config/litearm_litegrip.geometry.urdf.xacro`, so the arm
geometry, the gripper mounting transform, and the envelope are maintained in
one place. Gripper link names use the `litegrip_` prefix to avoid colliding
with the arm's `base_link`.

The default mounting offset is 0.0538 m from `ee_link`. It assumes the 10.3 mm
end cap is removed and the gripper boss occupies that space. If your hardware
keeps the cap or uses an adapter plate, override the value:

```bash
xacro config/litearm_litegrip.geometry.urdf.xacro \
    gripper_mount_xyz:="0 0 0.0641" gripper_mount_rpy:="0 0 0"
```

The gripper has no state source in this stack. MoveIt therefore treats
`gripper_opening_joint` as 0, the closed pose, while the physical gripper may be
open. `config/litearm_litegrip.geometry.urdf.xacro` compensates with a
collision-only envelope that covers the full finger travel plus
`gripper_envelope_margin`, 0.01 m by default. `config/litearm_litegrip.srdf`
disables the collision pairs that are structural contacts or that the envelope
is designed to contain.

**Do not** pass `include_gripper_envelope:=false` without also editing the
SRDF. The SRDF references the envelope links in `disable_collisions`; MoveIt
rejects a reference to a link that no longer exists.

The planning groups are `litearm_arm` from `base_link` to `ee_link`, and
`gripper` with the single active joint `gripper_opening_joint`.

## Execution supervisor

`avoid_supervisor.py` runs the stop, replan, continue loop. It is not MoveIt
Servo: it cancels the current trajectory, plans from the current state, and
resumes. When replanning fails it plans a low-speed retreat to the ready pose;
if that also fails it stops where it is.

| Interface | Type | Direction |
| --- | --- | --- |
| `/avoid_supervisor/goal` | `sensor_msgs/JointState` | input; seven arm joints |
| `/avoid_supervisor/status` | `std_msgs/String` JSON | output; `state`, `replans`, `busy`, `reason` |
| `/avoid_supervisor/planned_trajectory` | `trajectory_msgs/JointTrajectory` | output; every new plan |
| `/plan_kinematic_path` | `moveit_msgs/GetMotionPlan` | service client |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | action client |

The state machine moves through `idle`, `planning`, `executing`, and
`succeeded`; `canceling` and `replanning` when the scene changes; and
`retreating`, `retreated`, `safe_stop`, or `failed` on error paths.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `group_name` | `litearm_arm` | planning group |
| `velocity_scaling` | `0.15` | speed for normal execution |
| `acceleration_scaling` | `0.15` | acceleration for normal execution |
| `planning_time` | `2.0` s | planning budget per attempt |
| `plan_attempts` | `5` | attempts per plan request |
| `max_replans` | `3` | replans before the supervisor gives up |
| `goal_tolerance` | `0.06` rad | convergence tolerance |
| `max_state_age` | `1.0` s | refuse motion when `/joint_states` is older |
| `sensor_topic` | empty | optional point-cloud topic for a sensor-age check |
| `max_sensor_age` | `0.0` s | 0 disables the sensor-age check |
| `retreat_enabled` | `true` | allow a low-speed retreat after failure |
| `retreat_positions` | ready pose | seven joint values for the retreat |
| `retreat_velocity_scaling` | `0.1` | speed for the retreat |

Start it with the dry-run stack:

```bash
ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py start_supervisor:=true
```

## Environment isolation

Every launch file sets `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1` for its
nodes. This is deliberate: it keeps the test stack away from domain 0 and from
other robots on the network. Pass `ros_domain_id:=` or `ros_localhost_only:=`
to change it.

In a terminal that talks to the stack, set the same values before you run any
`ros2` command:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

`acceptance_fake.sh` uses the same defaults and passes them to the launch file.

## Known limitations

- The dry-run stack has no dynamics. State equals command, so the execution
  chain is tested but control quality is not. Real behaviour must be verified
  on the arm.
- Gazebo Classic does not support URDF mimic. The two gripper fingers are
  passive there; avoidance relies on the MoveIt-side envelope.
- The gripper has no state source. Collision checks use the conservative
  envelope, not the actual opening.
- `scene.yaml` is a test workcell. It is not calibrated to a production cell.
- The supervisor is stop, replan, continue. It is not a real-time avoidance
  controller.
- With MoveIt 2.5.10, `move_group` can crash while shutting down. This is an
  upstream issue and does not affect the checks before shutdown.
- The first synthetic point cloud can log one
  `shape_mask: Missing transform for shape mesh`. The self-filter transform is
  not ready yet; the message does not repeat.
