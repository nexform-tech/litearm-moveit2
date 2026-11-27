**English** | [简体中文](README.zh-CN.md)

# litearm_dual_arm

MoveIt 2 integration for two LiteArms. The package derives a dual-arm URDF/SRDF
from the single-arm description in `litearm_description`, exposes the
`left_arm`, `right_arm`, and `both_arms` planning groups from one `move_group`,
and keeps both arms in a single shared Planning Scene.

> **Status: deterministic simulation execution and the real-machine entry are in
> place.** The package provides the dual-arm model, the shared scene, the three
> groups, mutual collision checks, and `dual_arm_bridge.py` with two action
> servers. `dual_arm_real.launch.py` connects two `litearm_ros2_control`
> `side:=left|right` stacks; it is plan-only by default, and execution requires
> authorization, estop, workspace, and calibration gates. There is no Gazebo
> physics engine. No gripper is included; the chain tips are `left_ee_link` /
> `right_ee_link`.

## 1. What it provides

- **One model** — two prefixed LiteArm chains under the same `world` frame,
  derived from the single-arm URDF. No duplicated meshes, no second
  hand-maintained description.
- **One Planning Scene** — both arms and the static workcell live in the same
  `move_group`. Planning one arm keeps the other arm in the scene as an
  obstacle, so the two-arm stack cannot lose mutual collision checking.
- **Three groups** — `left_arm`, `right_arm`, and `both_arms`. `both_arms` is a
  14-DOF group for simultaneous planning.
- **Mutual collision avoidance** — the SRDF only disables adjacent pairs inside
  each arm; every left/right link pair stays enabled. Trajectory samples are
  re-checked with `/check_state_validity`.
- **Deterministic execution bridge** — `dual_arm_bridge.py` splits the MoveIt
  14-joint trajectory into left/right 7-joint goals and forwards them to two
  namespaced JTC actions. A failure, cancellation, or timeout on either side
  cancels the peer and enters fail-closed. The offline simulator exposes the same
  two-action interface as the real side stacks.
- **Synchronized carry task** — `task:=carry` moves a virtual tray between two
  tool frames with no gripper: IK, `both_arms` start planning, two Cartesian
  lines, axis-aligned synchronization, per-sample collision/geometry validation,
  and dual-arm execution metrics.
- **Alternating operation task** — `task:=alternating` lets the arms take turns
  entering the shared workspace while the idle arm holds position and stays in
  the full-robot collision check; single-arm executions also verify the idle-arm
  deviation.

## 2. Build

From the workspace root:

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select litearm_dual_arm
source install/setup.bash
```

Do **not** use `--symlink-install`; `litearm_ros2_control` fails under it in this
workspace.

The package expects installed `litearm_description` and
`litearm_moveit_config`. The generator locates them through the ament index, and
also accepts explicit paths.

Script shebangs are pinned to `/usr/bin/python3`, the system Python 3.10 used by
ROS Humble. If a conda `python3` (3.11) is first on `PATH`, `ros2 run` still uses
the shebang; run scripts manually with `/usr/bin/python3` or they fail with an
`rclpy_pybind11` ABI mismatch.

## 3. Offline planning and deterministic execution

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py
```

Nodes:

| Node | Purpose |
| --- | --- |
| `robot_state_publisher` | dual-arm TF, root frame `world` |
| `move_group` | three groups plus the shared Planning Scene |
| `litearm_dual_arm_sim` | two side state topics plus two JTC action servers |
| `litearm_dual_arm_bridge` | state merge, 14→7+7 trajectory split, fail-closed |
| `litearm_dual_arm_scene` | applies the `dual_static` workcell |
| `rviz2` | dual-arm MotionPlanning panel |

In RViz:

1. select `both_arms` in MotionPlanning → Planning Group;
2. set a goal and press **Plan**;
3. press **Execute**: MoveIt sends the trajectory through the bridge to the two
   deterministic action servers;
4. switch to `left_arm` / `right_arm` to verify that a single-arm execution
   leaves the other arm still.

Planning only:

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py execute:=false
```

This is a deterministic simulator, not Gazebo: no gravity, contact forces, or
tracking error. It validates the execution chain, timestamps, synchronization,
and failure propagation.

Headless:

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py use_rviz:=false
```

Without the static scene:

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py scene:=none
```

### 3.1 Synchronized carry

The first carry version needs no gripper: the two tool frames are virtual tray
supports, both with tool Z pointing up, and the tray rests on the flanges from
below. The task:

1. derives the start configuration from `dual_carry.yaml`;
2. plans `both_arms` to the start state and validates the path;
3. computes one Cartesian line per arm;
4. aligns both paths by the real tool coordinate along the carry axis and
   combines them into a 14-joint synchronized trajectory;
5. checks collisions, separation, plane, position, and orientation at every
   sample;
6. executes both arms through the bridge and writes a metrics JSON.

```bash
# plan + simulated execution; writes /tmp/litearm_dual_carry_metrics.json
ros2 launch litearm_dual_arm dual_arm_sim.launch.py task:=carry

# plan only
ros2 launch litearm_dual_arm dual_arm_sim.launch.py task:=carry execute:=false

# one-command acceptance
ros2 run litearm_dual_arm acceptance_carry.sh
ros2 run litearm_dual_arm acceptance_carry.sh --plan-only
```

The task scene is `dual_carry_static` (worktable only). With `task:=carry` the
generic scene node is not started, so the two scenes cannot stack.

Configuration lives in [dual_carry.yaml](config/dual_carry.yaml):
`start_center`, `direction`, `distance_m`, `hand_separation_m`,
`separation_axis`, the tool quaternions, the validated `start_left_rad` /
`start_right_rad`, motion, and tolerances.

The stored start joints are validated for the current two-arm yaw=+90° mounts;
startup checks them against the target tool poses with FK. After changing the
mounts, workcell, or tool, recompute IK and update those values.

Measured in the current simulation: start plan and 20-point validation pass;
both Cartesian fractions are 1.0; 41 synchronized samples have no collisions,
separation error about 2e-6 m and position error about 2e-5 m; dual execution
succeeds with about 3 ms synchronization error.

### 3.2 Alternating operation in the shared workspace

`task:=alternating` follows [dual_alternating.yaml](config/dual_alternating.yaml):

```text
both_arms to ready
  → left_arm enters the shared workspace
  → left_arm retreats
  → right_arm enters the shared workspace
  → right_arm retreats
  → both_arms to ready
```

A single-arm stage sends a trajectory only to the active side. The other arm
holds its current state and remains part of the full-robot
`/check_state_validity` check. Execution also verifies the idle-arm deviation.

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py task:=alternating
ros2 launch litearm_dual_arm dual_arm_sim.launch.py \
  task:=alternating execute:=false

ros2 run litearm_dual_arm acceptance_alternating.sh
ros2 run litearm_dual_arm acceptance_alternating.sh --plan-only
```

Targets come from `dual_goals.yaml` (`left_safe`, `right_safe`, `left_ready`,
`right_ready`, `both_ready`); the scene is `dual_alternating_static` (worktable).
All six stages plan, validate, and execute successfully; idle-arm deviation is
0, and dual-arm stage synchronization error is about 3–10 ms.

## 4. Acceptance probe

`dual_arm_probe.py` plans through `/plan_kinematic_path`, samples the trajectory,
and calls `/check_state_validity` at every sample:

```bash
ros2 run litearm_dual_arm dual_arm_probe.py --target both_safe
ros2 run litearm_dual_arm dual_arm_probe.py --target left_safe
ros2 run litearm_dual_arm dual_arm_probe.py --target right_safe --samples 31
```

Plan and execute directly through the bridge:

```bash
ros2 run litearm_dual_arm dual_arm_probe.py --target both_safe --execute
```

Validate the RViz/MoveIt Execute path (simple controller manager → bridge):

```bash
ros2 run litearm_dual_arm dual_arm_probe.py \
  --target both_ready --via-move-action
```

Success requires:

- `planning.success = true`
- `validation.invalid_indices = []`
- `validation.contacts = []`
- `execution.success = true` with `--execute` or `--via-move-action`
- `success = true`

The negative case `mutual_collision_case` only checks the target state, without
planning:

```bash
ros2 run litearm_dual_arm dual_arm_probe.py \
  --target mutual_collision_case --check-state
```

Here `success=true` means the opposite: MoveIt must reject the state and
`cross_arm_contacts` must contain `left_*` ↔ `right_*` pairs. That is the
negative proof that mutual collision checking is active.

The default JSON output is `/tmp/litearm_dual_arm_probe.json`. Targets live in
`config/dual_goals.yaml`; they are acceptance goals, not business tasks.

## 5. Model generation

The launch calls the generator automatically. To inspect the output:

```bash
ros2 run litearm_dual_arm generate_dual_arm_model.py --mode urdf
ros2 run litearm_dual_arm generate_dual_arm_model.py --mode srdf
ros2 run litearm_dual_arm generate_dual_arm_model.py --mode both
```

Rules:

- every single-arm `link`/`joint` gets a `left_` or `right_` prefix;
- `world` plus fixed `left_world_joint` / `right_world_joint` mounts;
- SRDF groups `left_arm`, `right_arm`, `both_arms`;
- named states `left_zero`/`right_zero`/`both_zero` and the matching `ready`
  states;
- `disable_collisions` only for intra-arm adjacent pairs and
  `world ↔ base_link`; no left/right pair is ever disabled.

## 6. Mount calibration

The defaults in `config/dual_arm_mounts.yaml` are bench references:

```text
left  base_link: [0.0,  0.38, 0.0]
right base_link: [0.0, -0.38, 0.0]
```

The arms stand side by side, 0.76 m apart, reaching toward +X. These are **not**
measurements. Before real-machine execution, measure each base in `world` and
set `calibration.measured: true`.

After changing the mounts, recompute the FK/collision checks behind
`dual_goals.yaml`; do not reuse the old targets. The current `tool.type` is
`none`. A cylindrical tool can be enabled later with measured dimensions; the
generator then moves the SRDF chain tip to `tool_link`.

## 7. Tests

```bash
colcon test --packages-select litearm_dual_arm --event-handlers console_direct+
colcon test-result --verbose
```

Full acceptance (start the stack, execute four targets, run the MoveGroup path,
run the collision negative case, verify that a fail-closed stop rejects new
goals, stop it):

```bash
ros2 run litearm_dual_arm acceptance_dual_arm.sh
ros2 run litearm_dual_arm acceptance_dual_arm.sh --scene none
TARGETS="both_safe left_safe" ros2 run litearm_dual_arm acceptance_dual_arm.sh

# synchronized carry
ros2 run litearm_dual_arm acceptance_carry.sh
ros2 run litearm_dual_arm acceptance_carry.sh --plan-only

# alternating operation
ros2 run litearm_dual_arm acceptance_alternating.sh
ros2 run litearm_dual_arm acceptance_alternating.sh --plan-only
```

Coverage:

- model: prefixes, `world` mounts, three groups, named states, cross-arm
  collision pairs, optional tool;
- config: OMPL groups, controller mapping, target joint sets, joint limits,
  scene references;
- execution bridge: 14→7+7 name mapping, velocity/acceleration/timestamp
  preservation, simulator initial-pose parsing;
- carry: configuration geometry, quaternion error, path interpolation,
  14-joint synchronized trajectory, and start-geometry validation;
- alternating: stage order, target existence, and target-group/active-side
  consistency;
- launch: argument set, `execute` gate, `task` argument, and the bridge node.

## 8. Real-machine entry

Prerequisites:

- `litearm_ros2_control` supports `side:=left|right`;
- both bases are measured and `dual_arm_mounts.yaml` has
  `calibration.measured: true`;
- the two USB devices are selected by `/dev/serial/by-id/`;
- the physical emergency stop is in place and the workspace is clear.

### 8.1 External side stacks, plan only

```bash
# terminal 1: left
ros2 launch litearm_ros2_control litearm_control.launch.py \
  side:=left port:=/dev/serial/by-id/<left-arm>

# terminal 2: right
ros2 launch litearm_ros2_control litearm_control.launch.py \
  side:=right port:=/dev/serial/by-id/<right-arm>

# terminal 3: dual-arm MoveIt + bridge, plan only
ros2 launch litearm_dual_arm dual_arm_real.launch.py
```

`execute:=false` is the default: the bridge merges states only and
`move_group` cannot execute. `preflight:=true` starts a read-only check of both
controller managers, JSB/JTC, state rate, actions (when executing), and TF; the
JSON lands in `/tmp/litearm_dual_arm_preflight.json`.

Then run a plan-only probe:

```bash
ros2 run litearm_dual_arm dual_arm_probe.py \
  --target both_safe --samples 20
```

### 8.2 Start both side stacks from this launch

```bash
ros2 launch litearm_dual_arm dual_arm_real.launch.py \
  start_control:=true \
  left_port:=/dev/serial/by-id/<left-arm> \
  right_port:=/dev/serial/by-id/<right-arm>
```

With `start_control:=true` both ports are mandatory. The launch passes
`start_robot_state_publisher:=false` to both sides; the dual-arm model publishes
TF once.

### 8.3 Gated execution

```bash
ros2 launch litearm_dual_arm dual_arm_real.launch.py \
  start_control:=true \
  left_port:=/dev/serial/by-id/<left-arm> \
  right_port:=/dev/serial/by-id/<right-arm> \
  execute:=true \
  authorization:=I_UNDERSTAND_DUAL_ARM_MOTION \
  estop_verified:=true \
  workspace_verified:=true
```

`execute:=true` also requires:

- `authorization:=I_UNDERSTAND_DUAL_ARM_MOTION`;
- `estop_verified:=true`;
- `workspace_verified:=true`;
- `calibration.measured: true` in `dual_arm_mounts.yaml`.

Start with one arm, a small motion, and low speed; then move to both arms. The
real launch does not replace the physical emergency stop; the bridge's
`/litearm_dual_arm/stop` is a fail-closed software request.

The real scene defaults to `scene:=none`. `dual_static` is a bench reference;
pass it only after confirming it matches the real workcell.

## 9. Next steps

1. Run the real side stacks through `execute:=false` preflight and plan-only.
2. Low-speed single arm → low-speed both arms → site calibration and acceptance.
3. Business tasks such as plate carry and alternating operation; optional
   Gazebo/Octomap and a dual-arm supervisor.

## 10. Notes

- The real entry is plan-only by default; only `execute:=true` moves hardware.
  Simulation acceptance is not real-machine acceptance.
- `/litearm_dual_arm/stop` is fail-closed: after it fires, new trajectories are
  rejected until the bridge is restarted and the cause is checked.
- With two arms sharing a workspace, do not drive them from two independent
  single-arm MoveIt stacks; neither stack sees the other arm.
- The physical emergency stop remains the independent safety measure. A
  software stop service is not a substitute.
