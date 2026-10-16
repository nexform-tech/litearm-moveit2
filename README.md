# litearm-moveit2

MoveIt 2 integration on ROS 2 for the **LiteArm robotic manipulator series**.

**English** · [简体中文](README.zh-CN.md)

> **Status:** the `litearm_moveit_config` package has landed. It has no automated
> test suite yet; the acceptance probe in the quickstart is what checks it.

## Quickstart

This walks you from an empty machine to a LiteArm you can drag in RViz: create a
colcon workspace, put the packages in `src/`, build from the workspace root, and
launch. Pick the track that matches your machine — both end at the same commands.

| Track | Use it when |
| --- | --- |
| [Quickstart with Docker](#quickstart-with-docker) | ROS 2 is not installed on the host, or you would rather not install it |
| [Quickstart without Docker](#quickstart-without-docker) | ROS 2 Humble is already installed on the host |

### What you need

| Item | Requirement |
| --- | --- |
| OS | Ubuntu 22.04 (jammy), x86_64 |
| ROS 2 | Humble |
| Docker | any recent version, for the container track only |
| Disk | about 8 GB for the container track, about 4 GB for the host track |
| Display | optional — RViz needs one, the acceptance probe does not |
| Arm | optional — the planning demo runs with no arm attached |

### The workspace layout

Both tracks produce this layout, because both clone into `src/` and build from
the workspace root:

```text
~/litearm_ws/          <- workspace root: run every colcon command here
├── src/               <- the packages go here, and only here
│   ├── litearm-cpp/
│   ├── litearm-description/
│   ├── litearm-moveit2/
│   └── litearm-ros2-control/
├── build/             <- colcon output; never edit, never commit
├── install/
└── log/
```

**Do not run `colcon build` from inside `src/`.** It has to run in the workspace
root, so that it finds `src/` and writes `build/`, `install/` and `log/` beside
it. Run it from `src/` and you get a workspace nested inside your source tree,
after which every `source install/setup.bash` resolves to the wrong prefix.

### What goes in `src/`

Four repositories. All four are needed even for the planning demo:

| Cloned into `src/` | Package it provides | Why it is needed |
| --- | --- | --- |
| `litearm-moveit2` | `litearm_moveit_config` | this repository: the SRDF, the planning pipeline, the controller mapping and the launch files |
| `litearm-description` | `litearm_description` | the URDF and the meshes; the package sits at `SE/litearm_description` |
| `litearm-ros2-control` | `litearm_ros2_control` | the ros2_control plugin, and the URDF entry point that includes it |
| `litearm-cpp` | none — plain CMake, no `package.xml` | the SDK the plugin links against |

That the planning demo needs the ros2_control package surprises people: the URDF
this package launches is a thin wrapper that includes
`$(find litearm_ros2_control)/urdf/litearm.urdf.xacro`. Without that package the
description does not resolve and RViz loads an empty robot.

**Do not add the firmware repository to the same workspace.** `litearm-stm32`
ships its own `litearm` URDF package, and two arm descriptions in one workspace
make it ambiguous which one a launch resolves.

## Quickstart with Docker

### 1. Build the run image

No image for this stack is published, so build one. It is the public
`osrf/ros:humble-desktop-full` image plus the packages this workspace needs:
`desktop-full` ships RViz, xacro and colcon, but no MoveIt and no ros2_control.

```bash
docker build -t litearm-moveit:humble - <<'EOF'
FROM osrf/ros:humble-desktop-full
RUN apt-get update && apt-get install -y \
      ros-humble-moveit \
      ros-humble-moveit-configs-utils \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-ros2controlcli \
 && rm -rf /var/lib/apt/lists/*
EOF
```

Two things in that list are deliberate:

- **`ros-humble-moveit-configs-utils` is named separately.** The `ros-humble-moveit`
  metapackage does not depend on it, but `litearm_moveit.launch.py` imports
  `MoveItConfigsBuilder` from it, and the launch fails on that import without it.
- **There is no `--no-install-recommends`.** For the MoveIt metapackage the
  recommends carry runtime plugins and planner configs, and a missing one does not
  fail at startup — it fails later, as a silent no-op inside `move_group`.

This takes a few minutes and produces an image of about 5.6 GB.

### 2. Create the workspace and clone the packages (on the host)

Clone on the host, not inside the container: the workspace is a bind mount, so it
outlives `--rm`, and you keep your clone and your editor outside Docker.

```bash
mkdir -p ~/litearm_ws/src
cd ~/litearm_ws/src
git clone https://github.com/nexform-tech/litearm-cpp.git
git clone https://github.com/nexform-tech/litearm-description.git
git clone https://github.com/nexform-tech/litearm-moveit2.git
git clone https://github.com/nexform-tech/litearm-ros2-control.git
```

The four packages are described [above](#what-goes-in-src).

### 3. Build the workspace in the container

```bash
docker run --rm -it \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$HOME/litearm_ws:/ws" \
  litearm-moveit:humble bash -lc '
    cd /ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    echo "build ok"'
```

`--user "$(id -u):$(id -g)"` and `-e HOME=/tmp` keep the container from writing
root-owned files into your workspace. Without them `build/` and `install/` come
back owned by root, and the next host-side edit or `git clean` fails on
permission.

**Do not add `--symlink-install`.** It fails in this workspace:

```text
CMake Error at ament_cmake_symlink_install/ament_cmake_symlink_install.cmake:272 (message):
  ament_cmake_symlink_install_targets() can't find
  '/ws/build/litearm_ros2_control/litearm-cpp/litearm-cpp'
```

`litearm_ros2_control` pulls the `litearm-cpp` source tree in with
`add_subdirectory(... EXCLUDE_FROM_ALL)`, but `litearm-cpp` registers an
unconditional `install(TARGETS litearm-cpp ...)` for its command-line tool. Plain
CMake skips installing a target that was never built; ament's symlink-install
macros assert the file exists instead. Plain `colcon build` is what this
workspace supports.

### 4. Run the planning demo in the container

```bash
docker run --rm -it \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  --net=host -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/litearm_ws:/ws" \
  litearm-moveit:humble bash -lc '
    cd /ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch litearm_moveit_config demo.launch.py'
```

RViz opens with the MotionPlanning panel. Drag the interactive marker and press
**Plan** — see [Planning in RViz](#planning-in-rviz).

If RViz cannot open a window, the container has no access to your X server. Grant
your own user, and only your own user:

```bash
xhost +si:localuser:$(id -un)
```

Without a display, run the acceptance probe instead. It starts the same stack,
plans to `ready`, prints what it checked, and tears the stack down:

```bash
docker run --rm -it \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$HOME/litearm_ws:/ws" \
  litearm-moveit:humble bash -lc '
    cd /ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run litearm_moveit_config acceptance_moveit.sh'
```

## Quickstart without Docker

### 1. Install ROS 2 Humble

Follow the official instructions for Ubuntu 22.04 (jammy) and choose the
`ros-humble-desktop` variant:

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

Then add what this workspace needs on top of it:

```bash
sudo apt install ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros2controlcli \
  ros-dev-tools
```

`ros-humble-moveit-configs-utils` is named separately because the `ros-humble-moveit`
metapackage does not depend on it, and `litearm_moveit.launch.py` imports
`MoveItConfigsBuilder` from it.

**`colcon` is not in Ubuntu's own repositories.** On jammy,
`sudo apt install colcon` fails with `E: Unable to locate package colcon` and
`sudo apt install python3-colcon-common-extensions` fails the same way — colcon
ships from the ROS 2 apt repository, and `ros-dev-tools` is the package that
pulls it in. There is no colcon to install until ROS 2 itself is installed.

For the real arm only, grant yourself access to the serial port:

```bash
sudo usermod -aG dialout "$USER"    # takes effect after you log in again
```

### 2. Create the workspace and clone the packages

```bash
mkdir -p ~/litearm_ws/src
cd ~/litearm_ws/src
git clone https://github.com/nexform-tech/litearm-cpp.git
git clone https://github.com/nexform-tech/litearm-description.git
git clone https://github.com/nexform-tech/litearm-moveit2.git
git clone https://github.com/nexform-tech/litearm-ros2-control.git
```

The four packages are described [above](#what-goes-in-src).

### 3. Build the workspace

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

As in the container track, build from `~/litearm_ws`, never from `src/`, and do
not pass `--symlink-install`. The reason is [above](#3-build-the-workspace-in-the-container).

### 4. Run the planning demo

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litearm_moveit_config demo.launch.py
```

## Planning in RViz

`demo.launch.py` starts MoveIt with `start_control:=false`: the arm plans in RViz,
and Execute needs a control stack, which needs an arm on the other end of the USB
cable. This deployment has no dry-run mode — the hardware component talks to the
real board through the litearm C++ SDK.

In the MotionPlanning panel the planning group is `litearm_arm`. Drag the
interactive marker, or pick one of the SRDF's named states and press Plan &
Execute:

| Named state | Pose |
| --- | --- |
| `zero` | all seven joints at zero — the firmware zero, and the pose after power-up |
| `ready` | elbow-up configuration; use this as the planning start, `zero` is near a singularity and Cartesian planning fails there |

⚠ The stack is single-instance, and the serial port is what makes it so: the
component owns the arm's USB CDC link and the SDK takes an exclusive `flock` on
it, so a second control stack fails to open the port and its hardware component
fails to configure. `move_group` still comes up and still plans — planning does
not need the hardware, execution does — so the failure shows up as a missing
controller rather than as a dead launch. Stop the other stack first.

## Running on the real arm

On the host, this is the whole command:

```bash
ros2 launch litearm_moveit_config litearm_moveit.launch.py
```

⚠ This one moves the arm. Power the board, connect USB, activate the license,
support the arm and keep the e-stop within reach.

In the container track it needs the board passed through, plus the group that
owns it on the host:

```bash
docker run --rm -it \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  --device /dev/ttyACM0 \
  --group-add "$(getent group dialout | cut -d: -f3)" \
  --net=host -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/litearm_ws:/ws" \
  litearm-moveit:humble bash -lc '
    cd /ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch litearm_moveit_config litearm_moveit.launch.py'
```

Check the board is there before you launch:

```bash
lsusb | grep 1d50          # want 1d50:606f
ls /dev/ttyACM*            # want /dev/ttyACM0
```

Add `start_control:=false` when a control stack is already running, so that two
`controller_manager`s do not fight over the same command interfaces. Empty `port`
(the default) auto-discovers the board by VID:PID `1d50:606f`; pass
`port:=/dev/ttyACM1` when several are attached.

This launch also clears a **latched fault** on the arm while configuring the hardware
(`clear_faults`, on by default). The firmware latches an emergency stop or a joint fault
and then refuses ENABLE until it is reset, so without this a bring-up after a fault needs
a separate `litearm_driver` session first. The reset happens with the link up and nothing
enabled or streamed, and it is logged as a warning; if the cause is still there the
firmware latches the fault again and ENABLE fails with a message that names it. Pass
`clear_faults:=false` when you want a latched fault to stop the launch — after a crash,
for instance, so you can inspect the arm before anything is enabled.

## Reaching the stack from another terminal

Both launch files pin the ROS domain to 42 and enable localhost-only discovery,
so nothing in a plain shell sees the stack until it exports the same settings:

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1
```

`ros_domain_id:=0 ros_localhost_only:=false` joins the default graph instead. Do
not do that on a shared network: a stranger's `move_group` can pick up your goal,
fail it against controllers it does not have, and leave the result looking like a
hardware fault.

If a `ros2` command still sees nothing, try it with `--no-daemon`: the CLI
otherwise answers from a cached daemon that may hold the discovery settings from
before your export.

## Where to go next

MoveIt 2 tutorials, Humble edition — the distribution this stack targets:

| Tutorial | Covers |
| --- | --- |
| [MoveIt Quickstart in RViz](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) | the panel this launch opens: planning groups, planned paths, the interactive marker |
| [MoveIt Setup Assistant](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html) | how the SRDF, joint limits and kinematics file under `config/` are produced |
| [URDF and SRDF](https://moveit.picknik.ai/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html) | the split between the robot's description and this package's semantic description |
| [Kinematics Configuration](https://moveit.picknik.ai/humble/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html) | `config/kinematics.yaml`, the IK solver and its budget |
| [Low Level Controllers](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html) | how MoveIt hands a trajectory to ros2_control — what `config/moveit_controllers.yaml` implements |
| [Move Group C++ Interface](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html) | planning and executing from your own node instead of RViz |
| [MoveIt Task Constructor](https://moveit.picknik.ai/humble/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html) | multi-stage tasks such as pick and place, the direction `litearm_manipulation` takes |

Full indexes: [tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
and [examples](https://moveit.picknik.ai/humble/doc/examples/examples.html).

## Scope

| | |
| --- | --- |
| Product | LiteArm robotic manipulator series |
| Repository role | MoveIt 2 integration on ROS 2 |
| ROS 2 distribution | Humble Hawksbill |
| Status | Active — `litearm_moveit_config` available |

## Packages

| Package | Role |
| --- | --- |
| `litearm_moveit_config` | MoveIt 2 configuration for the litearm seven-axis arm: SRDF and collision matrix, KDL kinematics, joint limits derived from the firmware parameter table, the OMPL planning pipeline, `moveit_controllers`, RViz layout, launch files, and the acceptance probe |

Two notes on that list, both of which the package's own headers explain at
length:

- **The IK solver is KDL, MoveIt's default.** It ships with MoveIt, so the
  workspace needs no extra package. KDL misses poses that exist — measured on this
  arm, four seeds fail within 705 ms where SNS-IK solves them — so when planning
  looks intermittent rather than geometric, switch to SNS-IK: the recipe, the
  plugin-name trap and the measured hit rates are in `config/kinematics.yaml`.
- **The Pilz pipeline is not loaded.** The package plans with OMPL only.
  `config/pilz_cartesian_limits.yaml` is there so that enabling Pilz needs no new
  file.

## Generated configuration

Three files under `config/` are derived, not written by hand. Each carries a
"do not edit by hand" warning in its own header:

| File | Produced by | Regenerate when |
| --- | --- | --- |
| `config/joint_limits.yaml` | `tools/gen_from_firmware_config.py <litearm-stm32>` | the firmware parameter table changes |
| `config/initial_positions.yaml` | the same | the same |
| the SRDF's `disable_collisions` block | `tools/compute_collision_matrix.py` | the URDF or the meshes change |

The joint limits are the firmware's, deliberately 1° tighter than the URDF's.
MoveIt has to plan inside the envelope the firmware enforces at runtime,
otherwise a plan succeeds and the execution is rejected.

The collision matrix is sampled from the real meshes: the script disables a link
pair only when the pair is structurally adjacent or touches in every sampled
pose. Pairs that merely come close stay checked — those are the ones MoveIt is
supposed to avoid.

## Related repositories

| Repository | Role |
| --- | --- |
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK |
| [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) | C++ SDK |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | Product documentation |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | ROS 2 driver |

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and
automated semantic-release versioning on every merge to `main`.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).
