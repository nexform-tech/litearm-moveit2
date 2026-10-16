# litearm-moveit2

**LiteArm 机械臂**系列在 ROS 2 上的 MoveIt 2 集成。

[English](README.md) · **简体中文**

> **状态：** `litearm_moveit_config` 包已落地。目前还没有自动化测试；快速上手里的
> 验收探针就是它的检查手段。

## 快速上手

这一节带你从一台空机器走到「能在 RViz 里拖动机械臂」：建工作区、把各包放进 `src/`、
在工作区根目录编译、启动。按你的机器选一条路线，两条最后的命令是一样的。

| 路线 | 什么时候用它 |
| --- | --- |
| [用容器快速上手](#用容器快速上手) | 宿主机上没装 ROS 2，或者你不想装 |
| [不用容器快速上手](#不用容器快速上手) | 宿主机上已经装好 ROS 2 Humble |

### 你需要什么

| 项目 | 要求 |
| --- | --- |
| 系统 | Ubuntu 22.04（jammy），x86_64 |
| ROS 2 | Humble |
| Docker | 任意较新版本，只有容器路线需要 |
| 磁盘 | 容器路线约 8 GB，宿主机路线约 4 GB |
| 显示 | 可选 —— RViz 需要，验收探针不需要 |
| 机械臂 | 可选 —— 只规划不动的演示不需要真机 |

### 工作区布局

两条路线产出同一套布局，因为两条都往 `src/` 里克隆、都在工作区根目录编译：

```text
~/litearm_ws/          <- 工作区根目录：所有 colcon 命令都在这里跑
├── src/               <- 各包放这里，也只放这里
│   ├── litearm-cpp/
│   ├── litearm-description/
│   ├── litearm-moveit2/
│   └── litearm-ros2-control/
├── build/             <- colcon 产物；不要手改，不要提交
├── install/
└── log/
```

**不要在 `src/` 里面跑 `colcon build`。** 它必须在工作区根目录执行，这样才能找到
`src/`，并把 `build/`、`install/`、`log/` 写在它旁边。在 `src/` 里跑出来的是一套嵌在
源码树内部的工作区，之后每一次 `source install/setup.bash` 都会指向错误的前缀。

### `src/` 里放什么

四个仓库。**即使是只规划不动的演示，四个也都要**：

| 克隆到 `src/` | 提供的包 | 为什么需要 |
| --- | --- | --- |
| `litearm-moveit2` | `litearm_moveit_config` | 本仓库：SRDF、规划流水线、控制器映射与 launch 文件 |
| `litearm-description` | `litearm_description` | URDF 与网格；包位于 `SE/litearm_description` |
| `litearm-ros2-control` | `litearm_ros2_control` | ros2_control 插件，以及 include 它的那个 URDF 入口 |
| `litearm-cpp` | 无 —— 纯 CMake，没有 `package.xml` | 插件链接的 SDK |

规划演示也需要 ros2_control 这个包，这一点容易让人意外：本包 launch 的 URDF 只是一层
瘦包装，它 include 的是 `$(find litearm_ros2_control)/urdf/litearm.urdf.xacro`。
没有那个包，描述解析不出来，RViz 会加载出一个空机器人。

## 用容器快速上手

### 1. 构建运行镜像

这套栈没有已发布的镜像，自己建一个。它是公开的 `osrf/ros:humble-desktop-full` 加上本
工作区需要的包：`desktop-full` 里有 RViz、xacro 和 colcon，但**没有 MoveIt，也没有
任何 ros2_control**。

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

这份列表里有两点是刻意的：

- **`ros-humble-moveit-configs-utils` 要单独列出来。** `ros-humble-moveit` 元包并不
  依赖它，但 `litearm_moveit.launch.py` 从它里面 import `MoveItConfigsBuilder`，
  少了这一个是 launch 在 import 阶段就失败。
- **刻意不加 `--no-install-recommends`。** 对 MoveIt 元包来说，推荐包里带着运行期的
  插件与规划器配置，缺一个不会在启动时报错 —— 它会在之后表现为 `move_group` 内部的
  静默失效。

构建要几分钟，产出的镜像约 5.6 GB。

### 2. 建工作区并克隆各包（在宿主机上）

在**宿主机**上克隆，不在容器里：工作区是 bind mount，所以它能活过容器的 `--rm`，
你的 clone 和编辑器也都留在 Docker 外面。

```bash
mkdir -p ~/litearm_ws/src
cd ~/litearm_ws/src
git clone https://github.com/nexform-tech/litearm-cpp.git
git clone https://github.com/nexform-tech/litearm-description.git
git clone https://github.com/nexform-tech/litearm-moveit2.git
git clone https://github.com/nexform-tech/litearm-ros2-control.git
```

四个包各自的用途见[上面](#src-里放什么)。

### 3. 在容器里编译工作区

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

`--user "$(id -u):$(id -g)"` 和 `-e HOME=/tmp` 用来防止容器往你的工作区里写 root 属主的
文件。少了它们，`build/` 和 `install/` 回来时属主是 root，之后在宿主机上任何一次编辑或
`git clean` 都会因权限失败。

**不要加 `--symlink-install`。** 它在本工作区会失败：

```text
CMake Error at ament_cmake_symlink_install/ament_cmake_symlink_install.cmake:272 (message):
  ament_cmake_symlink_install_targets() can't find
  '/ws/build/litearm_ros2_control/litearm-cpp/litearm-cpp'
```

`litearm_ros2_control` 用 `add_subdirectory(... EXCLUDE_FROM_ALL)` 把 `litearm-cpp`
源码树拉进来，但 `litearm-cpp` 给它的命令行工具注册了一条无条件的
`install(TARGETS litearm-cpp ...)`。普通 CMake 会跳过「从未被构建过的目标」的安装，
而 ament 的 symlink-install 宏则直接断言该文件存在。本工作区支持的是普通
`colcon build`。

### 4. 在容器里跑规划演示

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

RViz 会打开 MotionPlanning 面板。拖动交互式标记后点 **Plan** —— 见
[在 RViz 里规划](#在-rviz-里规划)。

如果 RViz 打不开窗口，说明容器没有访问你 X server 的权限。只授权你自己的用户：

```bash
xhost +si:localuser:$(id -un)
```

无显示环境改用验收探针：它起同一套栈、规划到 `ready`、打印检查项，然后收栈：

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

## 不用容器快速上手

### 1. 安装 ROS 2 Humble

按官方 Ubuntu 22.04（jammy）的说明操作，选 `ros-humble-desktop` 那一档：

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

然后补上本工作区额外需要的包：

```bash
sudo apt install ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros2controlcli \
  ros-dev-tools
```

`ros-humble-moveit-configs-utils` 要单独写，是因为 `ros-humble-moveit` 元包并不依赖
它，而 `litearm_moveit.launch.py` 从它里面 import `MoveItConfigsBuilder`。

**`colcon` 不在 Ubuntu 自己的软件源里。** 在 jammy 上，`sudo apt install colcon` 会报
`E: 无法定位软件包 colcon`，`sudo apt install python3-colcon-common-extensions` 同样
报错 —— colcon 由 ROS 2 的 apt 源提供，而把它拉进来的是 `ros-dev-tools` 这个包。
在 ROS 2 本身装好之前，根本没有 colcon 可装。

只有真机才需要下面这一步：给自己串口访问权限。

```bash
sudo usermod -aG dialout "$USER"    # 重新登录后生效
```

### 2. 建工作区并克隆各包

```bash
mkdir -p ~/litearm_ws/src
cd ~/litearm_ws/src
git clone https://github.com/nexform-tech/litearm-cpp.git
git clone https://github.com/nexform-tech/litearm-description.git
git clone https://github.com/nexform-tech/litearm-moveit2.git
git clone https://github.com/nexform-tech/litearm-ros2-control.git
```

四个包各自的用途见[上面](#src-里放什么)。

### 3. 编译工作区

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

和容器路线一样：在 `~/litearm_ws` 里编译，不要在 `src/` 里编译，也不要加
`--symlink-install`。原因见[上面](#3-在容器里编译工作区)。

### 4. 跑规划演示

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litearm_moveit_config demo.launch.py
```

## 在 RViz 里规划

`demo.launch.py` 用 `start_control:=false` 起 MoveIt：可以在 RViz 里规划，而 Execute 需要
控制栈，控制栈又需要 USB 另一头真的接着机械臂。本部署**没有 dry-run**：硬件组件经
litearm C++ SDK 直接与真板子通信。

MotionPlanning 面板里的规划组是 `litearm_arm`。拖动交互式标记，或选一个 SRDF
命名状态再点 Plan & Execute：

| 命名状态 | 位形 |
| --- | --- |
| `zero` | 七个关节全零 —— 固件零点，上电后的构型 |
| `ready` | 肘部抬起的舒展构型；用它作规划起点，`zero` 在奇异附近，笛卡尔规划在那里会失败 |

⚠ 这套栈是**单实例**的，而让它单实例的是**串口**：组件独占机械臂的 USB CDC 链路，SDK 对其加
独占 `flock`，于是第二个控制栈打不开串口、它的硬件组件 configure 失败。此时 `move_group`
照常起来、照常能规划 —— 规划不需要硬件，执行才需要 —— 所以症状是「控制器不见了」，而不是
「launch 挂了」。先停掉另一个栈。

## 真机运行

在宿主机上，整条命令就是这一句：

```bash
ros2 launch litearm_moveit_config litearm_moveit.launch.py
```

⚠ 这条会真的驱动机械臂。板子上电、USB 接好、license 已激活，并确认机械臂已支撑、
急停可触达。

在容器路线里，它还需要把板子透传进去，外加宿主机上拥有该设备的那个组：

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

启动前先确认板子在线：

```bash
lsusb | grep 1d50          # 应看到 1d50:606f
ls /dev/ttyACM*            # 应看到 /dev/ttyACM0
```

控制栈已在别处运行（例如真机调试时手动起的）时加 `start_control:=false`，避免两个
`controller_manager` 抢同一批命令接口。`port` 留空（默认）按 VID:PID `1d50:606f`
自动发现板子；接多块板卡时用 `port:=/dev/ttyACM1` 指定。

这条 launch 还会在配置硬件时**清掉机械臂上锁存的故障**（`clear_faults`，默认开）。
固件会把急停或关节故障锁存住，并在 RESET 之前一律拒绝 ENABLE —— 没有这一步，故障之后
每次启动都要先单独跑一趟 `litearm_driver`。清故障发生在链路已连、尚未使能、尚未下发任何
帧的时候，并且会打一条 WARNING；如果成因还在，固件会重新置位，ENABLE 会带着原因报错。
想让锁存故障**拦住启动**（例如刚出过事故、想先看清机械臂状态）就加 `clear_faults:=false`。

## 从另一个终端连上这套栈

两个 launch 都把 ROS 域钉在 42 并开启仅本机发现，因此默认终端里什么都看不到，需要先
导出同样的变量：

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1
```

用 `ros_domain_id:=0 ros_localhost_only:=false` 可以加入默认图。共享网络下**不要**这么
做：别家的 `move_group` 可能接管你的目标，用一套它没有的控制器把它跑失败，现场看起来
像硬件故障。

如果 `ros2` 命令仍然什么都看不到，加 `--no-daemon` 再试一次：命令行默认从缓存守护进程
取图，而那个守护进程可能还停在导出变量之前的发现设置上。

## 下一步：MoveIt 2 教程

以下都指向 MoveIt 2 官方教程的 Humble 版，与本栈的目标发行版一致：

| 教程 | 讲什么 |
| --- | --- |
| [MoveIt Quickstart in RViz](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) | 本 launch 打开的就是这个面板：规划组、规划路径、交互式标记 |
| [MoveIt Setup Assistant](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html) | `config/` 下的 SRDF、关节限位、运动学文件是怎么生成的 |
| [URDF and SRDF](https://moveit.picknik.ai/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html) | 机器人描述与本包语义描述的分工 |
| [Kinematics Configuration](https://moveit.picknik.ai/humble/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html) | `config/kinematics.yaml`，IK 求解器的选择与预算 |
| [Low Level Controllers](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html) | MoveIt 如何把轨迹交给 ros2_control —— `config/moveit_controllers.yaml` 实现的正是它 |
| [Move Group C++ Interface](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html) | 不经过 RViz，在自己的节点里规划并执行 |
| [MoveIt Task Constructor](https://moveit.picknik.ai/humble/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html) | 多阶段任务（如抓取放置），也就是 `litearm_manipulation` 的方向 |

教程总览：[tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)；
示例总览：[examples](https://moveit.picknik.ai/humble/doc/examples/examples.html)。

## 适用范围

| | |
| --- | --- |
| 产品 | LiteArm 机械臂系列 |
| 本仓库定位 | ROS 2 上的 MoveIt 2 集成 |
| ROS 2 发行版 | Humble Hawksbill |
| 状态 | 活跃 —— `litearm_moveit_config` 可用 |

## 包内容

| 包 | 作用 |
| --- | --- |
| `litearm_moveit_config` | litearm 七轴臂的 MoveIt 2 配置：SRDF 与碰撞矩阵、KDL 运动学、由固件参数表派生的关节限位、OMPL 规划流水线、`moveit_controllers`、RViz 布局、launch 文件，以及验收探针 |

其中两点在各自文件头部有详细说明，这里只给结论：

- **IK 求解器是 KDL（MoveIt 自带）。** 它随 moveit_core 一起装好，工作区不需要额外的
  插件包。KDL 会漏掉真实存在的解 —— 4 个种子各试一遍，705 ms 内可能全部无解，
  而这些位姿 SNS-IK 解得出来 —— 所以当规划表现为**间歇性失败**而不是几何不可达时，
  换回 SNS-IK：配方、插件名陷阱（必须用 `::` 形式）与命中率对比都写在
  `config/kinematics.yaml` 里。
- **Pilz 流水线默认不加载。** 本包只用 OMPL 规划。`config/pilz_cartesian_limits.yaml`
  的存在只是为了让日后启用 Pilz 时不必再补文件。

## 会自己生成的配置

`config/` 下有三个文件是派生出来的，不是手写的。它们各自的文件头都写着「请勿手工维护」：

| 文件 | 由谁生成 | 什么时候重跑 |
| --- | --- | --- |
| `config/joint_limits.yaml` | `tools/gen_from_firmware_config.py <litearm-stm32>` | 固件参数表变了 |
| `config/initial_positions.yaml` | 同上 | 同上 |
| SRDF 里的 `disable_collisions` 区块 | `tools/compute_collision_matrix.py` | URDF 或网格变了 |

关节限位来自固件，且刻意比 URDF 收窄 1°：MoveIt 必须规划在固件运行期强制执行的安全包络
之内，否则会出现「规划成功、执行被拒」。

碰撞矩阵由真实网格采样得出：只有当一对 link 结构相邻、或在**每一个**采样位形下都接触
时才禁用检查。仅仅「会接近」的 link 对保留检查 —— 那些正是 MoveIt 应当避开的区域。

## 相关仓库

| 仓库 | 作用 |
| --- | --- |
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK |
| [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) | C++ SDK |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | 产品文档 |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | ROS 2 驱动 |

## 仓库规范

本仓库遵循 NEXFORM ROBOTICS 的共享仓库规范：[AGENTS.md](AGENTS.md) 里的代理作业规则、
Conventional Commits，以及每次合入 `main` 时自动执行的 semantic-release 版本管理。

## 许可证

版权所有 © 2026 NEXFORM ROBOTICS。以 [Apache License 2.0](LICENSE) 发布。
