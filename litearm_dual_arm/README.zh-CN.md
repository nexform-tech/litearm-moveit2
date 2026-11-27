[English](README.md) | 简体中文

# litearm_dual_arm

双 LiteArm 的 MoveIt 2 集成。本包由 `litearm_description` 的单臂描述生成
双臂 URDF/SRDF，在一个 `move_group` 里提供 `left_arm`、`right_arm`、
`both_arms` 三个规划组，并共享同一份 Planning Scene。

> **状态：确定性仿真执行和真机入口都已落地。** 当前提供双臂模型、共享场景、
> 三组规划、互碰检查，以及 `dual_arm_bridge.py` + 双通道 action server 的
> 仿真执行；真机入口 `dual_arm_real.launch.py` 接两套
> `litearm_ros2_control` 的 `side:=left|right` 栈，默认只规划，执行需要
> 授权/急停/工作空间/标定门控。没有 Gazebo 物理引擎。
> 本包不带夹爪，链末端是 `left_ee_link` / `right_ee_link`。

## 1. 它解决什么问题

- **一个模型**：从单臂 URDF 派生两套带 `left_`/`right_` 前缀的链，挂在
  同一个 `world` 下；不复制网格，不维护第二份描述。
- **一个 Planning Scene**：左右臂、静态工位都在同一个 move_group 里。单臂
  规划时另一条臂仍在场景中作为障碍物，所以不会出现“两套单臂栈互相看不见”
  的互碰风险。
- **三个组**：`left_arm`、`right_arm`、`both_arms`。`both_arms` 是
  14 自由度联合组，用于双臂同时规划。
- **互避碰**：SRDF 只禁用每条臂内部的相邻对，左右臂之间的碰撞对全部保留；
  轨迹采样用 `/check_state_validity` 逐点复检。
- **确定性执行桥**：`dual_arm_bridge.py` 把 MoveIt 的 14 关节轨迹拆成左右
  7 关节，并发转发到两路 namespaced JTC action；单侧失败、取消或超时会取
  消另一侧并进入 fail-closed。离线仿真提供与真机同形的两路 action server。
- **同步搬运任务**：`task:=carry` 用两个虚拟夹持点搬运托盘，不需要夹爪；
  包含 IK、`both_arms` 起始规划、双笛卡尔路径、轴向同步、逐点互碰/几何
  复检和双臂执行指标。
- **交替作业任务**：`task:=alternating` 按阶段让左右臂轮流进入共享区，
  空闲臂保持原位并继续参与整机碰撞检查；单臂执行后检查空闲臂偏差。

## 2. 编译

在工作区根目录编译：

```bash
cd ~/litearm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select litearm_dual_arm
source install/setup.bash
```

**不要加 `--symlink-install`**：本工作区的 `litearm_ros2_control` 在
symlink install 下会失败（见工作区 README）。

本包依赖已安装的 `litearm_description`、`litearm_moveit_config`。模型生成器
默认从 ament 索引定位它们；也可以显式传路径。

本包脚本的 shebang 固定为 `/usr/bin/python3`，也就是 ROS Humble 使用的系统
Python 3.10。如果当前 shell 里 conda 的 `python3`（3.11）排在前面，`ros2 run`
仍会按 shebang 使用系统 Python；手工执行脚本时请显式用 `/usr/bin/python3`，
否则会因 `rclpy_pybind11` 的 ABI 不匹配而失败。

## 3. 离线规划与确定性执行

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py
```

这条命令启动：

| 节点 | 作用 |
| --- | --- |
| `robot_state_publisher` | 双臂 TF，根坐标系 `world` |
| `move_group` | 三个规划组 + 共享 Planning Scene |
| `litearm_dual_arm_sim` | 两路 `/left`、`/right` 状态 + 两路 JTC action server |
| `litearm_dual_arm_bridge` | 状态合并、14→7+7 轨迹拆分、失败取消 |
| `litearm_dual_arm_scene` | 把 `dual_static` 静态工位写入场景 |
| `rviz2` | 双臂 MotionPlanning 面板 |

在 RViz 里：

1. MotionPlanning → Planning Group 选 `both_arms`；
2. 拖动目标状态，按 **Plan**，确认轨迹；
3. 按 **Execute**：MoveIt 把轨迹经 bridge 发到两路仿真 action server；
4. 分别选 `left_arm` / `right_arm`，验证单臂执行时另一条臂保持不动。

`execute:=false` 退化为只规划：

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py execute:=false
```

这是确定性仿真，不是 Gazebo：没有重力、接触力和跟踪误差，只验证执行链路、
时间戳、同步和失败传播。

无显示环境：

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py use_rviz:=false
```

只看模型、不加载静态场景：

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py scene:=none
```

### 3.1 同步搬运（carry）

第一版 carry 不需要夹爪：左右工具被视为托盘的虚拟夹持点，工具 Z 轴朝上，
托盘由两个法兰从下方托住。任务会：

1. 用 `dual_carry.yaml` 的起始中心、间距和方向求起始位形；
2. `both_arms` 规划到起始位形并复检；
3. 左右分别做笛卡尔直线；
4. 用工具实际轴向坐标对齐两侧进度，拼成 14 关节同步轨迹；
5. 每个采样点检查互碰、间距、共面、位置和姿态；
6. 经 bridge 同时执行，并输出指标 JSON。

```bash
# 规划 + 仿真执行；默认写 /tmp/litearm_dual_carry_metrics.json
ros2 launch litearm_dual_arm dual_arm_sim.launch.py task:=carry

# 只规划
ros2 launch litearm_dual_arm dual_arm_sim.launch.py task:=carry execute:=false

# 一键验收（起栈、跑 carry、检查 JSON、收栈）
ros2 run litearm_dual_arm acceptance_carry.sh
ros2 run litearm_dual_arm acceptance_carry.sh --plan-only
```

任务场景是 `dual_carry_static`（只有工作台）。`task:=carry` 时不再启动通用
场景节点，避免两套场景叠在一起。配置在
[dual_carry.yaml](config/dual_carry.yaml)：

- `start_center`、`direction`、`distance_m`
- `hand_separation_m`、`separation_axis`
- 左右工具四元数
- 已验证的 `start_left_rad` / `start_right_rad`
- 速度、采样数、路径时长、容差

`start_left_rad` / `start_right_rad` 是针对当前“两臂 yaw=+90°”挂载验证过的
起始关节；启动时会用 FK 核对它们与目标工具位姿一致。改挂载、改台面或换
工具后必须重新求 IK 并更新这两组值，否则会明确报错。

当前实测：起始规划与 20 点复检通过；左右 Cartesian fraction 都是 1.0；
41 个同步采样点无碰撞、间距误差约 2e-6 m、位置误差约 2e-5 m；双臂执行
成功，同步误差约 3 ms。

### 3.2 共享工作空间交替作业

`task:=alternating` 按 [dual_alternating.yaml](config/dual_alternating.yaml)
的阶段顺序执行：

```text
both_arms 回 ready
  → left_arm 进入共享区
  → left_arm 回撤
  → right_arm 进入共享区
  → right_arm 回撤
  → both_arms 回 ready
```

单臂阶段只给 active 侧发轨迹；另一条臂保持当前状态，并继续参与
`/check_state_validity` 的整机碰撞检查。执行后检查空闲臂偏差。

```bash
ros2 launch litearm_dual_arm dual_arm_sim.launch.py task:=alternating
ros2 launch litearm_dual_arm dual_arm_sim.launch.py \
  task:=alternating execute:=false

ros2 run litearm_dual_arm acceptance_alternating.sh
ros2 run litearm_dual_arm acceptance_alternating.sh --plan-only
```

目标来自 `dual_goals.yaml` 的 `left_safe`/`right_safe`/`left_ready`/
`right_ready`/`both_ready`；场景是 `dual_alternating_static`（工作台）。
当前实测六个阶段全部规划/复检/执行成功，空闲臂偏差为 0，双臂阶段同步误差
约 3–10 ms。

## 4. 验收探针

`dual_arm_probe.py` 通过 `/plan_kinematic_path` 规划，再对轨迹均匀采样调用
`/check_state_validity`，输出 JSON：

```bash
ros2 run litearm_dual_arm dual_arm_probe.py --target both_safe
ros2 run litearm_dual_arm dual_arm_probe.py --target left_safe
ros2 run litearm_dual_arm dual_arm_probe.py --target right_safe --samples 31
```

规划并直接经 bridge 执行：

```bash
ros2 run litearm_dual_arm dual_arm_probe.py --target both_safe --execute
```

验证 RViz/MoveIt 的 Execute 路径（simple controller manager → bridge）：

```bash
ros2 run litearm_dual_arm dual_arm_probe.py \
  --target both_ready --via-move-action
```

判据：

- `planning.success = true`
- `validation.invalid_indices = []`
- `validation.contacts = []`
- `--execute` / `--via-move-action` 时 `execution.success = true`
- `success = true`

负向用例 `mutual_collision_case` 只检查目标位形本身，不规划：

```bash
ros2 run litearm_dual_arm dual_arm_probe.py \
  --target mutual_collision_case --check-state
```

这里 `success=true` 的含义相反：MoveIt 必须判该位形无效，并且
`cross_arm_contacts` 非空（出现 `left_*` ↔ `right_*` 接触对）。这是左右臂
互碰约束确实生效的负向证明。

默认输出 `/tmp/litearm_dual_arm_probe.json`。目标定义在
`config/dual_goals.yaml`；它们是验收用关节目标，不是业务任务。

## 5. 模型生成

launch 会自动调用生成器。手工检查输出：

```bash
ros2 run litearm_dual_arm generate_dual_arm_model.py --mode urdf
ros2 run litearm_dual_arm generate_dual_arm_model.py --mode srdf
ros2 run litearm_dual_arm generate_dual_arm_model.py --mode both
```

生成规则：

- 单臂 URDF 的所有 `link`/`joint` 加 `left_`/`right_` 前缀；
- `world` + `left_world_joint`/`right_world_joint` 固定挂载；
- SRDF 组：`left_arm`、`right_arm`、`both_arms`；
- 命名状态：`left_zero`/`right_zero`/`both_zero`、
  `left_ready`/`right_ready`/`both_ready`；
- `disable_collisions` 只保留同侧相邻对和 `world ↔ base_link`，**不**生成
  任何跨左右臂的禁用对。

## 6. 安装标定

`config/dual_arm_mounts.yaml` 的默认值是台架参考：

```text
left  base_link: [0.0,  0.38, 0.0]
right base_link: [0.0, -0.38, 0.0]
```

两臂并排、基座相距 0.76 m，各自向 +X 伸展。**这不是实测值**，真机执行前
必须测量左右基座在 `world` 下的 xyz/rpy，并置：

```yaml
calibration:
  measured: true
```

改挂载后，`dual_goals.yaml` 里的目标也要重新做 FK/碰撞校验，不能直接沿用。
当前 `tool.type: none`；以后加圆柱工具时改为 `cylinder` 并填实测尺寸，
生成器会自动把 SRDF 组末端改成 `tool_link`。

## 7. 测试

```bash
colcon test --packages-select litearm_dual_arm --event-handlers console_direct+
colcon test-result --verbose
```

完整验收（起栈、执行四个目标、MoveGroup 路径、碰撞负向用例、fail-closed
停止后拒绝新目标、收栈）：

```bash
ros2 run litearm_dual_arm acceptance_dual_arm.sh
ros2 run litearm_dual_arm acceptance_dual_arm.sh --scene none
TARGETS="both_safe left_safe" ros2 run litearm_dual_arm acceptance_dual_arm.sh

# 同步搬运专项
ros2 run litearm_dual_arm acceptance_carry.sh
ros2 run litearm_dual_arm acceptance_carry.sh --plan-only

# 交替作业专项
ros2 run litearm_dual_arm acceptance_alternating.sh
ros2 run litearm_dual_arm acceptance_alternating.sh --plan-only
```

测试覆盖：

- 模型：前缀、`world` 挂载、三组、命名状态、跨臂碰撞对、工具开关；
- 配置：OMPL 组、控制器映射、目标关节集、关节限位、场景引用；
- 执行桥：14→7+7 名称映射、速度/加速度/时间戳保留、仿真初始位形解析；
- carry：配置几何、四元数误差、路径插值、14 关节同步轨迹、起始几何校验；
- alternating：阶段顺序、目标存在性、目标组与 active 侧一致性；
- launch：参数集合、`execute` 门控、`task` 任务和 bridge 节点。

## 8. 真机入口

前置条件：

- `litearm_ros2_control` 已支持 `side:=left|right`；
- 左右基座已按实际安装测量，`dual_arm_mounts.yaml` 的
  `calibration.measured: true`；
- 两个 USB 口用 `/dev/serial/by-id/` 明确区分；
- 物理急停在岗、工作空间已清空。

### 8.1 外部启动左右控制栈，本 launch 只规划

```bash
# 终端 1：左臂
ros2 launch litearm_ros2_control litearm_control.launch.py \
  side:=left port:=/dev/serial/by-id/<left-arm>

# 终端 2：右臂
ros2 launch litearm_ros2_control litearm_control.launch.py \
  side:=right port:=/dev/serial/by-id/<right-arm>

# 终端 3：双臂 MoveIt + bridge，只规划
ros2 launch litearm_dual_arm dual_arm_real.launch.py
```

默认 `execute:=false`：bridge 只做状态合并，`move_group` 不允许执行。
`preflight:=true` 会启动只读预检，检查两套 controller_manager、JSB/JTC、
状态频率、action（`execute:=true` 时）和 TF；结果写到
`/tmp/litearm_dual_arm_preflight.json`。

然后用探针做只规划验收：

```bash
ros2 run litearm_dual_arm dual_arm_probe.py \
  --target both_safe --samples 20
```

### 8.2 由本 launch 一并启动两套 side 控制栈

```bash
ros2 launch litearm_dual_arm dual_arm_real.launch.py \
  start_control:=true \
  left_port:=/dev/serial/by-id/<left-arm> \
  right_port:=/dev/serial/by-id/<right-arm>
```

`start_control:=true` 时两个端口必须显式给出；本 launch 会把
`start_robot_state_publisher:=false` 传给两侧，TF 由双臂模型统一发布。

### 8.3 门控执行

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

`execute:=true` 同时要求：

- `authorization:=I_UNDERSTAND_DUAL_ARM_MOTION`；
- `estop_verified:=true`；
- `workspace_verified:=true`；
- `dual_arm_mounts.yaml` 的 `calibration.measured: true`。

首轮只做单臂、小幅度、低速运动；确认后再做双臂。`dual_arm_real.launch.py`
不替代物理急停，bridge 的 `/litearm_dual_arm/stop` 只是 fail-closed 软件停止。

真机场景默认 `scene:=none`。`dual_static` 是台架参考场景，只有确认与实际
工位一致时才传 `scene:=dual_static`。

## 9. 下一步

1. 用真机 side 栈跑通 `execute:=false` 的预检和只规划。
2. 低速单臂 → 低速双臂 → 现场标定与验收。
3. 端盘/交替作业等业务任务；可选 Gazebo/Octomap/双臂 supervisor。

## 10. 注意

- 真机入口默认只规划；`execute:=true` 才会动。不要用仿真验收替代真机。
- bridge 的 `/litearm_dual_arm/stop` 是 fail-closed 停止请求：触发后新的
  轨迹会被拒绝，必须重启 bridge 并查清原因。
- 双臂共享工作空间时，不要用两套独立的单臂 MoveIt 栈分别控制两条臂；
  它们看不到对方，互碰保护会失效。
- 真机急停始终是独立的安全手段，软件停止服务不能替代它。
