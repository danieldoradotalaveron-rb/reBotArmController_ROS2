# reBot Arm B601-DM ROS2 SDK

<p align="center">
  <img src="./media/rebot_arm_b601.png" alt="reBot Arm B601-DM" width="720">
</p>

<p align="center">
  <strong>ROS2 · 机械臂控制 · 夹爪控制 · 轨迹接口 · RViz 可视化 · 全开源</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble | Jazzy-blue.svg" alt="ROS2 Humble">
  <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python 3.10">
  <img src="https://img.shields.io/badge/Version-v0.1.0-brightgreen.svg" alt="Version v0.1.0">
  <img src="https://img.shields.io/badge/Platform-Ubuntu%2022.04+-orange.svg" alt="Ubuntu 22.04+">
  <img src="https://img.shields.io/badge/Hardware-B601--DM-lightgrey.svg" alt="B601-DM">
</p>

<p align="center">
  <strong>
    <a href="./README_zh.md">简体中文</a> &nbsp;|&nbsp;
    <a href="./README.md">English</a> &nbsp;|&nbsp;
    <a href="./API_zh.md">API 文档</a>
  </strong>
</p>

---

## 项目介绍

当前版本：`v0.1.0`

`rebotarm_ros2` 是 reBot Arm B601-DM 机械臂的 ROS2 SDK 工作空间。它将现有的
`reBotArm_control_py` Python 控制库封装为 ROS2 topic、service 和 action，
作为二次开发、上层规划、可视化、重力补偿和单电机调试的统一入口。

当前工作空间包含三个 ROS2 包：

| 包 | 作用 |
|---|---|
| `rebotarm_msgs` | 自定义 msg / srv / action 接口 |
| `rebotarmcontroller` | 控制节点包，提供 `reBotArmController` 节点 |
| `rebotarm_bringup` | launch、配置、URDF、RViz 等启动资源 |

---

## 核心功能

- 发布机械臂状态：`/rebotarm/joint_states`、`/rebotarm/arm_status`
- 提供基础服务：`enable`、`disable`、`set_mode`、`set_zero`、`safe_home`
- 支持笛卡尔目标：`MoveToPoseIK` service、`MoveToPose` action
- 支持标准轨迹接口：`control_msgs/action/FollowJointTrajectory`
- 支持夹爪控制：`SetGripper` service、`GripperCommand` action
- 支持 controller 内部重力补偿：`gravity_compensation/start`、`gravity_compensation/stop`
- 支持 per-joint raw command：`JointMitCmd`、`JointPosVelCmd`、`JointVelCmd`
- 复用 `reBotArm_control_py` 的 `RobotArm`、`ArmEndPos`、FK/IK 和动力学
- 机械臂与夹爪共用同一个底层 `Controller` / 串口，避免重复打开 `/dev/ttyACM*`

---

## 环境配置

| 组件 | 型号 / 要求 |
|---|---|
| 机械臂 | reBot Arm B601-DM |
| 通信接口 | USB2CAN 串口桥接器 |
| 主机 | Ubuntu 22.04+，ROS2，Python 3.10+ |

接线说明：

1. 将 USB2CAN 串口桥接器连接到机械臂 CAN 总线。
2. 将夹爪电机接入同一条 CAN 总线，不要为夹爪单独打开第二个串口连接。
3. 将 USB2CAN 插入主机，并确认设备名：

```bash
ls /dev/ttyACM*
```

如果需要临时开放串口权限：

```bash
sudo chmod 666 /dev/ttyACM0
```

---

## 配置开发环境

### Step 1. 安装 ROS2 依赖

请参考[ROS官方下载文档](https://www.ros.org/blog/getting-started/)选择适合的版本进行安装。

### Step 2. 获取 ROS2 源码

优先使用 Seeed-Projects 官方仓库：

```bash
mkdir -p ~/seeed
cd ~/seeed
git clone https://github.com/Seeed-Projects/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

也可以使用当前开发仓库：

```bash
mkdir -p ~/seeed
cd ~/seeed
git clone https://github.com/EclipseaHime017/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

### Step 3. 安装 motorbridge

`motorbridge` 从 PyPI 官方源安装：

```bash
python3 -m pip install --user --index-url https://pypi.org/simple motorbridge
```

### Step 4. 获取底层 SDK


```bash
cd ~/seeed/rebotarm_ros2
mkdir -p third_party
git clone https://github.com/vectorBH6/reBotArm_control_py.git third_party/reBotArm_control_py
```

## 构建工作空间

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

验证包和入口：

```bash
ros2 pkg executables rebotarmcontroller
```

期望输出：

```text
rebotarmcontroller reBotArmController
rebotarmcontroller GravityCompensation
rebotarmcontroller GripperControl
rebotarmcontroller MoveTo
rebotarmcontroller MoveToPose
```

---

## 目录结构

```text
rebotarm_ros2/
├── README_zh.md
├── API_zh.md
├── PLAN.md
├── instruction.md
└── src/
    ├── rebotarm_msgs/
    │   ├── msg/
    │   ├── srv/
    │   └── action/
    ├── rebotarmcontroller/
    │   ├── rebotarmcontroller/
    │   │   ├── rebotarm_controller.py
    │   │   ├── hardware_manager.py
    │   │   ├── ros_publishers.py
    │   │   ├── ros_services.py
    │   │   ├── ros_actions.py
    │   │   ├── motor_passthrough.py
    │   │   ├── conversions.py
    │   │   └── examples/
    └── rebotarm_bringup/
        ├── launch/
        ├── config/
        ├── description/
        └── rviz/
```

---

## 快速启动

### 启动完整系统

启动控制节点、`robot_state_publisher`，可选 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py
```

`reBotArmController` 启动时会直接连接真实硬件；如果默认 `/dev/ttyACM0` 不存在，
需要通过 `channel:=/dev/ttyACM*` 指定正确串口。

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:={/dev/实际的串口名称}
```

启用 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
```

### 只启动控制节点

```bash
ros2 launch rebotarm_bringup driver_only.launch.py
```

### 直接运行控制节点

```bash
ros2 run rebotarmcontroller reBotArmController
```

---

## 直接移动到 Pose

不运行 demo 时，可以直接调用 ROS service 和 action 完成一次末端位姿移动。
先在一个终端启动控制节点：

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

然后在另一个终端执行控制命令：

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

1. 使能机械臂：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
```

2. 移动末端到目标 pose：

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration: 2.0}"
```

`move_to_pose` action 内部会确保进入 `pos_vel` 控制，并直接调用 SDK `ArmEndPos.move_to_traj(...)`。

3. 回到安全零位：

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
```

4. 失能并退出：

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

---

## 示例脚本

所有示例都假设已经启动 `reBotArmController`：

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

示例已注册为 ROS2 可执行入口，可以直接通过 `ros2 run` 调用。

源文件位于：

```text
src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py
src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py
src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py
src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py
```

### move_to.py

关节空间绝对角移动示例。一次性控制 6 个电机，参数为 6 个绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- \
  0.20 -0.20 -0.20 -0.20 0.10 -0.10 \
  --duration 8.0
```

一次性控制 1 个电机，参数为目标关节名和绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- --joint joint3 --position -0.20 --duration 5.0
```

### move_to_pose.py

末端位姿移动示例。

```bash
ros2 run rebotarmcontroller MoveToPose -- --x 0.30 --y 0.0 --z 0.30 --qw 1.0 --duration 2.0
```

### gravity_compensation.py

重力补偿示例。

```bash
ros2 run rebotarmcontroller GravityCompensation
```

脚本启动时会先调用 `/rebotarm/enable`，再启动重力补偿。按 `Ctrl+C` 退出时，
脚本会依次调用 `/rebotarm/gravity_compensation/stop`、`/rebotarm/safe_home`
和 `/rebotarm/disable`，让机械臂回到安全零位后失能。

对应底层服务：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

### gripper_control.py

交互式夹爪开闭示例。

```bash
ros2 run rebotarmcontroller GripperControl
```

运行后输入：

```text
o / open    打开夹爪
c / close   闭合夹爪
q / quit    退出
```
---

## API 文档

完整 ROS2 API 已整理到独立文档：[API_zh.md](API_zh.md)。

其中包含：

- topic、service、action 的完整列表和类型
- `/rebotarm` 命名空间、QoS、单位和状态机约定
- `JointMitCmd`、`JointPosVelCmd`、`JointVelCmd`、`ArmStatus`、`MoveToPose` 等自定义接口说明
- 末端位姿移动、夹爪控制、重力补偿、低层 command 的调用示例
- 上层集成和多机械臂命名注意事项

---

## 配置说明

`rebotarm_bringup/config/` 提供默认配置：

| 文件 | 说明 |
|---|---|
| `arm.yaml` | 机械臂 6 个关节的电机、反馈 ID、控制参数 |
| `gripper.yaml` | 夹爪电机配置，包含电机 ID、反馈 ID、厂商、MIT/POS_VEL 参数 |
| `driver_params.yaml` | ROS 参数示例 |

常用 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `arm_config` | bringup 内置 `arm.yaml` | 机械臂配置路径 |
| `gripper_config` | bringup 内置 `gripper.yaml` | 夹爪配置路径 |
| `channel` | 空字符串 | 留空使用 YAML，非空时覆盖串口 |
| `joint_state_rate` | `100.0` | `/rebotarm/joint_states` 发布频率 |
| `cmd_arbitration` | `reject` | 轨迹运行中 arm joint 低层 cmd 仲裁，`reject` 或 `preempt`；gripper 低层 cmd 不抢占 arm 轨迹 |
| `arm_namespace` | `rebotarm` | ROS 命名空间前缀 |
| `frame_id` | `base_link` | 机械臂基座坐标系，预留给 TF、视觉和规划集成 |
| `ee_frame_id` | `end_link` | 末端坐标系，预留给 TF、视觉和规划集成 |
| `use_rviz` | `false` | 是否启动 RViz |

---

## 排障

### 找不到串口

如果启动时报：

```text
open serial port /dev/ttyACM0 failed: No such file or directory
```

说明默认串口不存在。先查看实际设备：

```bash
ls /dev/ttyACM*
```

然后用 `channel:=...` 覆盖：

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

### 权限不足

如果串口存在但无权限：

```bash
sudo usermod -a -G dialout $USER
```

重新登录后生效。

### RViz 模型不显示

确认 URDF mesh 路径已经是：

```text
package://rebotarm_bringup/description/meshes/...
```

### FastDDS SHM 端口提示

如果终端出现类似：

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7002: open_and_lock_file failed
```

通常是之前的 ROS2 进程异常退出后，FastDDS shared memory 锁文件残留。服务和 action
能正常响应时，这个提示一般不影响控制。需要清理时，先停掉相关 ROS2 进程，再执行：

```bash
pkill -f ros2
pkill -f reBotArmController
rm -f /dev/shm/fastrtps_port*
```

如果希望临时绕开 shared memory transport，可在启动 ROS2 前设置：

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```
