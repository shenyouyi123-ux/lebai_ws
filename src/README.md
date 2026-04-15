# Lebai LM3 + 夹爪 + 轴系 MoveIt 2 集成说明

## 项目概述

本项目在 ROS 2 Humble + MoveIt 2 环境下，实现了乐白 LM3 机械臂（含并联夹爪 + 外部轴系）的仿真与真实硬件控制。

- 仿真模式：`mock_components/GenericSystem` 虚拟硬件，无需连接实体机械臂
- 真实硬件模式：通过乐白官方驱动 + 自研夹爪桥接节点控制实体机械臂

---

## 涉及的包

| 包名 | 说明 |
|------|------|
| `lebai_with_gripper_and_shaft_moveit_config` | MoveIt 配置包（SRDF、控制器、规划器等） |
| `lebai_gripper_bridge` | 夹爪桥接节点（新建） |
| `lebai_lm3_support`（lebai-ros-sdk 内） | URDF 模型文件 |

---

## 修改的文件清单

### lebai_lm3_support（lebai-ros-sdk）

**`urdf/lm3_with_gripper_and_shaft.urdf`**
- 新增 `<ros2_control name="FakeSystem">` 块，使用 `mock_components/GenericSystem` 注册 7 个关节（joint_1~6 + gripper_r_joint1），支持仿真模式
- 将 `shaft_joint1` 从 `continuous` 改为 `fixed`，消除 MoveIt 启动时的关节状态缺失警告
- 将 `gripper_r_joint1` 上限从 `0.85` 改为 `1.1`，匹配硬件实际上报范围（~1.026 rad）

---

### lebai_with_gripper_and_shaft_moveit_config

**`config/joint_limits.yaml`**
- 将所有整数值改为浮点数（`max_velocity: 1` → `1.0`，`max_acceleration: 0` → `0.0`），修复 ROS 2 严格类型检查报错

**`config/lm3_with_shaft.srdf`**
- 配置夹爪 Planning Group（`gripper`，仅含 `gripper_r_joint1`，Kinematic Solver 为 None）
- 配置 End Effector（`hand`，parent_link=`tool0`，parent_group=`manipulator`）
- 将 `open_gripper` 状态值从 `0.85` 改为 `1.0`，与硬件实际范围对齐
- 移除 `<passive_joint name="shaft_joint1"/>`（shaft 已改为 fixed joint）

**`config/moveit_controllers.yaml`**（仿真模式）
- 新增 `moveit_controller_manager` 插件声明
- 配置 `manipulator_controller` 和 `gripper_controller`，`action_ns: follow_joint_trajectory`

**`config/moveit_controllers_real.yaml`**（新建，真实硬件模式）
- 配置 `lebai_trajectory_controller`（对接乐白驱动 Action Server）
- 配置 `gripper_bridge_controller`（对接夹爪桥接节点 Action Server）
- `action_ns: follow_joint_trajectory`

**`config/ros2_controllers.yaml`**（新建）
- 仿真模式 ros2_control_node 控制器配置
- `manipulator_controller` 和 `gripper_controller`，`command_interfaces: [position]`

**`config/sensors_3d.yaml`**（新建）
- 空传感器列表（`sensors: []`），防止 MoveIt 加载默认 Kinect 配置导致类型错误

**`launch/real_robot.launch.py`**（新建）
- 真实硬件启动文件，启动：乐白驱动、夹爪桥接节点、robot_state_publisher、static_tf、move_group、rviz2
- 手动构建 `moveit_controllers` 参数字典，将控制器配置嵌套在 `moveit_simple_controller_manager` 命名空间下（MoveItSimpleControllerManager 插件要求）

---

### lebai_gripper_bridge（新建包）

**功能**：将 MoveIt 发出的 `FollowJointTrajectory` Action 转换为乐白夹爪的 `SetGripper` Service 调用。

**`lebai_gripper_bridge/gripper_bridge_node.py`**
- Action Server：`/gripper_bridge_controller/follow_joint_trajectory`
- Service Client：`/io_service/set_gripper_position`
- 换算：`amplitude = joint_angle / 1.0 * 100`（0 rad=全闭/0%，1.0 rad=全开/100%）

**`setup.py`** / **`setup.cfg`** / **`package.xml`**
- 标准 ROS 2 Python 包结构，`setup.cfg` 指定可执行文件安装路径

---

## 启动步骤

### 编译

```bash
cd ~/lebai_ws
colcon build --packages-select \
  lebai_lm3_support \
  lebai_with_gripper_and_shaft_moveit_config \
  lebai_gripper_bridge
source install/setup.bash
```

### 仿真模式（无需连接机械臂）

```bash
ros2 launch lebai_with_gripper_and_shaft_moveit_config demo.launch.py
```

在 RViz Motion Planning 面板中：
- Planning Group 切换为 `manipulator` 规划主臂
- Planning Group 切换为 `gripper`，使用 `open_gripper` / `close_gripper` 预设状态控制夹爪

### 真实硬件模式

```bash
ros2 launch lebai_with_gripper_and_shaft_moveit_config real_robot.launch.py robot_ip:=10.20.17.1
```

将 `robot_ip` 替换为实际机械臂 IP 地址。

---

## 真实硬件连接操作步骤

### 前置检查

1. 确认机械臂已上电，控制器 IP 可达：
   ```bash
   ping 10.20.17.1
   ```
2. 确认夹爪已安装并通过 IO 接口连接到机械臂控制器

### 启动顺序

```bash
# 1. source 环境
source ~/lebai_ws/install/setup.bash

# 2. 启动（一条命令，内部按顺序拉起所有节点）
ros2 launch lebai_with_gripper_and_shaft_moveit_config real_robot.launch.py robot_ip:=10.20.17.1
```

启动后终端应依次出现：
- `lebai_driver` 连接成功日志
- `等待 /io_service/set_gripper_position 服务...` → `夹爪 IO 服务已就绪`
- `move_group` 输出 `MoveGroup context initialization complete`
- RViz 窗口弹出

### 夹爪控制步骤（RViz）

1. 在 Motion Planning 面板左上角 **Planning Group** 下拉框选择 `gripper`
2. **Goal State** 下拉框选择 `open_gripper` 或 `close_gripper`
3. 点击 **Plan & Execute**
4. 观察终端 gripper_bridge 节点输出：`夹爪目标：X.XXX rad → amplitude=XX.X`

### 验证夹爪 IO 服务是否正常

```bash
# 手动调用 SetGripper，amplitude=50 表示半开
ros2 service call /io_service/set_gripper_position lebai_interfaces/srv/SetGripper "{val: 50.0}"
```

返回 `ret: true` 表示正常。

---

## 遇到的问题与解决方案

### 1. `expected [double] got [integer]`

**原因**：MoveIt Setup Assistant 生成的 `joint_limits.yaml` 中速度/加速度值为整数（如 `max_velocity: 1`），ROS 2 参数系统严格区分类型。

**解决**：将所有数值改为浮点数（`1` → `1.0`，`0` → `0.0`）。

---

### 2. `No ros2_control tag found in URDF`

**原因**：URDF 缺少 `<ros2_control>` 块，ros2_control_node 无法加载硬件接口。

**解决**：在 URDF 末尾添加 `mock_components/GenericSystem` 块，注册所有需要控制的关节。

---

### 3. `sensors parameter tuple type error`

**原因**：MoveIt Setup Assistant 生成的 `sensors_3d.yaml` 包含 Kinect 传感器配置，加载时产生空 tuple 类型错误。

**解决**：将 `sensors_3d.yaml` 内容替换为 `sensors: []`。

---

### 4. `moveit_controller_manager not specified`

**原因**：`moveit_controllers.yaml` 缺少插件声明行。

**解决**：在文件顶部添加：
```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
```

---

### 5. `No controller_names specified` / `Returned 0 controllers`

**原因**：`MoveItSimpleControllerManager` 插件要求 `controller_names` 等配置嵌套在 `moveit_simple_controller_manager` 参数命名空间下。使用 `MoveItConfigsBuilder.to_dict()` 会将所有参数展平到顶层，导致插件找不到配置。

**解决**：在 launch 文件中手动构建参数字典：
```python
controllers_yaml = load_yaml("pkg_name", "config/moveit_controllers_real.yaml")
moveit_controllers = {
    "moveit_simple_controller_manager": controllers_yaml,
    "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
}
```

---

### 6. `No action namespace specified`

**原因**：`moveit_controllers_real.yaml` 中 `action_ns: ""` 为空，MoveIt 无法拼出完整 Action 路径。

**解决**：改为 `action_ns: follow_joint_trajectory`，与 Action Server 注册名称一致。

---

### 7. `Skipping invalid start state (invalid bounds)` / 夹爪无法关闭

**原因**：夹爪硬件实际上报的全开位置约 1.026 rad，超出 URDF 中 `gripper_r_joint1` 的上限 `0.85`。MoveIt 认为当前状态非法，规划器拒绝初始化起始树。

**解决**：
- URDF `gripper_r_joint1` 上限：`0.85` → `1.1`
- SRDF `open_gripper` 状态值：`0.85` → `1.0`
- bridge 节点换算基准 `GRIPPER_JOINT_MAX_RAD`：`0.85` → `1.0`

---

### 8. `Missing shaft_joint1` 关节状态警告

**原因**：`shaft_joint1` 为 `continuous` 类型，MoveIt 期望收到其关节状态，但没有对应的控制器发布。

**解决**：将 URDF 中 `shaft_joint1` 改为 `type="fixed"`，同时移除 SRDF 中的 `<passive_joint name="shaft_joint1"/>`。

---

## 深度相机接入指南（预留）

接入深度相机（如 RealSense D435、Azure Kinect）时，需注意以下几点，避免重蹈本次踩过的坑：

### sensors_3d.yaml 配置

MoveIt 的 3D 感知功能通过 `sensors_3d.yaml` 配置。**不要留空文件**，也不要使用 Setup Assistant 自动生成的 Kinect 模板（会触发类型错误）。

正确的最小配置（无传感器时）：
```yaml
sensors: []
```

接入 RealSense 点云时的参考配置：
```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth/color/points
    max_range: 5.0
    padding_offset: 0.1
    padding_scale: 1.0
    point_subsample: 1
    filtered_cloud_topic: /filtered_cloud
```

### 坐标系（TF）

深度相机需要发布相机光心到机器人 TF 树的变换。如果相机固定安装在夹爪上：
- parent frame：`tool0`（或 `gripper_base_link`）
- child frame：`camera_link`（相机驱动发布的根 frame）

在 launch 文件中添加 static_transform_publisher，根据实际安装位置测量 xyz/rpy：
```python
Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["x", "y", "z", "roll", "pitch", "yaw",
               "tool0", "camera_link"],
)
```

### 点云话题

确认相机驱动发布的点云话题名称与 `sensors_3d.yaml` 中 `point_cloud_topic` 一致：
```bash
ros2 topic list | grep points
```

### 避免的错误

- 不要在 `sensors_3d.yaml` 中使用整数参数（如 `max_range: 5`），统一写浮点数（`5.0`）
- 相机 frame 必须在 TF 树中可达，否则 OctoMap 更新器会静默失败
- 点云频率过高会导致 move_group 规划卡顿，建议通过 `point_subsample` 降采样

---

## 注意事项

1. **夹爪关节范围**：硬件实际全开位置约 1.026 rad，URDF 上限设为 1.1 rad。SRDF `open_gripper` 目标为 1.0 rad（对应 amplitude=100%）。

2. **mimic 关节**：`gripper_r_joint2`、`gripper_l_joint1` 等 mimic 关节不在 Planning Group 内，由硬件层自动跟随 `gripper_r_joint1`，不需要也不能单独控制。

3. **轴系（shaft）**：`shaft_joint1` 为 fixed 类型，不参与 MoveIt 规划，仅作为静态 TF 存在于 URDF 中。

4. **控制器命名空间**：`MoveItSimpleControllerManager` 要求控制器配置嵌套在 `moveit_simple_controller_manager` 参数命名空间下，不能使用 `MoveItConfigsBuilder.to_dict()` 直接展开（会丢失命名空间层级）。

5. **仿真/真实切换**：两种模式使用独立的 launch 文件和控制器配置文件，互不影响，可随时切换。

6. **依赖**：真实硬件模式需要 `lebai_driver` 包已安装并可正常连接机械臂，`lebai_interfaces` 包提供 `SetGripper` Service 类型定义。
