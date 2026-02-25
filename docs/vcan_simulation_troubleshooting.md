# vcan 仿真测试问题排查报告

## 背景

在 `test/simulate` 分支上，我们尝试使用 UR5 URDF 和 vcan 虚拟 CAN 接口对 `can_driver` 包进行仿真测试，以验证其与 MoveIt 的集成是否正确。测试过程中遇到了一系列问题，本文档记录了问题现象、根本原因和解决方案。

---

## 问题 1：控制器加载失败 — 缺少 URDF

### 现象

启动 `can_driver.launch` 后，`arm_controller` 和 `wheel_controller` 加载失败，报错：

```
[ERROR] Could not find parameter robot_description on parameter server
[ERROR] Failed to parse URDF contained in 'robot_description' parameter
[ERROR] Failed to initialize the controller
```

### 原因

`JointTrajectoryController` 和 `DiffDriveController` 都需要从 `/robot_description` 参数读取 URDF，用于：
- 验证关节名是否存在
- 读取关节限位（position/velocity limits）
- 验证关节类型（revolute/continuous/prismatic）

而我们的 launch 文件没有加载 URDF。

### 解决方案

在 `can_driver.launch` 中添加 URDF 加载：

```xml
<param name="robot_description" command="$(find xacro)/xacro '$(find ur_description)/urdf/ur5.xacro'"/>
```

### 关键教训

**`can_driver` 本身不依赖 URDF**（硬件层从 YAML 读取配置），但 **ros_control 的标准控制器（JointTrajectoryController、DiffDriveController）依赖 URDF**。这是 ros_control 的设计，不是 can_driver 的问题。

类比 ros_canopen：
- `canopen_motor_node` 本身不需要 URDF
- 但要用 `JointTrajectoryController`，就必须提供 URDF

---

## 问题 2：控制器包未安装

### 现象

```
[ERROR] Could not load controller 'arm_controller' because controller type
        'position_controllers/JointTrajectoryController' does not exist.
```

### 原因

系统缺少 `joint_trajectory_controller` 和 `diff_drive_controller` 包。

### 解决方案

```bash
sudo apt install ros-noetic-joint-trajectory-controller ros-noetic-diff-drive-controller
```

---

## 问题 3：RViz 中机械臂在两个位置间跳变

### 现象

启动 MoveIt 后，RViz 中的机械臂模型在两个位置之间不停跳变：
- 一个位置在零位（全关节角度为 0）
- 另一个位置是拖动的目标位置

### 初步分析

怀疑是 `/joint_states` 有两个发布源冲突：
1. `can_driver` 的 `joint_state_controller`
2. MoveIt `demo.launch` 的 `fake_execution`

### 尝试的解决方案 1：禁用 MoveIt 的 fake_execution

```bash
roslaunch HelloArm_moveit_config demo.launch fake_execution:=false
```

**结果：问题依旧。**

### 尝试的解决方案 2：让 vcan 模式下 `read()` 返回命令值

修改 `CanDriverHW::read()`，在 vcan 模式下直接返回 `posCmd`：

```cpp
if (jc.canDevice.find("vcan") == 0) {
    jc.pos = jc.posCmd;  // 反馈 = 指令
    jc.vel = jc.velCmd;
    jc.eff = 0.0;
}
```

**结果：问题依旧。** `/joint_states` 中所有关节的 position 仍然是 0.0。

### 尝试的解决方案 3：让 `pos` 以固定速度追踪 `posCmd`

模拟电机运动过程，让 `pos` 逐渐逼近 `posCmd`：

```cpp
if (jc.canDevice.find("vcan") == 0) {
    double error = jc.posCmd - jc.pos;
    double maxStep = 2.0 * period.toSec();  // 最大 2 rad/s
    if (std::abs(error) < maxStep) {
        jc.pos = jc.posCmd;
    } else {
        jc.pos += (error > 0 ? maxStep : -maxStep);
    }
    jc.vel = (jc.posCmd - jc.pos) / period.toSec();
    jc.eff = 0.0;
}
```

**结果：问题依旧。** 机械臂仍然被"拉回"零位。

---

## 问题 3 的根本原因

通过 `rostopic echo /joint_states` 发现，所有关节的 position 始终为 0.0，说明有某个机制在持续将关节位置重置为 0。

**真正的原因：`JointTrajectoryController` 的初始化行为**

1. **控制器启动时读取当前的 `pos` 值作为"起始位置"**
   - `JointConfig` 结构体中 `pos` 的默认初始化值为 `0.0`
   - 控制器认为机械臂在零位

2. **控制器在没有收到轨迹时，会把 `posCmd` 设置为当前的 `pos`**
   - 为了"保持当前位置"，它持续发送 `posCmd = 0`

3. **`read()` 函数让 `pos` 追踪 `posCmd`**
   - `pos` 逐渐向 `posCmd` (0) 靠拢
   - 控制器看到 `pos` 在变化，又更新 `posCmd` 为新的 `pos`
   - 形成死循环：`pos` → 0，`posCmd` → 0，`pos` → 0...

4. **即使 MoveIt 发送了新目标，控制器也会因为"当前位置是 0"而规划从 0 开始的轨迹**
   - 导致机械臂一直在零位和目标位置之间跳变

---

## 最终解决方案：初始化关节位置为合理姿态

在 `CanDriverHW::init()` 中，**在注册 ros_control 接口之前**，给 vcan 设备的关节设置一个合理的初始姿态（UR5 的 home 姿态）：

```cpp
// --- vcan 仿真模式：初始化关节位置为非零姿态（避免 JointTrajectoryController 拉回零位）---
// UR5 home 姿态（从 HelloArm_moveit_config/config/ur5_robot.srdf 读取）
std::map<std::string, double> ur5HomePos = {
    {"shoulder_pan_joint",  0.0},
    {"shoulder_lift_joint", -2.8497},
    {"elbow_joint",          2.3862},
    {"wrist_1_joint",       -1.545},
    {"wrist_2_joint",        0.0},
    {"wrist_3_joint",        0.0}
};
for (auto &jc : joints_) {
    if (jc.canDevice.find("vcan") == 0 && ur5HomePos.count(jc.name)) {
        jc.pos = ur5HomePos[jc.name];
        jc.posCmd = jc.pos;  // 命令缓存也初始化为同样的值
    }
}
```

### 为什么这样有效？

- 控制器启动时认为机械臂在 home 姿态（而不是零位）
- `posCmd` 初始值也是 home 姿态
- `read()` 函数让 `pos` 追踪 `posCmd`，两者保持一致
- MoveIt 发送新轨迹时，控制器从 home 姿态平滑运动到目标位置
- 不再有"拉回零位"的问题

---

## 关键教训总结

### 1. ros_control 的初始状态很重要

`hardware_interface` 不仅仅是"数据通道"，**初始状态也会影响控制器行为**。控制器会基于初始的 `pos` 值做决策，如果初始值不合理（比如全零），就会导致意外行为。

### 2. URDF 依赖是控制器层的要求，不是硬件层的要求

- `CanDriverHW` 本身不需要 URDF（从 YAML 读取配置）
- `JointTrajectoryController` / `DiffDriveController` 需要 URDF
- 这是 ros_control 标准控制器的设计，ros_canopen 也是同样的架构

### 3. vcan 仿真需要模拟合理的初始状态

虚拟 CAN 接口无法提供真实反馈，因此需要在代码中：
- 初始化关节位置为合理姿态（不是全零）
- 在 `read()` 中模拟电机运动（让 `pos` 追踪 `posCmd`）

### 4. 调试 ros_control 问题的方法

1. 用 `rostopic echo /joint_states` 查看实际发布的关节状态
2. 用 `rosservice call /controller_manager/list_controllers` 查看控制器状态
3. 检查 `/robot_description` 参数是否正确加载
4. 检查关节名在 YAML、URDF、控制器配置中是否完全一致

---

## 测试结果

修复后，vcan 仿真测试成功：

✅ `can_driver_node` 启动无错误
✅ `joint_state_controller` 和 `arm_controller` 加载成功
✅ `/joint_states` 正常发布，初始姿态为 UR5 home 姿态
✅ MoveIt 能够规划并执行轨迹
✅ RViz 中机械臂平滑运动到目标位置，无跳变

---

## 后续工作

1. **实物联调（Phase 7）**：将 `vcan0`/`vcan1` 改回 `can0`/`can1`，连接真实 CAN 硬件
2. **移除 vcan 特殊逻辑**：实物联调时，`read()` 函数应该从协议层读取真实反馈，而不是模拟追踪
3. **调整初始姿态**：根据实际机器人的安全姿态，修改初始化代码中的关节角度

---

## 附录：相关文件修改记录

### 修改的文件

1. `launch/can_driver.launch` — 添加 URDF 加载，启用 `arm_controller`
2. `config/can_driver.yaml` — 关节名改为 UR5 的 6 个关节，`can_device` 改为 `vcan0`/`vcan1`
3. `config/ros_controllers.yaml` — `arm_controller` 的 joints 列表改为 UR5 关节
4. `src/CanDriverHW.cpp` — 添加 vcan 初始姿态设置，修改 `read()` 函数模拟电机运动

### Git 分支

- `main` 分支：保留原始配置（实际机器人的关节名和 `can0`/`can1`）
- `test/simulate` 分支：UR5 仿真配置（用于测试 MoveIt 集成）

---

**文档版本**：v1.0
**日期**：2026-02-26
**作者**：Claude Sonnet 4.6 + 用户
