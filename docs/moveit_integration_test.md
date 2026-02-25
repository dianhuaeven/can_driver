# can_driver + MoveIt 集成测试指南

## 概述

本测试使用软件回环模式验证 can_driver 与 MoveIt 的集成，无需真实 CAN 硬件。

## 架构

```
MoveIt move_group (规划器)
    ↓ FollowJointTrajectory action
arm_controller (JointTrajectoryController)
    ↓ PositionJointInterface
CanDriverHW::write() → 软件回环 → CanDriverHW::read()
    ↓ JointStateInterface
joint_state_controller → /joint_states
    ↓
RViz 可视化
```

## 软件回环模式

在 `CanDriverHW.cpp` 中启用：
```cpp
#define SOFTWARE_LOOPBACK_MODE 1
```

- **启用时**：`read()` 直接返回 `write()` 的命令值，模拟完美响应的电机
- **禁用时**：`read()` 从真实 CAN 总线读取反馈

## 启动测试

```bash
roslaunch can_driver demo_moveit.launch
```

可选参数：
- `use_rviz:=false` - 不启动 RViz（仅后台运行）
- `pipeline:=chomp` - 使用 CHOMP 规划器（默认 OMPL）

## 测试步骤

1. **启动后检查**
   ```bash
   # 检查控制器状态
   rosservice call /controller_manager/list_controllers

   # 应该看到：
   # - joint_state_controller: running
   # - arm_controller: running
   ```

2. **在 RViz 中测试**
   - 打开 MotionPlanning 插件
   - 拖动交互式标记（Interactive Marker）移动机器人
   - 点击 "Plan" 规划轨迹
   - 点击 "Execute" 执行轨迹
   - 观察机器人是否平滑移动到目标位置

3. **监控关节状态**
   ```bash
   # 查看关节状态
   rostopic echo /joint_states

   # 查看控制器命令
   rostopic echo /arm_controller/follow_joint_trajectory/goal
   ```

4. **测试直接命令**（可选）
   ```bash
   # 直接发送位置命令到单个关节（绕过 MoveIt）
   rostopic pub /motor/shoulder_pan_joint/cmd_position std_msgs/Float64 "data: 1.57"
   ```

## 预期行为

✅ **正常情况**：
- RViz 中机器人模型平滑移动
- `/joint_states` 实时更新，位置值接近命令值
- MoveIt 规划和执行无错误

❌ **异常情况**：
- 机器人不动 → 检查 `arm_controller` 是否 running
- 机器人跳变 → 检查初始位置是否匹配（vcan 模式已自动初始化为 home 姿态）
- 规划失败 → 检查目标位置是否在工作空间内

## 故障排查

### 控制器未启动
```bash
rosservice call /controller_manager/load_controller "name: 'arm_controller'"
rosservice call /controller_manager/switch_controller "{start_controllers: ['arm_controller'], stop_controllers: [], strictness: 2}"
```

### 查看 MoveIt 日志
```bash
rosrun rqt_console rqt_console
```

### 检查 TF 树
```bash
rosrun rqt_tf_tree rqt_tf_tree
# 应该看到：world → base_link → shoulder_link → ... → wrist_3_link
```

## 下一步：真实硬件测试

1. 关闭软件回环模式：
   ```cpp
   #define SOFTWARE_LOOPBACK_MODE 0  // 在 CanDriverHW.cpp 中
   ```

2. 修改 CAN 设备：
   ```yaml
   # 在 can_driver.yaml 中
   can_device: can0  # 改为真实 CAN 接口
   ```

3. 重新编译并启动：
   ```bash
   catkin_make
   roslaunch can_driver demo_moveit.launch
   ```

## 文件清单

- `launch/demo_moveit.launch` - 主启动文件
- `config/moveit_controllers.yaml` - MoveIt 控制器配置
- `config/can_driver.yaml` - 硬件参数（关节映射）
- `config/ros_controllers.yaml` - ros_control 控制器配置
- `src/CanDriverHW.cpp` - 硬件接口实现（包含软件回环逻辑）
