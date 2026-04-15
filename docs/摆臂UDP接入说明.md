# 摆臂电机（swingarm）UDP 接入说明

本文说明如何将 `motor_driver_single_file.hpp` 中的摆臂电机（PH 协议）接入 `can_driver`。

> 已新增“多车 profile 控制接口”，无需改 C++ 代码即可在两台车切换调用。

## 1. 实现概览

本次接入新增了两层能力：

1. `UdpCanTransport`
   - 作用：把 `CanTransport::Frame` 封装成 13 字节 UDP 报文收发。
   - 兼容点：沿用你现有上位机的报文头布局（`[0]=0x08, [3..4]=CAN ID, [5..12]=8字节数据`）。

2. `EyouPhCan`
   - 作用：实现摆臂 PH 协议（CANopen SDO）控制。
   - 支持：`setMode / setVelocity / setPosition / setAcceleration / setDeceleration / Enable / Disable / Stop / 状态轮询`。

同时在协议枚举中新增了 `CanType::PH`，并在参数解析中支持：
- `protocol: PH`
- `protocol: PH_UDP`
- `protocol: SWINGARM`

## 2. 为什么这和现有架构兼容

`MtCan`、`EyouCan`、`EyouPhCan` 都复用了统一抽象 `CanTransport`。

- 现有 `SocketCanController`：走 Linux SocketCAN
- 新增 `UdpCanTransport`：走网络 UDP

这就是你提到的“socketcan 和网络 socket 本质都属于 socket I/O，需要做兼容层”的落地点。

## 3. 设备串格式（重点）

摆臂 PH 关节在 `can_device` 字段里不要写 `can0`，改成 UDP 设备串：

- `udp://<local_port>@<remote_ip>:<remote_port>`
- `udp://<local_ip>:<local_port>@<remote_ip>:<remote_port>`

示例：
- `udp://7101@192.168.1.253:1031`
- `udp://0.0.0.0:7101@192.168.1.253:1031`

## 4. joints 配置示例（swingarm）

```yaml
can_driver_node:
  joints:
    - name: swing_arm_left_forward
      motor_id: 0x601
      protocol: PH
      can_device: udp://7101@192.168.1.253:1031
      control_mode: velocity
      position_scale: 1.0
      velocity_scale: 1.0

    - name: swing_arm_right_forward
      motor_id: 0x602
      protocol: PH
      can_device: udp://7101@192.168.1.253:1031
      control_mode: velocity

    - name: swing_arm_left_backward
      motor_id: 0x603
      protocol: PH
      can_device: udp://7101@192.168.1.253:1031
      control_mode: velocity

    - name: swing_arm_right_backward
      motor_id: 0x604
      protocol: PH
      can_device: udp://7101@192.168.1.253:1031
      control_mode: velocity
```

> 建议：四个摆臂电机共用同一个 UDP 通道（同一个 `can_device`），保持与原先 Qt 驱动一致。

## 5. 控制行为对齐说明

对齐了你现有单文件驱动中的关键语义：

- `setVelocity` -> 写 `0x60FF`
- `setPosition` -> 写 `0x607A`
- `setAcceleration` -> 写 `0x6083`
- `setDeceleration` -> 写 `0x6084`
- `Stop/Disable` -> 控制字 `0x6040 = 0x0002`
- `Enable` -> `0x06 -> 0x07 -> 0x0F`（CiA402 常见使能流程）
- 轮询 -> 位置 `0x6064`、电流 `0x6077`、速度 `0x606C`、状态字 `0x6041`

## 6. 联调建议

1. 先只挂 1 个摆臂电机（如 `0x601`）做冒烟测试。
2. 用 `CMD_ENABLE` 使能后再发速度命令。
3. 观察 `~/motor_states` 的 `position/velocity/current/enabled/fault`。
4. 稳定后再扩到 4 个摆臂电机并发。

## 7. 常见问题

- 设备串解析失败：
  - 检查是否以 `udp://` 开头；
  - 是否包含 `@`；
  - 远端是否是 `ip:port`。

- 没有回包：
  - 检查本机 `local_port` 是否被占用；
  - 检查 UDP 网关 IP/端口和网段；
  - 检查防火墙。

- 使能后不动作：
  - 先确认模式（位置/速度）；
  - 再确认 `safety_require_enabled_for_motion` 参数是否阻断；
  - 检查状态字故障位。

## 8. 多车 PH 控制接口（避免改代码）

### 8.1 profile 配置文件

配置文件：`config/ph_control_profiles.yaml`

已预置两套配置：
- `car_a`
- `car_b`

你只需要修改各自 `can_device` 即可区分两台车，无需改协议代码。

### 8.2 接口脚本

接口脚本：`scripts/ph_motor_interface.py`

示例：

1) 查看 profile

```bash
rosrun can_driver ph_motor_interface.py --profile car_a --action list
```

2) 使能 + 切速度模式 + 下发速度

```bash
rosrun can_driver ph_motor_interface.py --profile car_a --action enable   --motor-id 0x601
rosrun can_driver ph_motor_interface.py --profile car_a --action mode     --motor-id 0x601 --value 1
rosrun can_driver ph_motor_interface.py --profile car_a --action velocity --motor-id 0x601 --value 2.0
```

3) 停止 + 失能

```bash
rosrun can_driver ph_motor_interface.py --profile car_a --action stop    --motor-id 0x601
rosrun can_driver ph_motor_interface.py --profile car_a --action disable --motor-id 0x601
```

### 8.3 一键运动测试脚本（sh）

测试脚本：`scripts/test_ph_motor_motion.sh`

示例：

```bash
bash scripts/test_ph_motor_motion.sh car_a 0x601 2.0 2.0
```

流程：
- Enable
- Mode=Velocity
- 正转 `duration`
- 反转 `duration`
- Stop
- Disable
