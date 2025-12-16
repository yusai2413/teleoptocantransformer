# teleoptocantransformer

这个包将远程端的控制指令（JSON 格式）转换为 cannode 可以接收的控制指令（protobuf 格式）。

## 功能

- 订阅 `/controls/teleop` 话题（`BEST_EFFORT` QoS），接收 JSON 格式的控制命令
- 支持完整的 `ExcavatorControls` 接口，包括挖掘机控制和装载机控制
- 将控制命令转换为 `ControlCommand` protobuf 消息
- 发布到 `/vehicle_command` 话题（`RELIABLE` QoS）供 cannode 使用
- 实现范围映射、死区处理和速度模式控制
- 自动计算目标速度（根据油门、档位和速度模式）
- 定期检查连接状态并输出日志

## 输入格式

订阅 `/controls/teleop` 话题（`std_msgs/String`），支持 `ExcavatorControls` 接口的完整 JSON 格式：

```json
{
  "leftTrack": 0.3,
  "rightTrack": 0.3,
  "swing": 0.0,
  "boom": 0.5,
  "stick": -0.3,
  "bucket": 0.4,
  "steering": 0.2,
  "throttle": 0.6,
  "brake": 0.0,
  "gear": "D",
  "speed_mode": "rabbit",
  "emergency_stop": false,
  "parking_brake": false,
  "horn": false,
  "light_code": 5,
  "hydraulic_lock": false,
  "power_enable": true
}
```

### 字段说明

#### 通用挖掘臂控制
- `leftTrack`: 左履带，范围 [-1, 1]，-1 后退，1 前进（保留格式，不映射到输出）
- `rightTrack`: 右履带，范围 [-1, 1]，-1 后退，1 前进（保留格式，不映射到输出）
- `swing`: 驾驶室旋转，范围 [-1, 1]，-1 左转，1 右转（保留格式，不映射到输出）
- `boom`: 大臂控制，范围 [-1, 1]，-1 降，1 提 → 映射到 `arm_angle`
- `stick`: 小臂控制，范围 [-1, 1]，-1 收，1 伸（保留格式，不映射到输出）
- `bucket`: 铲斗控制，范围 [-1, 1]，-1 收，1 翻 → 映射到 `shovel_angle`

#### 装载机/线控底盘扩展信号
- `steering`: 铰接转向，范围 [-1, 1]，-1 左转，1 右转 → 映射到 `steering_target`（反向）
- `throttle`: 油门，范围 [0, 1] → 映射到 `throttle`（百分比）
- `brake`: 刹车，范围 [0, 1] → 映射到 `brake`（百分比）

#### 关键辅助信号
- `gear`: 档位，"N"（空档）、"D"（前进）、"R"（后退） → 映射到 `gear_location`
- `speed_mode`: 速度模式，"turtle"（乌龟，50%速度）或 "rabbit"（兔子，100%速度） → 影响 `max_speed` 计算
- `emergency_stop`: 紧急急停，布尔值 → 映射到 `estop`
- `parking_brake`: 停车制动，布尔值 → 映射到 `parking_brake`
- `power_enable`: 上高压，布尔值 → 映射到 `engine_on_off`
- `horn`: 喇叭，布尔值（保留格式，不映射到输出）
- `light_code`: 灯光代码，位掩码 0-31（0x01:左转, 0x02:右转, 0x04:远光, 0x08:近光, 0x10:工作灯）（保留格式，不映射到输出）
- `hydraulic_lock`: 液压锁，布尔值（保留格式，不映射到输出）

## 输出格式

发布到 `/vehicle_command` 话题（`sa_msgs::msg::ProtoAdapter`），包含序列化的 `control::ControlCommand` protobuf 消息。

### 映射关系

#### 映射到输出的字段
- `steering: [-1, 1]` → `steering_target: [-800, 800]`（反向映射）
- `throttle: [0, 1]` → `throttle: [0, 100]%`
- `brake: [0, 1]` → `brake: [0, 100]%`
- `gear: "N"/"D"/"R"` → `gear_location: 2(N)/1(D)/3(R)`
- `boom: [-1, 1]` → `arm_angle: [arm_angle_min, arm_angle_max]°`（启用 `arm_enable`）
- `bucket: [-1, 1]` → `shovel_angle: [shovel_angle_min, shovel_angle_max]°`（反向，启用 `shovel_enable`）
- `emergency_stop: boolean` → `estop: boolean`
- `parking_brake: boolean` → `parking_brake: boolean`
- `power_enable: boolean` → `engine_on_off: boolean`
- `speed_mode: "turtle"/"rabbit"` → 影响 `max_speed` 计算（turtle=50%, rabbit=100%）
- 根据 `throttle` 和 `gear` 自动计算 `speed: [-max_speed, max_speed] m/s`

#### 保留格式但不映射的字段
以下字段会被接收和记录日志，但不会映射到 `ControlCommand` 输出：
- `leftTrack`, `rightTrack`, `swing`, `stick`: 挖掘机专用控制（cannode 不支持）
- `light_code`: 灯光代码（cannode 的灯光字段已 deprecated）
- `hydraulic_lock`: 液压锁（cannode 不支持）
- `horn`: 喇叭（cannode 的 horn 字段已 deprecated）

## 死区处理

所有控制输入都应用死区处理，避免小幅度抖动：

- 死区内的值被映射为 0
- 死区外的值线性缩放到有效范围
- 默认死区大小：0.05（可通过参数调整）

## 速度模式

`speed_mode` 字段影响最大速度计算：

- `"rabbit"`（兔子模式）：使用 100% 的 `max_speed`
- `"turtle"`（乌龟模式）：使用 50% 的 `max_speed`
- 未指定时：默认使用 100% 的 `max_speed`

目标速度计算公式：
- 前进档（D）：`speed = (throttle / 100.0) * effective_max_speed`
- 倒车档（R）：`speed = -(throttle / 100.0) * effective_max_speed`
- 空档（N）：`speed = 0`

## 参数配置

可通过 launch 文件或 ROS2 参数配置：

- `steering_deadzone`: 转向死区（默认 0.05）
- `throttle_deadzone`: 油门死区（默认 0.05）
- `brake_deadzone`: 刹车死区（默认 0.05）
- `boom_deadzone`: 大臂死区（默认 0.05）
- `bucket_deadzone`: 铲斗死区（默认 0.05）
- `arm_angle_min`: 大臂最小角度（度，默认 -60.0）
- `arm_angle_max`: 大臂最大角度（度，默认 60.0）
- `shovel_angle_min`: 铲斗最小角度（度，默认 0.0）
- `shovel_angle_max`: 铲斗最大角度（度，默认 60.0）
- `max_speed`: 最大速度（m/s，默认 3.0）

## 使用方法

### 编译

```bash
cd /home/cyber007/cannode_ws
colcon build --packages-select teleoptocantransformer
source install/setup.bash
```

### 运行

```bash
ros2 launch teleoptocantransformer teleop_converter.launch.py
```

### 测试

可以使用以下命令发布测试消息：

```bash
# 基础测试
ros2 topic pub /controls/teleop std_msgs/String "data: '{\"steering\": 0.5, \"throttle\": 0.8, \"gear\": \"D\"}'"

# 完整格式测试
ros2 topic pub /controls/teleop std_msgs/String "data: '{\"leftTrack\": 0.3, \"rightTrack\": 0.3, \"boom\": 0.5, \"bucket\": 0.4, \"steering\": 0.2, \"throttle\": 0.6, \"brake\": 0.0, \"gear\": \"D\", \"speed_mode\": \"rabbit\", \"power_enable\": true}'"
```

## 测试程序

提供了一个完整的测试程序 (`scripts/test_converter.py`)，包含所有测试功能：

```bash
python3 src/teleoptocantransformer/scripts/test_converter.py
```

### 功能特性

1. **模拟输入信息及其范围**（支持完整的 `ExcavatorControls` 接口）
   - 通用挖掘臂控制：leftTrack, rightTrack, swing, boom, stick, bucket: [-1.0, 1.0]
   - 装载机/线控底盘：steering: [-1.0, 1.0], throttle/brake: [0.0, 1.0]
   - 关键辅助信号：gear ('N'/'D'/'R'), speed_mode ('turtle'/'rabbit'), emergency_stop, parking_brake, horn, light_code, hydraulic_lock, power_enable

2. **验证输出信息及范围**（参考 `control_cmd.proto` 和 cannode 实现）
   - steering_target: [-800, 800]
   - throttle: [0, 100]%
   - brake: [0, 100]%
   - arm_angle: [0, 60]°（cannode限制）
   - shovel_angle: [0, 60]°
   - gear_location: 1(D), 2(N), 3(R)
   - speed: [-max_speed, max_speed] m/s（根据 speed_mode 调整）

3. **测试用例类型**（共 40+ 个测试用例）
   - 基础功能测试（空档、前进、倒车等）
   - 新格式字段测试（履带、驾驶室旋转、小臂、速度模式、灯光代码、液压锁、喇叭）
   - 边界值测试（最小值、最大值）
   - 死区测试（验证死区处理）
   - 档位测试（N/D/R）
   - 大臂/铲斗测试（角度范围）
   - 超出范围测试（验证限制功能）
   - 特殊功能测试（紧急停止、驻车制动、发动机开关）
   - 速度范围测试（前进/倒车全油门）
   - 新格式完整组合测试（所有字段组合）

4. **自动验证**
   - 解析 protobuf 消息
   - 验证期望值与实际值
   - 验证输出范围
   - 生成测试报告和统计

## QoS 配置

- **订阅话题** `/controls/teleop`：
  - QoS: `BEST_EFFORT`（匹配远程端发布者）
  - 深度: 10
  - 历史策略: `KEEP_LAST`

- **发布话题** `/vehicle_command`：
  - QoS: `RELIABLE`（确保 cannode 可靠接收）
  - 深度: 10
  - 历史策略: `KEEP_LAST`

## 日志输出

节点会输出详细的转换日志，包括：
- 接收到的原始 JSON 输入
- 转换后的 protobuf 字段值
- 范围验证警告（如果值超出预期范围）
- 新格式字段信息（保留格式但不映射的字段）
- 连接状态检查（每 5 秒检查一次发布者/订阅者数量）

## 注意事项

1. 确保 cannode 包已编译，protobuf 文件已生成
2. 确保 `/vehicle_command` 话题的订阅者（cannode）正在运行
3. 角度映射范围可根据实际车辆参数调整（通过 launch 文件参数）
4. 死区大小可根据控制精度需求调整（通过 launch 文件参数）
5. 速度模式会影响最大速度，请根据实际需求选择 "turtle" 或 "rabbit"
6. 部分字段（leftTrack, rightTrack, swing, stick, light_code, hydraulic_lock, horn）会被接收但不会映射到输出，这些字段仅用于日志记录
7. 测试前确保 teleop2can_transformer 节点正在运行：
   ```bash
   ros2 launch teleoptocantransformer teleop_converter.launch.py
   ```
