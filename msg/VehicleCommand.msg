# VehicleCommand 消息定义
# 用于发布非序列化的车辆控制命令，方便调试和查看

std_msgs/Header header

# 转向控制 (steering_target: [-800, 800])
float64 steering_target

# 油门控制 (throttle: [0, 200])
float64 throttle

# 刹车控制 (brake: [0, 1000])
float64 brake

# 档位 (0=NEUTRAL, 1=DRIVE, 2=REVERSE)
int32 gear_location

# 目标速度 (m/s)
float64 speed

# 大臂角度 (度，范围: -800~800)
float64 arm_angle
bool arm_enable

# 铲斗角度 (度，范围: -800~800)
float64 shovel_angle
bool shovel_enable

# 紧急停止
bool estop

# 驻车制动
bool parking_brake

# 发动机开关
bool engine_on_off

