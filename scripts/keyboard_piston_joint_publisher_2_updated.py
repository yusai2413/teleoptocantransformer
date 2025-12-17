#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math
import threading
import json

class KeyboardJointPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_joint_publisher')
        
        # Create publisher for joint states
        # 设置QoS配置（使用reliable可靠性，确保关节状态可靠传输）
        joint_qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.joint_pub = self.create_publisher(JointState, '/joint_command', joint_qos_profile)
        
        # Subscribe to teleop control topic (JSON in std_msgs/String)
        # 设置QoS配置以兼容发布者
        # 根据错误信息，发布者可能使用BEST_EFFORT，所以订阅者也使用BEST_EFFORT
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 使用best_effort以匹配发布者
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.teleop_sub = self.create_subscription(
            StringMsg,
            '/controls/teleop',
            self.teleop_callback,
            qos_profile
        )
        self.get_logger().info(f'订阅 /controls/teleop 话题，QoS: reliability=BEST_EFFORT, depth=10')
        
        # Initialize joint states
        self.joint_names = ['chassis_body_revolute', 'RLwheel_revolute', 'RRwheel_revolute', 'FLwheel_revolute', 'FRwheel_revolute', 'bucket_cylinder_prismatic', 'boom_cylinder_prismatic']
        
        # Initialize control values
        self.steer_position = 0.0  # Position control for chasis_body_revolute
        self.bucket_prismatic_length = 0.0  # Length for bucket_cylinder_prismatic
        self.arm_prismatic_length = 0.0    # Length for boom_cylinder_prismatic
        self.wheel_velocity = 0.0  # Velocity for wheel joints (starts at 0)
        
        # Step sizes (per timer callback, i.e., every 0.1 seconds)
        self.steer_limit = 1.57  # radians, [-1.57, 1.57]
        self.bucket_min, self.bucket_max = -700.0, 100.0
        self.arm_min, self.arm_max = -1300.0, 100.0
        self.velocity_step = 0.5
        self.velocity_min, self.velocity_max = -3.0, 3.0
        
        # Threshold for output change detection
        self.output_epsilon = 1e-6
        
        # Previous values for change detection (printing only on change)
        self.prev_values = {
            'steer_position': None,
            'bucket_prismatic_length': None,
            'arm_prismatic_length': None,
            'wheel_velocity': None,
        }
        
        # Latest controls received from teleop topic (支持新格式的所有字段)
        # 根据 ExcavatorControls 接口定义
        self.latest_controls = {
            # --- 通用挖掘臂控制 ---
            'leftTrack': 0.0,      # 左履带: -1 (后) to 1 (前)
            'rightTrack': 0.0,     # 右履带: -1 (后) to 1 (前)
            'swing': 0.0,          # 驾驶室旋转: -1 (左) to 1 (右)
            'boom': 0.0,           # 大臂: -1 (降) to 1 (提)
            'stick': 0.0,          # 小臂: -1 (收) to 1 (伸)
            'bucket': 0.0,         # 铲斗: -1 (收) to 1 (翻)
            
            # --- 装载机/线控底盘扩展信号 ---
            'steering': 0.0,        # 铰接转向: -1 (左) to 1 (右)
            'throttle': 0.0,       # 油门: 0 to 1
            'brake': 0.0,          # 刹车: 0 to 1
            
            # --- 关键辅助信号 ---
            'emergency_stop': False,  # 紧急急停 (红色蘑菇头)
            'parking_brake': False,  # 停车制动 (手刹)
            'horn': False,            # 喇叭
            'gear': 'N',              # 档位: 'N' | 'D' | 'R'
            'speed_mode': 'turtle',   # 速度模式: 'turtle' | 'rabbit'
            
            # --- 灯光信号 ---
            'light_code': 0,          # 位掩码: 0x01左转, 0x02右转, 0x04远光, 0x08近光, 0x10工作灯
            
            # --- 其他安全信号 ---
            'hydraulic_lock': False,  # 液压锁
            'power_enable': True,     # 上高压
            
            # 兼容旧格式的字段
            'rotation': 0.0,          # 兼容旧格式，与steering同步
            'left_track': 0.0,        # 兼容旧格式，与leftTrack同步
            'right_track': 0.0,       # 兼容旧格式，与rightTrack同步
            'device_type': 'excavator',  # 设备类型
            'timestamp': 0,           # 时间戳
        }
        
        # Create timer (kept but no longer publishes; event-driven publish on input changes)
        self.create_timer(1.0/30.0, self.timer_callback)
        
        self.get_logger().info('Teleop Joint Publisher started (listening to /controls/teleop with ExcavatorControls interface support)')

    def teleop_callback(self, msg: StringMsg):
        try:
            data = json.loads(msg.data)
            # 打印接收到的原始数据
            self.get_logger().info(f'收到控制命令: {json.dumps(data, indent=2, ensure_ascii=False)}')
            
            # Update only known keys; clamp to valid ranges
            def clamp(val, min_v, max_v):
                return max(min_v, min(max_v, val))
            
            # Track changes only for specified keys
            # 注意：steering和rotation是同步的，需要同时检测两者
            watched_float_keys = ['rotation', 'brake', 'throttle', 'boom', 'bucket']
            watched_str_keys = ['gear']
            changed_flag = False
            # Snapshot before (包含rotation，因为steering会同步到rotation)
            before = {k: self.latest_controls.get(k) for k in watched_float_keys + watched_str_keys}
            
            # --- 处理转向控制：优先使用steering字段（新格式），如果没有则使用rotation（兼容旧格式） ---
            if 'steering' in data:
                new_steering = clamp(float(data['steering']), -1.0, 1.0)
                old_rotation = before.get('rotation', 0.0)  # 使用snapshot中的旧值
                self.latest_controls['steering'] = new_steering
                # 同步到rotation字段，用于后续计算
                self.latest_controls['rotation'] = new_steering
                # 如果steering变化导致rotation变化，需要触发更新
                if old_rotation is None or abs(float(old_rotation) - float(new_steering)) > self.output_epsilon:
                    changed_flag = True
            elif 'rotation' in data:
                new_rotation = clamp(float(data['rotation']), -1.0, 1.0)
                self.latest_controls['rotation'] = new_rotation
                self.latest_controls['steering'] = new_rotation
            
            # --- 装载机/线控底盘扩展信号 ---
            if 'throttle' in data:
                self.latest_controls['throttle'] = clamp(float(data['throttle']), 0.0, 1.0)
            if 'brake' in data:
                self.latest_controls['brake'] = clamp(float(data['brake']), 0.0, 1.0)
            
            # --- 通用挖掘臂控制 ---
            if 'boom' in data:
                self.latest_controls['boom'] = clamp(float(data['boom']), -1.0, 1.0)
            if 'bucket' in data:
                self.latest_controls['bucket'] = clamp(float(data['bucket']), -1.0, 1.0)
            if 'stick' in data:
                self.latest_controls['stick'] = clamp(float(data['stick']), -1.0, 1.0)
            if 'swing' in data:
                self.latest_controls['swing'] = clamp(float(data['swing']), -1.0, 1.0)
            
            # 处理履带控制：支持驼峰命名（新格式）和下划线命名（兼容旧格式）
            if 'leftTrack' in data:
                self.latest_controls['leftTrack'] = clamp(float(data['leftTrack']), -1.0, 1.0)
                self.latest_controls['left_track'] = self.latest_controls['leftTrack']  # 同步到旧格式
            elif 'left_track' in data:
                self.latest_controls['left_track'] = clamp(float(data['left_track']), -1.0, 1.0)
                self.latest_controls['leftTrack'] = self.latest_controls['left_track']  # 同步到新格式
            
            if 'rightTrack' in data:
                self.latest_controls['rightTrack'] = clamp(float(data['rightTrack']), -1.0, 1.0)
                self.latest_controls['right_track'] = self.latest_controls['rightTrack']  # 同步到旧格式
            elif 'right_track' in data:
                self.latest_controls['right_track'] = clamp(float(data['right_track']), -1.0, 1.0)
                self.latest_controls['rightTrack'] = self.latest_controls['right_track']  # 同步到新格式
            
            # --- 关键辅助信号 ---
            if 'gear' in data:
                gear_value = str(data['gear'])
                # 验证gear值是否有效
                if gear_value in ['N', 'D', 'R']:
                    self.latest_controls['gear'] = gear_value
                else:
                    self.get_logger().warn(f'Invalid gear value: {gear_value}, expected N/D/R')
            
            if 'speed_mode' in data:
                speed_mode_value = str(data['speed_mode'])
                # 验证speed_mode值是否有效
                if speed_mode_value in ['turtle', 'rabbit']:
                    self.latest_controls['speed_mode'] = speed_mode_value
                else:
                    self.get_logger().warn(f'Invalid speed_mode value: {speed_mode_value}, expected turtle/rabbit')
            
            if 'emergency_stop' in data:
                self.latest_controls['emergency_stop'] = bool(data['emergency_stop'])
            if 'parking_brake' in data:
                self.latest_controls['parking_brake'] = bool(data['parking_brake'])
            if 'horn' in data:
                self.latest_controls['horn'] = bool(data['horn'])
            
            # --- 灯光信号 ---
            if 'light_code' in data:
                self.latest_controls['light_code'] = int(data['light_code'])
            
            # --- 其他安全信号 ---
            if 'hydraulic_lock' in data:
                self.latest_controls['hydraulic_lock'] = bool(data['hydraulic_lock'])
            if 'power_enable' in data:
                self.latest_controls['power_enable'] = bool(data['power_enable'])
            
            # 兼容旧格式的字段
            if 'device_type' in data:
                self.latest_controls['device_type'] = str(data['device_type'])
            if 'timestamp' in data:
                self.latest_controls['timestamp'] = int(data['timestamp'])
            
            # Detect changes
            # 如果steering已经触发了变化，就不需要再检查了
            if not changed_flag:
                for k in watched_float_keys:
                    if k in data:
                        prev_v = before.get(k)
                        new_v = self.latest_controls.get(k)
                        if prev_v is None or abs(float(prev_v) - float(new_v)) > self.output_epsilon:
                            changed_flag = True
                            self.get_logger().info(f'检测到变化: {k} 从 {prev_v} 变为 {new_v}')
                            break
            if not changed_flag:
                for k in watched_str_keys:
                    if k in data:
                        if before.get(k) != self.latest_controls.get(k):
                            changed_flag = True
                            self.get_logger().info(f'检测到变化: {k} 从 {before.get(k)} 变为 {self.latest_controls.get(k)}')
                            break
            
            # 打印当前的控制值（仅显示关键字段）
            self.get_logger().info(f'当前控制值 - steering: {self.latest_controls.get("steering"):.3f}, '
                                  f'throttle: {self.latest_controls.get("throttle"):.3f}, '
                                  f'brake: {self.latest_controls.get("brake"):.3f}, '
                                  f'boom: {self.latest_controls.get("boom"):.3f}, '
                                  f'bucket: {self.latest_controls.get("bucket"):.3f}, '
                                  f'gear: {self.latest_controls.get("gear")}')
            
            # If relevant inputs changed, update states and publish once
            if changed_flag:
                self.get_logger().info('检测到相关输入变化，更新关节状态并发布...')
                self.update_positions_and_velocity()
                self.publish_joint_state()
            else:
                self.get_logger().info('未检测到相关输入变化，跳过更新')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse /controls/teleop JSON: {e}')
            import traceback
            self.get_logger().error(f'错误详情: {traceback.format_exc()}')

    def update_positions_and_velocity(self):
        # Map rotation (-1..1) to steering position [-1.57..1.57] (inverted)
        # 使用rotation字段进行计算（保持输出逻辑不变）
        rotation = float(self.latest_controls['rotation'])
        self.steer_position = max(-self.steer_limit, min(self.steer_limit, -rotation * self.steer_limit))
        self.get_logger().info(f'计算转向位置: rotation={rotation:.3f} -> steer_position={self.steer_position:.3f} rad ({self.steer_position*180/3.14159:.1f}°)')

        # Map bucket (-1..1) to bucket prismatic length (-1..1), inverted
        bucket = float(self.latest_controls['bucket'])
        self.bucket_prismatic_length = -bucket
        self.get_logger().info(f'计算铲斗位置: bucket={bucket:.3f} -> bucket_prismatic_length={self.bucket_prismatic_length:.3f}')

        # Map boom (-1..1) to arm prismatic length (-1..1)
        boom = float(self.latest_controls['boom'])
        self.arm_prismatic_length = boom
        self.get_logger().info(f'计算臂位置: boom={boom:.3f} -> arm_prismatic_length={self.arm_prismatic_length:.3f}')

        # Update wheel velocity using throttle/brake integration
        # 根据档位控制前进/后退
        throttle = float(self.latest_controls['throttle'])  # 0..1
        brake = float(self.latest_controls['brake'])        # 0..1
        gear = str(self.latest_controls.get('gear', 'N'))    # 获取档位
        
        old_velocity = self.wheel_velocity
        
        # 根据档位决定速度方向
        if gear == 'D':  # 前进档
            # D档：前进操作，速度为正
            delta_v = (throttle - brake) * self.velocity_step
            # 如果当前速度为负（后退状态），先归零再加速
            if self.wheel_velocity < 0:
                self.wheel_velocity = min(0.0, self.wheel_velocity + brake * self.velocity_step)
            else:
                self.wheel_velocity = max(0.0, min(self.velocity_max, self.wheel_velocity + delta_v))
            direction = "前进"
        elif gear == 'R':  # 后退档
            # R档：后退操作，速度为负
            delta_v = (throttle - brake) * self.velocity_step
            # 如果当前速度为正（前进状态），先归零再加速
            if self.wheel_velocity > 0:
                self.wheel_velocity = max(0.0, self.wheel_velocity - brake * self.velocity_step)
            else:
                self.wheel_velocity = min(0.0, max(self.velocity_min, self.wheel_velocity - delta_v))
            direction = "后退"
        else:  # N档或其他档位
            # 其他档位：油门设置为0，速度逐渐归零
            if abs(self.wheel_velocity) > self.output_epsilon:
                # 如果当前速度不为0，逐渐减速到0
                if self.wheel_velocity > 0:
                    self.wheel_velocity = max(0.0, self.wheel_velocity - brake * self.velocity_step)
                else:
                    self.wheel_velocity = min(0.0, self.wheel_velocity + brake * self.velocity_step)
            else:
                self.wheel_velocity = 0.0
            direction = "空档(停止)"
        
        self.get_logger().info(f'计算轮速: 档位={gear}, throttle={throttle:.3f}, brake={brake:.3f}, '
                              f'velocity: {old_velocity:.3f} -> {self.wheel_velocity:.3f} ({direction})')

        # Only output when values change beyond epsilon
        def changed(a, b):
            if a is None or b is None:
                return True
            return abs(a - b) > self.output_epsilon

        if (
            changed(self.prev_values['bucket_prismatic_length'], self.bucket_prismatic_length) or
            changed(self.prev_values['arm_prismatic_length'], self.arm_prismatic_length) or
            changed(self.prev_values['steer_position'], self.steer_position) or
            changed(self.prev_values['wheel_velocity'], self.wheel_velocity)
        ):
            print("bucket_prismatic_length: ", self.bucket_prismatic_length, "arm_prismatic_length: ", self.arm_prismatic_length)
            print("Chasis angle: ", self.steer_position, "(", self.steer_position*180/3.14159, ")", "Wheel velocity: ", self.wheel_velocity)
            print("-----------------------------------------------------")
            self.prev_values['bucket_prismatic_length'] = self.bucket_prismatic_length
            self.prev_values['arm_prismatic_length'] = self.arm_prismatic_length
            self.prev_values['steer_position'] = self.steer_position
            self.prev_values['wheel_velocity'] = self.wheel_velocity


    def timer_callback(self):
        # Event-driven模式下，定时器不发布，仅保持节点活跃
        return

    def publish_joint_state(self):
        # Publish joint states once (call when inputs changed)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            self.steer_position,
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            self.bucket_prismatic_length,
            self.arm_prismatic_length
        ]
        msg.velocity = [
            float('nan'),
            self.wheel_velocity,    # RLwheel_revolute
            self.wheel_velocity,   # RRwheel_revolute
            self.wheel_velocity,    # FLwheel_revolute
            self.wheel_velocity,   # FRwheel_revolute
            float('nan'),
            float('nan'),
        ]
        msg.effort = [0.0] * len(self.joint_names)
        
        # 打印发布的关节状态信息
        self.get_logger().info('=' * 60)
        self.get_logger().info('发布关节状态到 /joint_command:')
        self.get_logger().info(f'  关节名称: {self.joint_names}')
        self.get_logger().info(f'  位置: {[f"{p:.3f}" if not (isinstance(p, float) and p != p) else "NaN" for p in msg.position]}')
        self.get_logger().info(f'  速度: {[f"{v:.3f}" if not (isinstance(v, float) and v != v) else "NaN" for v in msg.velocity]}')
        self.get_logger().info(f'  详细: steer_position={self.steer_position:.3f} rad, '
                              f'bucket={self.bucket_prismatic_length:.3f}, '
                              f'boom={self.arm_prismatic_length:.3f}, '
                              f'wheel_velocity={self.wheel_velocity:.3f}')
        self.get_logger().info('=' * 60)
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

