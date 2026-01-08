#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本：测试 teleop_converter_xiaosong 的功能
- 发布 JSON 格式的控制指令到 /controls/teleop
- 订阅 /vehicle_command 和 /vehicle_command_debug 验证转换结果
- 验证映射规则：-1到0映射到700~0，0到1映射到0~700
"""

import json
import sys
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

try:
    from sa_msgs.msg import ProtoAdapter
    HAS_SA_MSGS = True
except ImportError:
    print("警告: 无法导入 sa_msgs，将无法解析 protobuf 消息")
    HAS_SA_MSGS = False

# 尝试导入 protobuf
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    protobuf_path = os.path.join(script_dir, '../../cannode/protobuf/out')
    if os.path.exists(protobuf_path) and protobuf_path not in sys.path:
        sys.path.insert(0, protobuf_path)
    
    from control_msgs.control_cmd_pb2 import ControlCommand
    PROTOBUF_AVAILABLE = True
except ImportError as e:
    print(f"警告: 无法导入 protobuf 模块: {e}")
    PROTOBUF_AVAILABLE = False


class XiaosongConverterTester(Node):
    def __init__(self):
        super().__init__('xiaosong_converter_tester')
        
        # QoS 配置
        qos_profile_teleop = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # 创建发布者
        self.pub = self.create_publisher(StringMsg, '/controls/teleop', qos_profile_teleop)
        self.get_logger().info('已创建发布者 /controls/teleop')
        
        # 创建订阅者
        if HAS_SA_MSGS:
            self.sub = self.create_subscription(
                ProtoAdapter,
                '/vehicle_command',
                self.vehicle_cmd_callback,
                reliable_qos
            )
            self.get_logger().info('已订阅 /vehicle_command')
        
        # 订阅调试消息
        self.debug_sub = self.create_subscription(
            StringMsg,
            '/vehicle_command_debug',
            self.debug_callback,
            reliable_qos
        )
        self.get_logger().info('已订阅 /vehicle_command_debug')
        
        # 存储接收到的消息
        self.received_commands = []
        self.received_debug = []
        self.test_results = []
        
        # 等待连接并处理一些消息
        self.get_logger().info('等待连接...')
        for i in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.5)
        self.get_logger().info(f'发布者连接状态: {self.pub.get_subscription_count()} 个订阅者')
        # 注意：ROS2 订阅者对象没有 get_publisher_count() 方法
        # 可以通过检查是否收到消息来判断连接状态
    
    def vehicle_cmd_callback(self, msg):
        """处理接收到的 protobuf 消息"""
        if not PROTOBUF_AVAILABLE:
            self.get_logger().warn('Protobuf 不可用，无法解析消息')
            return
        
        try:
            # 反序列化 protobuf
            # msg.pb 是 bytes 列表，需要转换为 bytes
            if isinstance(msg.pb, list):
                pb_bytes = b''.join(msg.pb)
            else:
                pb_bytes = bytes(msg.pb)
            
            cmd = ControlCommand()
            cmd.ParseFromString(pb_bytes)
            
            result = {
                'timestamp': time.time(),
                'arm_up_current': cmd.arm_up_current,
                'arm_down_current': cmd.arm_down_current,
                'stick_retract_current': cmd.stick_retract_current,
                'stick_extend_current': cmd.stick_extend_current,
                'bucket_close_current': cmd.bucket_close_current,
                'bucket_dump_current': cmd.bucket_dump_current,
                'rotate_left_current': cmd.rotate_left_current,
                'rotate_right_current': cmd.rotate_right_current,
                'left_track_forward_current': cmd.left_track_forward_current,
                'left_track_backward_current': cmd.left_track_backward_current,
                'right_track_forward_current': cmd.right_track_forward_current,
                'right_track_backward_current': cmd.right_track_backward_current,
            }
            
            self.received_commands.append(result)
            
            self.get_logger().info(
                f'收到 protobuf: arm_up={cmd.arm_up_current:.1f}, '
                f'arm_down={cmd.arm_down_current:.1f}, '
                f'stick_retract={cmd.stick_retract_current:.1f}, '
                f'stick_extend={cmd.stick_extend_current:.1f}'
            )
        except Exception as e:
            self.get_logger().error(f'解析 protobuf 失败: {e}')
            import traceback
            traceback.print_exc()
    
    def debug_callback(self, msg):
        """处理接收到的调试消息"""
        try:
            data = json.loads(msg.data)
            self.received_debug.append({
                'timestamp': time.time(),
                'data': data
            })
        except Exception as e:
            self.get_logger().error(f'解析调试消息失败: {e}')
    
    def send_test_command(self, test_name, command_dict):
        """发送测试命令并等待响应"""
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'测试: {test_name}')
        self.get_logger().info(f'输入: {json.dumps(command_dict, indent=2)}')
        
        # 清空之前的接收记录
        self.received_commands.clear()
        self.received_debug.clear()
        
        # 发布命令
        json_str = json.dumps(command_dict)
        msg = StringMsg()
        msg.data = json_str
        self.pub.publish(msg)
        self.get_logger().info(f'已发布命令，等待响应...')
        
        # 等待响应（增加等待时间，并处理 ROS2 消息）
        start_time = time.time()
        timeout = 1.0
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.received_commands) > 0 or len(self.received_debug) > 0:
                break
        
        # 验证结果
        if len(self.received_commands) > 0:
            result = self.received_commands[-1]
            self.get_logger().info(f'输出: {result}')
            return result
        elif len(self.received_debug) > 0:
            self.get_logger().warn('收到调试消息但未收到 protobuf 消息')
            self.get_logger().info(f'调试消息: {self.received_debug[-1]}')
            return None
        else:
            self.get_logger().warn('未收到响应')
            self.get_logger().warn(f'发布者订阅者数: {self.pub.get_subscription_count()}')
            # 注意：ROS2 订阅者对象没有 get_publisher_count() 方法
            # 如果未收到消息，可能是 converter 节点未运行或未发布消息
            return None
    
    def verify_mapping(self, input_value, expected_current, actual_current, direction, tolerance=5.0):
        """验证映射是否正确"""
        error = abs(actual_current - expected_current)
        if error <= tolerance:
            self.get_logger().info(f'  ✓ {direction}: 输入={input_value:.2f}, 期望={expected_current:.1f}mA, 实际={actual_current:.1f}mA, 误差={error:.1f}mA')
            return True
        else:
            self.get_logger().error(f'  ✗ {direction}: 输入={input_value:.2f}, 期望={expected_current:.1f}mA, 实际={actual_current:.1f}mA, 误差={error:.1f}mA (超出容差)')
            return False
    
    def test_boom_mapping(self):
        """测试大臂映射：-1到0映射到700~0，0到1映射到0~700"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('测试大臂 (boom) 映射')
        self.get_logger().info('='*60)
        
        test_cases = [
            (-1.0, 700.0, 0.0),    # -1 -> arm_down=700, arm_up=0
            (-0.5, 350.0, 0.0),    # -0.5 -> arm_down=350, arm_up=0
            (0.0, 0.0, 0.0),       # 0 -> arm_down=0, arm_up=0
            (0.5, 0.0, 350.0),     # 0.5 -> arm_down=0, arm_up=350
            (1.0, 0.0, 700.0),     # 1 -> arm_down=0, arm_up=700
        ]
        
        all_passed = True
        for input_val, expected_down, expected_up in test_cases:
            result = self.send_test_command(
                f'大臂测试: boom={input_val}',
                {'boom': input_val}
            )
            
            if result:
                down_ok = self.verify_mapping(input_val, expected_down, result['arm_down_current'], '下降')
                up_ok = self.verify_mapping(input_val, expected_up, result['arm_up_current'], '抬升')
                all_passed = all_passed and down_ok and up_ok
            else:
                all_passed = False
            
            time.sleep(0.1)
        
        return all_passed
    
    def test_stick_mapping(self):
        """测试斗杆映射"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('测试斗杆 (stick) 映射')
        self.get_logger().info('='*60)
        
        test_cases = [
            (-1.0, 700.0, 0.0),    # -1 -> retract=700, extend=0
            (-0.5, 350.0, 0.0),    # -0.5 -> retract=350, extend=0
            (0.0, 0.0, 0.0),       # 0 -> retract=0, extend=0
            (0.5, 0.0, 350.0),     # 0.5 -> retract=0, extend=350
            (1.0, 0.0, 700.0),     # 1 -> retract=0, extend=700
        ]
        
        all_passed = True
        for input_val, expected_retract, expected_extend in test_cases:
            result = self.send_test_command(
                f'斗杆测试: stick={input_val}',
                {'stick': input_val}
            )
            
            if result:
                retract_ok = self.verify_mapping(input_val, expected_retract, result['stick_retract_current'], '收回')
                extend_ok = self.verify_mapping(input_val, expected_extend, result['stick_extend_current'], '伸出')
                all_passed = all_passed and retract_ok and extend_ok
            else:
                all_passed = False
            
            time.sleep(0.1)
        
        return all_passed
    
    def test_bucket_mapping(self):
        """测试铲斗映射"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('测试铲斗 (bucket) 映射')
        self.get_logger().info('='*60)
        
        test_cases = [
            (-1.0, 700.0, 0.0),    # -1 -> close=700, dump=0
            (-0.5, 350.0, 0.0),   # -0.5 -> close=350, dump=0
            (0.0, 0.0, 0.0),      # 0 -> close=0, dump=0
            (0.5, 0.0, 350.0),    # 0.5 -> close=0, dump=350
            (1.0, 0.0, 700.0),    # 1 -> close=0, dump=700
        ]
        
        all_passed = True
        for input_val, expected_close, expected_dump in test_cases:
            result = self.send_test_command(
                f'铲斗测试: bucket={input_val}',
                {'bucket': input_val}
            )
            
            if result:
                close_ok = self.verify_mapping(input_val, expected_close, result['bucket_close_current'], '收斗')
                dump_ok = self.verify_mapping(input_val, expected_dump, result['bucket_dump_current'], '翻斗')
                all_passed = all_passed and close_ok and dump_ok
            else:
                all_passed = False
            
            time.sleep(0.1)
        
        return all_passed
    
    def test_swing_mapping(self):
        """测试回转映射"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('测试回转 (swing) 映射')
        self.get_logger().info('='*60)
        
        test_cases = [
            (-1.0, 700.0, 0.0),    # -1 -> left=700, right=0
            (-0.5, 350.0, 0.0),    # -0.5 -> left=350, right=0
            (0.0, 0.0, 0.0),       # 0 -> left=0, right=0
            (0.5, 0.0, 350.0),     # 0.5 -> left=0, right=350
            (1.0, 0.0, 700.0),     # 1 -> left=0, right=700
        ]
        
        all_passed = True
        for input_val, expected_left, expected_right in test_cases:
            result = self.send_test_command(
                f'回转测试: swing={input_val}',
                {'swing': input_val}
            )
            
            if result:
                left_ok = self.verify_mapping(input_val, expected_left, result['rotate_left_current'], '左转')
                right_ok = self.verify_mapping(input_val, expected_right, result['rotate_right_current'], '右转')
                all_passed = all_passed and left_ok and right_ok
            else:
                all_passed = False
            
            time.sleep(0.1)
        
        return all_passed
    
    def test_track_mapping(self):
        """测试履带映射"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('测试履带 (leftTrack/rightTrack) 映射')
        self.get_logger().info('='*60)
        
        test_cases = [
            (-1.0, 700.0, 0.0),    # -1 -> backward=700, forward=0
            (-0.5, 350.0, 0.0),    # -0.5 -> backward=350, forward=0
            (0.0, 0.0, 0.0),       # 0 -> backward=0, forward=0
            (0.5, 0.0, 350.0),     # 0.5 -> backward=0, forward=350
            (1.0, 0.0, 700.0),     # 1 -> backward=0, forward=700
        ]
        
        all_passed = True
        
        # 测试左履带
        for input_val, expected_backward, expected_forward in test_cases:
            result = self.send_test_command(
                f'左履带测试: leftTrack={input_val}',
                {'leftTrack': input_val}
            )
            
            if result:
                backward_ok = self.verify_mapping(input_val, expected_backward, result['left_track_backward_current'], '左后退')
                forward_ok = self.verify_mapping(input_val, expected_forward, result['left_track_forward_current'], '左前进')
                all_passed = all_passed and backward_ok and forward_ok
            else:
                all_passed = False
            
            time.sleep(0.1)
        
        # 测试右履带
        for input_val, expected_backward, expected_forward in test_cases:
            result = self.send_test_command(
                f'右履带测试: rightTrack={input_val}',
                {'rightTrack': input_val}
            )
            
            if result:
                backward_ok = self.verify_mapping(input_val, expected_backward, result['right_track_backward_current'], '右后退')
                forward_ok = self.verify_mapping(input_val, expected_forward, result['right_track_forward_current'], '右前进')
                all_passed = all_passed and backward_ok and forward_ok
            else:
                all_passed = False
            
            time.sleep(0.1)
        
        return all_passed
    
    def test_combined_controls(self):
        """测试组合控制"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('测试组合控制')
        self.get_logger().info('='*60)
        
        result = self.send_test_command(
            '组合控制测试',
            {
                'boom': 0.5,
                'stick': -0.5,
                'bucket': 1.0,
                'swing': -1.0,
                'leftTrack': 0.5,
                'rightTrack': -0.5,
                'emergency_stop': False,
                'parking_brake': True
            }
        )
        
        if result:
            self.get_logger().info('组合控制输出:')
            self.get_logger().info(f'  大臂: 抬升={result["arm_up_current"]:.1f}mA, 下降={result["arm_down_current"]:.1f}mA')
            self.get_logger().info(f'  斗杆: 收回={result["stick_retract_current"]:.1f}mA, 伸出={result["stick_extend_current"]:.1f}mA')
            self.get_logger().info(f'  铲斗: 收斗={result["bucket_close_current"]:.1f}mA, 翻斗={result["bucket_dump_current"]:.1f}mA')
            self.get_logger().info(f'  回转: 左转={result["rotate_left_current"]:.1f}mA, 右转={result["rotate_right_current"]:.1f}mA')
            self.get_logger().info(f'  左履带: 前进={result["left_track_forward_current"]:.1f}mA, 后退={result["left_track_backward_current"]:.1f}mA')
            self.get_logger().info(f'  右履带: 前进={result["right_track_forward_current"]:.1f}mA, 后退={result["right_track_backward_current"]:.1f}mA')
            return True
        else:
            return False
    
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('开始测试 teleop_converter_xiaosong')
        self.get_logger().info('='*60)
        
        results = {}
        
        # 运行各项测试
        results['boom'] = self.test_boom_mapping()
        results['stick'] = self.test_stick_mapping()
        results['bucket'] = self.test_bucket_mapping()
        results['swing'] = self.test_swing_mapping()
        results['track'] = self.test_track_mapping()
        results['combined'] = self.test_combined_controls()
        
        # 打印测试总结
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('测试总结')
        self.get_logger().info('='*60)
        
        for test_name, passed in results.items():
            status = '✓ 通过' if passed else '✗ 失败'
            self.get_logger().info(f'{test_name}: {status}')
        
        total_passed = sum(1 for v in results.values() if v)
        total_tests = len(results)
        
        self.get_logger().info(f'\n总计: {total_passed}/{total_tests} 测试通过')
        
        if total_passed == total_tests:
            self.get_logger().info('🎉 所有测试通过！')
        else:
            self.get_logger().warn('⚠️  部分测试失败，请检查输出')
        
        return total_passed == total_tests


def main(args=None):
    rclpy.init(args=args)
    
    tester = XiaosongConverterTester()
    
    try:
        # 运行所有测试
        success = tester.run_all_tests()
        
        # 保持节点运行一段时间以接收所有消息
        time.sleep(1.0)
        
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        tester.get_logger().info('测试被用户中断')
    except Exception as e:
        tester.get_logger().error(f'测试出错: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

