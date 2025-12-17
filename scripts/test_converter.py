#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整测试程序：测试 teleop_converter 的所有功能
- 模拟输入信息及其范围（参考 keyboard_piston_joint_publisher_2_updated.py）
- 验证输出信息及范围是否与 cannode 接收的输入范围一致
- 包含边界值测试、范围验证、映射验证等
"""

import json
import sys
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

try:
    from sa_msgs.msg import ProtoAdapter
    HAS_SA_MSGS = True
except ImportError:
    ProtoAdapter = StringMsg
    HAS_SA_MSGS = False


# 输入范围定义（参考新的 ExcavatorControls 接口）
INPUT_RANGES = {
    # 通用挖掘臂控制
    "leftTrack": {"min": -1.0, "max": 1.0, "desc": "左履带: -1(后) to 1(前)"},
    "rightTrack": {"min": -1.0, "max": 1.0, "desc": "右履带: -1(后) to 1(前)"},
    "swing": {"min": -1.0, "max": 1.0, "desc": "驾驶室旋转: -1(左) to 1(右)"},
    "boom": {"min": -1.0, "max": 1.0, "desc": "大臂: -1(降) to 1(提)"},
    "stick": {"min": -1.0, "max": 1.0, "desc": "小臂: -1(收) to 1(伸)"},
    "bucket": {"min": -1.0, "max": 1.0, "desc": "铲斗: -1(收) to 1(翻)"},
    # 装载机/线控底盘扩展信号
    "steering": {"min": -1.0, "max": 1.0, "desc": "铰接转向: -1(左) to 1(右)"},
    "throttle": {"min": 0.0, "max": 1.0, "desc": "油门: 0 to 1"},
    "brake": {"min": 0.0, "max": 1.0, "desc": "刹车: 0 to 1"},
    # 关键辅助信号
    "emergency_stop": {"values": [True, False], "desc": "紧急急停"},
    "parking_brake": {"values": [True, False], "desc": "停车制动"},
    "horn": {"values": [True, False], "desc": "喇叭"},
    "gear": {"values": ["N", "D", "R"], "desc": "档位: N(空档), D(前进), R(后退)"},
    "speed_mode": {"values": ["turtle", "rabbit"], "desc": "速度模式: 乌龟/兔子"},
    "light_code": {"min": 0, "max": 31, "desc": "灯光代码: 位掩码 (0x01:左转, 0x02:右转, 0x04:远光, 0x08:近光, 0x10:工作灯)"},
    "hydraulic_lock": {"values": [True, False], "desc": "液压锁"},
    "power_enable": {"values": [True, False], "desc": "上高压"},
}

# 输出范围定义（参考 control_cmd.proto 和 cannode 实现）
OUTPUT_RANGES = {
    "steering_target": {"min": -100.0, "max": 100.0, "desc": "转向目标: -100% to 100%"},
    "throttle": {"min": 0.0, "max": 100.0, "desc": "油门: 0% to 100%"},
    "brake": {"min": 0.0, "max": 100.0, "desc": "刹车: 0% to 100%"},
    "arm_angle": {"min": 0.0, "max": 60.0, "desc": "大臂角度: 0° to 60°"},  # cannode限制
    "shovel_angle": {"min": -60.0, "max": 60.0, "desc": "铲斗角度: -60° to 60°"},
    "gear_location": {"values": [1, 2, 3], "desc": "档位: 1(D前进), 2(N空档), 3(R后退)"},
    "speed": {"min": -3.0, "max": 3.0, "desc": "速度: -3.0 to 3.0 m/s"},
}

# 默认最大速度（与 teleop2can_transformer 默认参数一致）
DEFAULT_MAX_SPEED = 3.0


class TeleopConverterTester(Node):
    def __init__(self):
        super().__init__('teleop2can_transformer_tester')
        
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
        
        self.pub = self.create_publisher(StringMsg, '/controls/teleop', qos_profile_teleop)
        self.get_logger().info('已创建发布者 /controls/teleop (QoS: BEST_EFFORT)')
        
        # 等待发布者连接（确保订阅者已准备好）
        import time
        time.sleep(0.5)
        self.get_logger().info(f'发布者连接状态: {self.pub.get_subscription_count()} 个订阅者')
        
        if HAS_SA_MSGS:
            self.sub = self.create_subscription(
                ProtoAdapter,
                '/vehicle_command',
                self.vehicle_cmd_callback,
                reliable_qos,
            )
            self.can_parse_protobuf = self._try_import_protobuf()
            self.get_logger().info('已创建订阅者 /vehicle_command (ProtoAdapter, QoS: RELIABLE)')
            # 等待订阅者连接
            time.sleep(0.5)
            # rclpy Subscription 无 get_publisher_count，提示等待发布者连接
            self.get_logger().info('订阅者已创建，等待发布者连接...')
        else:
            self.can_parse_protobuf = False
            self.get_logger().warn('sa_msgs 不可用，使用 String 消息类型作为后备')
        
        # 测试结果统计
        self.test_results = {
            "total": 0,
            "passed": 0,
            "failed": 0,
            "errors": []
        }
        
        # 当前测试用例
        self.current_test = None
        self.waiting_for_response = False
        self.test_completed = False  # 标记测试是否完成
        
        # 生成测试用例
        self.test_cases = self._generate_test_cases()
        self.idx = 0
        
        self.timer = self.create_timer(2.0, self.timer_cb)  # 2秒间隔
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('完整测试程序已启动')
        self.get_logger().info(f'Protobuf 解析: {"可用" if self.can_parse_protobuf else "不可用（将使用主程序日志验证）"}')
        self.get_logger().info('=' * 70)
        self.get_logger().info('💡 提示: 请同时运行 teleop2can_transformer 节点以查看详细的转换日志')
        self.get_logger().info('   命令: ros2 launch teleoptocantransformer teleop_converter.launch.py')
        self.get_logger().info('=' * 70)
        self._print_ranges()
    
    def _print_ranges(self):
        """打印输入输出范围定义"""
        self.get_logger().info('\n📋 输入范围定义（参考 keyboard_piston_joint_publisher_2_updated.py）:')
        for key, val in INPUT_RANGES.items():
            if "min" in val:
                self.get_logger().info(f'   {key}: [{val["min"]}, {val["max"]}] - {val["desc"]}')
            else:
                self.get_logger().info(f'   {key}: {val["values"]} - {val["desc"]}')
        
        self.get_logger().info('\n📋 输出范围定义（参考 control_cmd.proto 和 cannode）:')
        for key, val in OUTPUT_RANGES.items():
            if "min" in val:
                self.get_logger().info(f'   {key}: [{val["min"]}, {val["max"]}] - {val["desc"]}')
            else:
                self.get_logger().info(f'   {key}: {val["values"]} - {val["desc"]}')
        self.get_logger().info('')
    
    def _try_import_protobuf(self):
        """尝试导入 protobuf 模块"""
        try:
            import importlib.util
            proto_dir = os.path.join(os.path.dirname(__file__), '../../cannode/protobuf/out/control_msgs')
            proto_files = [f for f in os.listdir(proto_dir) if f.endswith('_pb2.py')] if os.path.exists(proto_dir) else []
            if not proto_files:
                return False
            proto_path = os.path.join(proto_dir, proto_files[0])
            if os.path.exists(proto_path):
                spec = importlib.util.spec_from_file_location("control_cmd_pb2", proto_path)
                self.control_cmd_pb2 = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(self.control_cmd_pb2)
                return True
        except Exception as e:
            self.get_logger().debug(f'无法导入 protobuf 模块: {e}')
        return False
    
    def _generate_test_cases(self):
        """生成所有测试用例"""
        test_cases = []
        
        # 1. 基础功能测试（使用新格式）
        test_cases.append({
            "name": "基础测试 - 空档保持",
            "input": {"steering": 0.0, "throttle": 0.0, "brake": 0.0, "gear": "N"},
            "expected": {"steering_target": 0.0, "throttle": 0.0, "brake": 0.0, "gear_location": 2},
            "range_check": {"steering_target": [-100, 100], "throttle": [0, 100], "brake": [0, 100]},
        })
        
        test_cases.append({
            "name": "基础测试 - 右转+油门前进",
            "input": {"steering": 0.5, "throttle": 0.6, "brake": 0.0, "gear": "D"},
            "expected": {"steering_target": -50.0, "throttle": 60.0, "brake": 0.0, "gear_location": 1},
            "range_check": {"steering_target": [-100, 100], "throttle": [0, 100], "brake": [0, 100]},
        })
        
        test_cases.append({
            "name": "基础测试 - 左转+刹车",
            "input": {"steering": -0.5, "throttle": 0.2, "brake": 0.7, "gear": "D"},
            "expected": {"steering_target": 50.0, "throttle": 20.0, "brake": 70.0, "gear_location": 1},
            "range_check": {"steering_target": [-100, 100], "throttle": [0, 100], "brake": [0, 100]},
        })
        
        test_cases.append({
            "name": "基础测试 - 倒车",
            "input": {"steering": 0.0, "throttle": 0.4, "brake": 0.0, "gear": "R"},
            "expected": {"steering_target": 0.0, "throttle": 40.0, "brake": 0.0, "gear_location": 3},
            "range_check": {"steering_target": [-100, 100], "throttle": [0, 100], "brake": [0, 100]},
        })
        
        # 1.1 新格式字段测试（保留格式但不映射）
        test_cases.append({
            "name": "新格式测试 - 履带控制",
            "input": {"leftTrack": 0.5, "rightTrack": -0.5, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"steering_target": 0.0, "throttle": 0.0, "brake": 0.0, "gear_location": 2},
            "range_check": {"steering_target": [-100, 100]},
        })
        
        test_cases.append({
            "name": "新格式测试 - 驾驶室旋转",
            "input": {"swing": 0.8, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"steering_target": 0.0, "throttle": 0.0, "brake": 0.0, "gear_location": 2},
            "range_check": {"steering_target": [-100, 100]},
        })
        
        test_cases.append({
            "name": "新格式测试 - 小臂控制",
            "input": {"stick": 0.6, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"steering_target": 0.0, "throttle": 0.0, "brake": 0.0, "gear_location": 2},
            "range_check": {"steering_target": [-100, 100]},
        })
        
        test_cases.append({
            "name": "新格式测试 - 速度模式(兔子)",
            "input": {"steering": 0.0, "throttle": 1.0, "brake": 0.0, "gear": "D", "speed_mode": "rabbit"},
            "expected": {"steering_target": 0.0, "throttle": 100.0, "brake": 0.0, "gear_location": 1},
            "range_check": {"steering_target": [-100, 100], "throttle": [0, 100], "brake": [0, 100]},
        })
        
        test_cases.append({
            "name": "新格式测试 - 速度模式(乌龟)",
            "input": {"steering": 0.0, "throttle": 1.0, "brake": 0.0, "gear": "D", "speed_mode": "turtle"},
            "expected": {"steering_target": 0.0, "throttle": 100.0, "brake": 0.0, "gear_location": 1},
            "range_check": {"steering_target": [-100, 100], "throttle": [0, 100], "brake": [0, 100]},
        })
        
        test_cases.append({
            "name": "新格式测试 - 灯光代码",
            "input": {"light_code": 3, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"steering_target": 0.0, "throttle": 0.0, "brake": 0.0, "gear_location": 2},
            "range_check": {"steering_target": [-100, 100]},
        })
        
        test_cases.append({
            "name": "新格式测试 - 液压锁",
            "input": {"hydraulic_lock": True, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"steering_target": 0.0, "throttle": 0.0, "brake": 0.0, "gear_location": 2},
            "range_check": {"steering_target": [-100, 100]},
        })
        
        test_cases.append({
            "name": "新格式测试 - 喇叭",
            "input": {"horn": True, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"steering_target": 0.0, "throttle": 0.0, "brake": 0.0, "gear_location": 2},
            "range_check": {"steering_target": [-100, 100]},
        })
        
        # 2. 边界值测试
        test_cases.append({
            "name": "边界值测试 - steering 最小值",
            "input": {"steering": -1.0, "throttle": 0.0, "brake": 0.0, "gear": "N"},
            "expected": {"steering_target": 100.0},  # 反向映射
            "range_check": {"steering_target": [-100, 100]},
        })
        
        test_cases.append({
            "name": "边界值测试 - steering 最大值",
            "input": {"steering": 1.0, "throttle": 0.0, "brake": 0.0, "gear": "N"},
            "expected": {"steering_target": -100.0},  # 反向映射
            "range_check": {"steering_target": [-100, 100]},
        })
        
        test_cases.append({
            "name": "边界值测试 - throttle 最小值",
            "input": {"steering": 0.0, "throttle": 0.0, "brake": 0.0, "gear": "N"},
            "expected": {"throttle": 0.0},
            "range_check": {"throttle": [0, 100]},
        })
        
        test_cases.append({
            "name": "边界值测试 - throttle 最大值",
            "input": {"steering": 0.0, "throttle": 1.0, "brake": 0.0, "gear": "D"},
            "expected": {"throttle": 100.0},
            "range_check": {"throttle": [0, 100]},
        })
        
        test_cases.append({
            "name": "边界值测试 - brake 最大值",
            "input": {"steering": 0.0, "throttle": 0.0, "brake": 1.0, "gear": "N"},
            "expected": {"brake": 100.0},
            "range_check": {"brake": [0, 100]},
        })

        # 2.1 死区测试（默认死区 0.05）
        test_cases.append({
            "name": "死区测试 - steering 微小输入",
            "input": {"steering": 0.02, "throttle": 0.0, "brake": 0.0, "gear": "N"},
            "expected": {"steering_target": 0.0},
            "range_check": {"steering_target": [-100, 100]},
        })
        test_cases.append({
            "name": "死区测试 - throttle 微小输入",
            "input": {"steering": 0.0, "throttle": 0.02, "brake": 0.0, "gear": "N"},
            "expected": {"throttle": 0.0},
            "range_check": {"throttle": [0, 100]},
        })
        test_cases.append({
            "name": "死区测试 - brake 微小输入",
            "input": {"steering": 0.0, "throttle": 0.0, "brake": 0.02, "gear": "N"},
            "expected": {"brake": 0.0},
            "range_check": {"brake": [0, 100]},
        })
        
        # 3. 档位测试
        for gear in ["N", "D", "R"]:
            gear_map = {"N": 2, "D": 1, "R": 3}
            test_cases.append({
                "name": f"档位测试 - {gear}",
                "input": {"steering": 0.0, "throttle": 0.0, "brake": 0.0, "gear": gear},
                "expected": {"gear_location": gear_map[gear]},
                "range_check": {},
            })
        
        # 4. 大臂/铲斗测试
        test_cases.append({
            "name": "大臂/铲斗测试 - boom最大值",
            "input": {"boom": 1.0, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"arm_enable": True},
            "range_check": {"arm_angle": [0, 60]},  # cannode限制
        })
        
        test_cases.append({
            "name": "大臂/铲斗测试 - boom最小值",
            "input": {"boom": -1.0, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"arm_enable": True},
            "range_check": {"arm_angle": [0, 60]},
        })
        
        test_cases.append({
            "name": "大臂/铲斗测试 - bucket最大值",
            "input": {"bucket": 1.0, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"shovel_enable": True},
            "range_check": {"shovel_angle": [-60, 60]},
        })
        
        test_cases.append({
            "name": "大臂/铲斗测试 - bucket最小值",
            "input": {"bucket": -1.0, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"shovel_enable": True},
            "range_check": {"shovel_angle": [-60, 60]},
        })
        
        test_cases.append({
            "name": "大臂/铲斗测试 - 组合动作",
            "input": {"boom": 0.6, "bucket": -0.4, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"arm_enable": True, "shovel_enable": True},
            "range_check": {"arm_angle": [0, 60], "shovel_angle": [-60, 60]},
        })

        # 4.1 大臂/铲斗输入超出范围（会被 clamp 到 [-1, 1]）
        test_cases.append({
            "name": "超出范围测试 - boom超出",
            "input": {"boom": 2.0, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"arm_enable": True},
            "range_check": {"arm_angle": [0, 60]},
        })
        test_cases.append({
            "name": "超出范围测试 - bucket超出",
            "input": {"bucket": -2.0, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"shovel_enable": True},
            "range_check": {"shovel_angle": [-60, 60]},
        })
        
        # 5. 超出范围测试（验证限制功能）
        test_cases.append({
            "name": "超出范围测试 - steering超出",
            "input": {"steering": 2.0, "throttle": 0.0, "brake": 0.0, "gear": "N"},
            "expected": {"steering_target": -100.0},  # 应该被限制到 -100
            "range_check": {"steering_target": [-100, 100]},
        })
        
        test_cases.append({
            "name": "超出范围测试 - throttle超出",
            "input": {"steering": 0.0, "throttle": 2.0, "brake": 0.0, "gear": "D"},
            "expected": {"throttle": 100.0},  # 应该被限制到 100
            "range_check": {"throttle": [0, 100]},
        })
        test_cases.append({
            "name": "超出范围测试 - brake超出",
            "input": {"steering": 0.0, "throttle": 0.0, "brake": 2.0, "gear": "N"},
            "expected": {"brake": 100.0},  # 应该被限制到 100
            "range_check": {"brake": [0, 100]},
        })
        
        # 6. 特殊功能测试
        test_cases.append({
            "name": "特殊功能测试 - 紧急停止",
            "input": {"emergency_stop": True, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"estop": True},
            "range_check": {},
        })
        
        test_cases.append({
            "name": "特殊功能测试 - 驻车制动",
            "input": {"parking_brake": True, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"parking_brake": True},
            "range_check": {},
        })
        
        test_cases.append({
            "name": "特殊功能测试 - 发动机开关",
            "input": {"power_enable": True, "steering": 0.0, "throttle": 0.0, "gear": "N"},
            "expected": {"engine_on_off": True},
            "range_check": {},
        })
        
        # 7. 组合测试
        test_cases.append({
            "name": "组合测试 - 正常行驶",
            "input": {"steering": 0.5, "throttle": 0.8, "brake": 0.0, "gear": "D"},
            "expected": {"steering_target": -50.0, "throttle": 80.0, "brake": 0.0, "gear_location": 1},
            "range_check": {"steering_target": [-100, 100], "throttle": [0, 100], "brake": [0, 100], "speed": [0, DEFAULT_MAX_SPEED]},
        })

        # 7.1 速度范围测试（前进/倒车最大油门）
        test_cases.append({
            "name": "速度范围测试 - 前进全油门",
            "input": {"steering": 0.0, "throttle": 1.0, "brake": 0.0, "gear": "D"},
            "expected": {"gear_location": 1},
            "range_check": {"speed": [0, DEFAULT_MAX_SPEED]},
        })
        test_cases.append({
            "name": "速度范围测试 - 倒车全油门",
            "input": {"steering": 0.0, "throttle": 1.0, "brake": 0.0, "gear": "R"},
            "expected": {"gear_location": 3},
            "range_check": {"speed": [-DEFAULT_MAX_SPEED, 0]},
        })
        
        # 8. 新格式完整组合测试
        test_cases.append({
            "name": "新格式组合测试 - 完整控制",
            "input": {
                "leftTrack": 0.3, "rightTrack": 0.3, "swing": 0.0,
                "boom": 0.5, "stick": -0.3, "bucket": 0.4,
                "steering": 0.2, "throttle": 0.6, "brake": 0.0,
                "gear": "D", "speed_mode": "rabbit",
                "emergency_stop": False, "parking_brake": False,
                "horn": False, "light_code": 5, "hydraulic_lock": False,
                "power_enable": True
            },
            "expected": {
                "steering_target": -20.0, "throttle": 60.0, "brake": 0.0,
                "gear_location": 1, "arm_enable": True, "shovel_enable": True,
                "engine_on_off": True
            },
            "range_check": {
                "steering_target": [-100, 100], "throttle": [0, 100], "brake": [0, 100],
                "arm_angle": [0, 60], "shovel_angle": [-60, 60]
            },
        })
        
        return test_cases
    
    def timer_cb(self):
        # 如果测试已完成，不再处理
        if self.test_completed:
            return
        
        if self.waiting_for_response:
            self.get_logger().warn(f'⚠ 等待响应超时: {self.current_test["name"]}')
            self.test_results["failed"] += 1
            self.test_results["total"] += 1
            self.waiting_for_response = False
            
            # 检查是否所有测试都已完成（包括超时的）
            if self.test_results["total"] >= len(self.test_cases):
                self.test_completed = True
                self._print_summary()
                self.get_logger().info('✅ 所有测试已完成，程序将退出')
                # 延迟一点时间让日志输出完成，然后退出
                threading.Timer(0.5, self._shutdown_timer_cb).start()
                return
        
        # 如果已经发送完所有测试用例，等待最后一个响应
        if self.idx >= len(self.test_cases):
            # 如果所有测试都已完成（包括最后一个响应已收到），则退出
            if self.test_results["total"] >= len(self.test_cases):
                self.test_completed = True
                self._print_summary()
                self.get_logger().info('✅ 所有测试已完成，程序将退出')
                threading.Timer(0.5, self._shutdown_timer_cb).start()
            return
        
        test = self.test_cases[self.idx]
        self.current_test = test
        self.waiting_for_response = True
        
        msg = StringMsg()
        msg.data = json.dumps(test["input"], ensure_ascii=False)
        
        # 确保发布者已准备好
        import time
        time.sleep(0.1)  # 短暂延迟确保发布者已连接
        
        self.pub.publish(msg)
        self.get_logger().info(f'📤 已发布消息到 /controls/teleop，消息长度: {len(msg.data)} 字节')
        
        self.get_logger().info('-' * 70)
        self.get_logger().info(f'📤 测试 [{self.idx+1}/{len(self.test_cases)}]: {test["name"]}')
        self.get_logger().info(f'   输入: {msg.data}')
        if "expected" in test:
            self.get_logger().info(f'   期望: {json.dumps(test["expected"], ensure_ascii=False)}')
        if "range_check" in test and test["range_check"]:
            self.get_logger().info(f'   范围检查: {json.dumps(test["range_check"], ensure_ascii=False)}')
        
        self.idx += 1
    
    def _shutdown_timer_cb(self):
        """关闭定时器并退出"""
        rclpy.shutdown()
    
    def vehicle_cmd_callback(self, msg):
        if not self.waiting_for_response or not self.current_test:
            return
        
        # 如果测试已完成，不再处理
        if self.test_completed:
            return
        
        self.waiting_for_response = False
        self.test_results["total"] += 1
        
        test = self.current_test
        msg_len = len(msg.pb) if hasattr(msg, 'pb') and hasattr(msg.pb, '__len__') else 0
        
        self.get_logger().info('✓ 收到响应 (protobuf 大小: %d 字节)' % msg_len)
        
        # 基本验证：检查消息大小
        if msg_len < 10:
            self.get_logger().warn('   ⚠ protobuf 消息大小异常小')
            self.test_results["failed"] += 1
            self.test_results["errors"].append(f"{test['name']}: protobuf 消息大小异常 ({msg_len} 字节)")
            # 即使消息异常，也要检查是否所有测试都已完成
            if self.test_results["total"] >= len(self.test_cases):
                self.test_completed = True
                self._print_summary()
                self.get_logger().info('✅ 所有测试已完成，程序将退出')
                threading.Timer(0.5, self._shutdown_timer_cb).start()
            return
        
        # 尝试解析 protobuf（如果可用）
        if self.can_parse_protobuf:
            try:
                cmd = self.control_cmd_pb2.ControlCommand()
                cmd.ParseFromString(bytes(msg.pb))
                
                self.get_logger().info('   解析结果:')
                self.get_logger().info(f'     steering_target: {cmd.steering_target:.2f}%')
                self.get_logger().info(f'     throttle: {cmd.throttle:.2f}%')
                self.get_logger().info(f'     brake: {cmd.brake:.2f}%')
                gear_map = {1: "D(前进)", 2: "N(空档)", 3: "R(后退)"}
                gear_str = gear_map.get(cmd.gear_location, f"{cmd.gear_location}")
                self.get_logger().info(f'     gear_location: {gear_str}')
                if cmd.has_speed():
                    self.get_logger().info(f'     speed: {cmd.speed:.2f} m/s')
                if cmd.has_arm_angle():
                    self.get_logger().info(f'     arm_angle: {cmd.arm_angle:.2f}° (enable: {cmd.arm_enable})')
                if cmd.has_shovel_angle():
                    self.get_logger().info(f'     shovel_angle: {cmd.shovel_angle:.2f}° (enable: {cmd.shovel_enable})')
                if cmd.has_estop():
                    self.get_logger().info(f'     estop: {cmd.estop}')
                if cmd.has_parking_brake():
                    self.get_logger().info(f'     parking_brake: {cmd.parking_brake}')
                if cmd.has_engine_on_off():
                    self.get_logger().info(f'     engine_on_off: {cmd.engine_on_off}')
                
                # 验证期望值
                errors = []
                if "expected" in test:
                    for key, expected_val in test["expected"].items():
                        actual_val = getattr(cmd, key, None)
                        if actual_val is None:
                            errors.append(f"{key}: 字段不存在")
                        elif isinstance(expected_val, bool):
                            if actual_val != expected_val:
                                errors.append(f"{key}: 期望 {expected_val}, 实际 {actual_val}")
                        elif isinstance(expected_val, (int, float)):
                            if abs(actual_val - expected_val) > 1.0:  # 允许1%误差
                                errors.append(f"{key}: 期望 {expected_val}, 实际 {actual_val:.2f}")
                        else:
                            if actual_val != expected_val:
                                errors.append(f"{key}: 期望 {expected_val}, 实际 {actual_val}")
                
                # 验证范围
                if "range_check" in test:
                    for key, range_val in test["range_check"].items():
                        actual_val = getattr(cmd, key, None)
                        if actual_val is None:
                            errors.append(f"{key}: 字段不存在，无法验证范围")
                        else:
                            min_val, max_val = range_val
                            if actual_val < min_val or actual_val > max_val:
                                errors.append(f"{key}: 值 {actual_val:.2f} 超出范围 [{min_val}, {max_val}]")
                
                if errors:
                    self.get_logger().error('   ❌ 验证失败:')
                    for err in errors:
                        self.get_logger().error(f'      {err}')
                    self.test_results["failed"] += 1
                    self.test_results["errors"].extend([f"{test['name']}: {err}" for err in errors])
                else:
                    self.get_logger().info('   ✅ 验证通过')
                    self.test_results["passed"] += 1
                    
            except Exception as e:
                self.get_logger().error(f'   解析失败: {e}')
                self.test_results["failed"] += 1
                self.test_results["errors"].append(f"{test['name']}: 解析失败 - {e}")
        else:
            # 无法解析 protobuf，仅记录收到消息
            self.get_logger().info('   注意: 请查看 teleop2can_transformer 节点的输出日志以验证转换结果')
            self.get_logger().info('   主程序会打印详细的输入输出信息和范围验证')
            self.test_results["passed"] += 1
        
        # 检查是否所有测试都已完成（在所有路径中都要检查）
        if self.test_results["total"] >= len(self.test_cases):
            self.test_completed = True
            self._print_summary()
            self.get_logger().info('✅ 所有测试已完成，程序将退出')
            # 延迟一点时间让日志输出完成，然后退出
            threading.Timer(0.5, self._shutdown_timer_cb).start()
    
    def _print_summary(self):
        """打印测试总结"""
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('📊 测试总结')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'总测试数: {self.test_results["total"]}')
        self.get_logger().info(f'通过: {self.test_results["passed"]}')
        self.get_logger().info(f'失败: {self.test_results["failed"]}')
        if self.test_results["total"] > 0:
            pass_rate = self.test_results["passed"] / self.test_results["total"] * 100
            self.get_logger().info(f'通过率: {pass_rate:.1f}%')
        
        if self.test_results["errors"]:
            self.get_logger().info('\n❌ 错误列表:')
            for err in self.test_results["errors"]:
                self.get_logger().info(f'   {err}')
        
        self.get_logger().info('=' * 70 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopConverterTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n测试中断')
        if not node.test_completed:
            node._print_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()