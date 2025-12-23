#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
挖掘机控制可视化界面测试脚本
用于测试 teleop_visualizer.py 的UI显示功能
支持自动演示和手动控制两种模式
"""

import sys
import os
import json
import time
import math
import threading

# 自动设置 ROS2 环境
def setup_ros2_environment():
    """自动设置 ROS2 环境路径"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = os.path.abspath(os.path.join(script_dir, '../../..'))
    install_dir = os.path.join(ws_dir, 'install')
    
    if os.path.exists(install_dir):
        for pkg_dir in os.listdir(install_dir):
            pkg_path = os.path.join(install_dir, pkg_dir, 'local', 'lib', 'python3.10', 'dist-packages')
            if os.path.exists(pkg_path) and pkg_path not in sys.path:
                sys.path.insert(0, pkg_path)
    
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    ros_python_path = f'/opt/ros/{ros_distro}/lib/python3.10/site-packages'
    if os.path.exists(ros_python_path) and ros_python_path not in sys.path:
        sys.path.insert(0, ros_python_path)
    
    ros_local_path = f'/opt/ros/{ros_distro}/local/lib/python3.10/dist-packages'
    if os.path.exists(ros_local_path) and ros_local_path not in sys.path:
        sys.path.insert(0, ros_local_path)

setup_ros2_environment()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class VisualizerTester(Node):
    def __init__(self):
        super().__init__('visualizer_tester')
        
        # 创建发布者
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.pub = self.create_publisher(StringMsg, '/controls/teleop', qos_profile)
        self.get_logger().info('✅ 已创建发布者 /controls/teleop')
        
        # 等待发布者就绪
        time.sleep(0.5)
        self.get_logger().info(f'📡 发布者连接状态: {self.pub.get_subscription_count()} 个订阅者')
        
        # 当前控制值
        self.controls = {
            'leftTrack': 0.0,
            'rightTrack': 0.0,
            'swing': 0.0,
            'boom': 0.0,
            'stick': 0.0,
            'bucket': 0.0,
            'steering': 0.0,
            'throttle': 0.0,
            'brake': 0.0,
            'gear': 'N',
            'speed_mode': 'turtle',
            'emergency_stop': False,
            'parking_brake': False,
            'power_enable': True,
            'device_type': 'excavator'
        }
        
        self.running = True
    
    def publish_controls(self):
        """发布当前控制值"""
        msg = StringMsg()
        msg.data = json.dumps(self.controls)
        self.pub.publish(msg)
        self.get_logger().info(f'📤 已发布: {json.dumps(self.controls, indent=2, ensure_ascii=False)}')
    
    def reset_controls(self):
        """重置所有控制值为默认值"""
        self.controls = {
            'leftTrack': 0.0,
            'rightTrack': 0.0,
            'swing': 0.0,
            'boom': 0.0,
            'stick': 0.0,
            'bucket': 0.0,
            'steering': 0.0,
            'throttle': 0.0,
            'brake': 0.0,
            'gear': 'N',
            'speed_mode': 'turtle',
            'emergency_stop': False,
            'parking_brake': False,
            'power_enable': True,
            'device_type': 'excavator'
        }
        self.publish_controls()
    
    def demo_forward_backward(self):
        """演示前进后退"""
        print("\n🚜 演示: 前进后退")
        # 先回到原位置
        self.reset_controls()
        time.sleep(0.5)
        
        for i in range(3):
            # 前进
            self.controls['leftTrack'] = 0.5
            self.controls['rightTrack'] = 0.5
            self.controls['gear'] = 'D'
            self.controls['throttle'] = 0.6
            self.publish_controls()
            time.sleep(1.0)
            
            # 停止
            self.controls['leftTrack'] = 0.0
            self.controls['rightTrack'] = 0.0
            self.controls['throttle'] = 0.0
            self.publish_controls()
            time.sleep(0.5)
            
            # 后退
            self.controls['leftTrack'] = -0.5
            self.controls['rightTrack'] = -0.5
            self.controls['gear'] = 'R'
            self.controls['throttle'] = 0.6
            self.publish_controls()
            time.sleep(1.0)
            
            # 停止
            self.reset_controls()
            time.sleep(0.5)
        
        # 最后回到原位置
        self.reset_controls()
        time.sleep(0.5)
    
    def demo_swing(self):
        """演示回转"""
        print("\n🔄 演示: 车身回转")
        # 先回到原位置
        self.reset_controls()
        time.sleep(0.5)
        
        for i in range(3):
            # 左转
            self.controls['swing'] = -0.8
            self.publish_controls()
            time.sleep(1.0)
            
            # 右转
            self.controls['swing'] = 0.8
            self.publish_controls()
            time.sleep(1.0)
            
            # 回中
            self.controls['swing'] = 0.0
            self.publish_controls()
            time.sleep(0.5)
        
        # 最后回到原位置
        self.reset_controls()
        time.sleep(0.5)
    
    def demo_boom(self):
        """演示大臂"""
        print("\n📏 演示: 大臂升降")
        # 先回到原位置
        self.reset_controls()
        time.sleep(0.5)
        
        for i in range(3):
            # 提升
            self.controls['boom'] = 0.8
            self.publish_controls()
            time.sleep(1.0)
            
            # 下降
            self.controls['boom'] = -0.8
            self.publish_controls()
            time.sleep(1.0)
            
            # 回中
            self.controls['boom'] = 0.0
            self.publish_controls()
            time.sleep(0.5)
        
        # 最后回到原位置
        self.reset_controls()
        time.sleep(0.5)
    
    def demo_stick(self):
        """演示斗杆"""
        print("\n🔧 演示: 斗杆伸缩")
        # 先回到原位置
        self.reset_controls()
        time.sleep(0.5)
        
        for i in range(3):
            # 伸出
            self.controls['stick'] = 0.8
            self.publish_controls()
            time.sleep(1.0)
            
            # 收回
            self.controls['stick'] = -0.8
            self.publish_controls()
            time.sleep(1.0)
            
            # 回中
            self.controls['stick'] = 0.0
            self.publish_controls()
            time.sleep(0.5)
        
        # 最后回到原位置
        self.reset_controls()
        time.sleep(0.5)
    
    def demo_bucket(self):
        """演示铲斗"""
        print("\n🪣 演示: 铲斗翻转")
        # 先回到原位置
        self.reset_controls()
        time.sleep(0.5)
        
        for i in range(3):
            # 翻转
            self.controls['bucket'] = 0.8
            self.publish_controls()
            time.sleep(1.0)
            
            # 收回
            self.controls['bucket'] = -0.8
            self.publish_controls()
            time.sleep(1.0)
            
            # 回中
            self.controls['bucket'] = 0.0
            self.publish_controls()
            time.sleep(0.5)
        
        # 最后回到原位置
        self.reset_controls()
        time.sleep(0.5)
    
    def demo_combined(self):
        """演示组合动作"""
        print("\n🎭 演示: 组合动作")
        # 先回到原位置
        self.reset_controls()
        time.sleep(0.5)
        
        # 动作1: 前进 + 回转 + 大臂提升
        print("  动作1: 前进 + 回转 + 大臂提升")
        self.controls['leftTrack'] = 0.3
        self.controls['rightTrack'] = 0.3
        self.controls['gear'] = 'D'
        self.controls['throttle'] = 0.5
        self.controls['swing'] = 0.5
        self.controls['boom'] = 0.6
        self.publish_controls()
        time.sleep(2.0)
        
        # 动作2: 大臂下降 + 斗杆伸出 + 铲斗翻转
        print("  动作2: 大臂下降 + 斗杆伸出 + 铲斗翻转")
        self.controls['leftTrack'] = 0.0
        self.controls['rightTrack'] = 0.0
        self.controls['throttle'] = 0.0
        self.controls['boom'] = -0.5
        self.controls['stick'] = 0.7
        self.controls['bucket'] = 0.8
        self.publish_controls()
        time.sleep(2.0)
        
        # 动作3: 回转 + 收回
        print("  动作3: 回转 + 收回")
        self.controls['swing'] = -0.6
        self.controls['boom'] = 0.3
        self.controls['stick'] = -0.5
        self.controls['bucket'] = -0.3
        self.publish_controls()
        time.sleep(2.0)
        
        # 最后回到原位置
        self.reset_controls()
        time.sleep(0.5)
    
    def demo_smooth_animation(self):
        """演示平滑动画（正弦波）"""
        print("\n🌊 演示: 平滑动画（正弦波）")
        # 先回到原位置
        self.reset_controls()
        time.sleep(0.5)
        
        duration = 10.0  # 10秒
        steps = 100
        dt = duration / steps
        
        for i in range(steps):
            t = i * dt
            # 使用正弦波创建平滑动画
            self.controls['swing'] = math.sin(t * 2 * math.pi / 5) * 0.8
            self.controls['boom'] = math.sin(t * 2 * math.pi / 4) * 0.7
            self.controls['stick'] = math.sin(t * 2 * math.pi / 3) * 0.6
            self.controls['bucket'] = math.sin(t * 2 * math.pi / 2.5) * 0.8
            self.controls['leftTrack'] = math.sin(t * 2 * math.pi / 6) * 0.5
            self.controls['rightTrack'] = math.sin(t * 2 * math.pi / 6 + math.pi/4) * 0.5
            self.publish_controls()
            time.sleep(dt)
        
        # 最后回到原位置
        self.reset_controls()
        time.sleep(0.5)
    
    def demo_turn(self):
        """演示差速转向"""
        print("\n🔄 演示: 差速转向")
        # 先回到原位置
        self.reset_controls()
        time.sleep(0.5)
        
        for i in range(3):
            # 左转（左履带慢，右履带快）
            self.controls['leftTrack'] = 0.2
            self.controls['rightTrack'] = 0.8
            self.publish_controls()
            time.sleep(1.5)
            
            # 右转（左履带快，右履带慢）
            self.controls['leftTrack'] = 0.8
            self.controls['rightTrack'] = 0.2
            self.publish_controls()
            time.sleep(1.5)
            
            # 停止
            self.reset_controls()
            time.sleep(0.5)
        
        # 最后回到原位置
        self.reset_controls()
        time.sleep(0.5)
    
    def demo_turn_backward(self):
        """演示左右转后退"""
        print("\n🔄⬅️ 演示: 左右转后退")
        # 先回到原位置
        self.reset_controls()
        time.sleep(0.5)
        
        for i in range(3):
            # 左转后退（左履带慢后退，右履带快后退）
            print("  左转后退")
            self.controls['leftTrack'] = -0.3
            self.controls['rightTrack'] = -0.8
            self.controls['gear'] = 'R'
            self.controls['throttle'] = 0.6
            self.publish_controls()
            time.sleep(1.5)
            
            # 停止
            self.reset_controls()
            time.sleep(0.5)
            
            # 右转后退（左履带快后退，右履带慢后退）
            print("  右转后退")
            self.controls['leftTrack'] = -0.8
            self.controls['rightTrack'] = -0.3
            self.controls['gear'] = 'R'
            self.controls['throttle'] = 0.6
            self.publish_controls()
            time.sleep(1.5)
            
            # 停止
            self.reset_controls()
            time.sleep(0.5)
        
        # 最后回到原位置
        self.reset_controls()
        time.sleep(0.5)
    
    def run_auto_demo(self):
        """运行自动演示"""
        print("\n" + "="*60)
        print("🎬 开始自动演示模式")
        print("="*60)
        
        demos = [
            ("前进后退", self.demo_forward_backward),
            ("差速转向", self.demo_turn),
            ("左右转后退", self.demo_turn_backward),
            ("车身回转", self.demo_swing),
            ("大臂升降", self.demo_boom),
            ("斗杆伸缩", self.demo_stick),
            ("铲斗翻转", self.demo_bucket),
            ("组合动作", self.demo_combined),
            ("平滑动画", self.demo_smooth_animation),
        ]
        
        for name, demo_func in demos:
            print(f"\n▶️  执行演示: {name}")
            demo_func()
            time.sleep(1.0)
        
        print("\n✅ 自动演示完成")
        self.reset_controls()
    
    def interactive_mode(self):
        """交互式控制模式"""
        print("\n" + "="*60)
        print("🎮 交互式控制模式")
        print("="*60)
        print("输入控制值（输入 'help' 查看帮助，输入 'quit' 退出）")
        print("格式: 字段名=值，例如: boom=0.5, leftTrack=-0.3")
        print("支持的命令:")
        print("  reset - 重置所有控制值")
        print("  demo - 运行自动演示")
        print("  quit - 退出")
        print("-"*60)
        
        while self.running:
            try:
                cmd = input("\n> ").strip()
                
                if cmd.lower() == 'quit' or cmd.lower() == 'q':
                    break
                elif cmd.lower() == 'reset' or cmd.lower() == 'r':
                    self.reset_controls()
                    continue
                elif cmd.lower() == 'demo' or cmd.lower() == 'd':
                    self.run_auto_demo()
                    continue
                elif cmd.lower() == 'help' or cmd.lower() == 'h':
                    print("\n可用字段:")
                    print("  leftTrack, rightTrack: -1.0 到 1.0")
                    print("  swing, boom, stick, bucket: -1.0 到 1.0")
                    print("  steering, throttle, brake: -1.0 到 1.0")
                    print("  gear: N, D, R")
                    print("  speed_mode: turtle, rabbit")
                    print("  emergency_stop, parking_brake, power_enable: true, false")
                    continue
                elif not cmd:
                    self.publish_controls()
                    continue
                
                # 解析命令
                parts = cmd.split(',')
                for part in parts:
                    part = part.strip()
                    if '=' in part:
                        key, value = part.split('=', 1)
                        key = key.strip()
                        value = value.strip()
                        
                        # 更新控制值
                        if key in self.controls:
                            if isinstance(self.controls[key], bool):
                                self.controls[key] = value.lower() in ['true', '1', 'yes', 'on']
                            elif isinstance(self.controls[key], str):
                                self.controls[key] = value
                            else:
                                try:
                                    self.controls[key] = float(value)
                                    # 限制范围
                                    if key in ['leftTrack', 'rightTrack', 'swing', 'boom', 'stick', 'bucket', 'steering']:
                                        self.controls[key] = max(-1.0, min(1.0, self.controls[key]))
                                    elif key in ['throttle', 'brake']:
                                        self.controls[key] = max(0.0, min(1.0, self.controls[key]))
                                except ValueError:
                                    print(f"❌ 无效的数值: {value}")
                                    continue
                        else:
                            print(f"❌ 未知字段: {key}")
                            continue
                
                self.publish_controls()
                
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        print("\n👋 退出交互式模式")
        self.reset_controls()


def main(args=None):
    rclpy.init(args=args)
    tester = VisualizerTester()
    
    print("\n" + "="*60)
    print("🧪 挖掘机控制可视化界面测试工具")
    print("="*60)
    print("\n请选择模式:")
    print("  1. 自动演示模式 (auto)")
    print("  2. 交互式控制模式 (interactive)")
    print("\n或者直接运行: python test_visualizer.py auto|interactive")
    
    mode = 'interactive'
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
    else:
        try:
            user_input = input("\n请输入模式 (1/2 或 auto/interactive): ").strip()
            if user_input in ['1', 'auto', 'a']:
                mode = 'auto'
            elif user_input in ['2', 'interactive', 'i']:
                mode = 'interactive'
        except (KeyboardInterrupt, EOFError):
            print("\n👋 退出")
            rclpy.shutdown()
            return
    
    try:
        if mode == 'auto':
            # 在单独线程中运行 ROS2
            def spin_ros():
                try:
                    while tester.running:
                        rclpy.spin_once(tester, timeout_sec=0.1)
                except:
                    pass
            
            ros_thread = threading.Thread(target=spin_ros, daemon=True)
            ros_thread.start()
            
            # 运行演示
            tester.run_auto_demo()
            
        else:
            # 交互式模式
            def spin_ros():
                try:
                    while tester.running:
                        rclpy.spin_once(tester, timeout_sec=0.1)
                except:
                    pass
            
            ros_thread = threading.Thread(target=spin_ros, daemon=True)
            ros_thread.start()
            
            tester.interactive_mode()
    
    except KeyboardInterrupt:
        print("\n\n⚠️  收到中断信号")
    finally:
        tester.running = False
        tester.reset_controls()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

