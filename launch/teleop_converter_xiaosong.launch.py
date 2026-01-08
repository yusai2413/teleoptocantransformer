from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 默认参数文件（使用已安装的包路径）
    pkg_share = get_package_share_directory('teleoptocantransformer')
    default_params = os.path.join(pkg_share, 'config', 'output_topics.yaml')

    return LaunchDescription([
        Node(
            package='teleoptocantransformer',
            executable='teleop2can_transformer_xiaosong',
            name='teleop2can_transformer_xiaosong',
            output='screen',
            parameters=[
                {
                    # 死区参数（用于过滤小的输入抖动）
                    'arm_deadzone': 0.05,      # 大臂死区
                    'stick_deadzone': 0.05,     # 斗杆死区
                    'bucket_deadzone': 0.05,    # 铲斗死区
                    'swing_deadzone': 0.05,     # 回转死区
                    'track_deadzone': 0.05,     # 履带死区
                    
                    # 电流映射范围（mA）
                    # xiaosong 协议：所有控制都是直接电流控制，范围 0~700mA
                    'max_current': 700.0,       # 最大电流 (mA)
                },
                # 额外参数文件（含 topic 发布开关等）
                default_params
            ]
        )
    ])

