#!/bin/bash
# 启动挖掘机控制可视化界面

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

# Source ROS2 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
else
    echo "警告: 未找到 ROS2 安装，尝试使用默认路径"
    source /opt/ros/*/setup.bash 2>/dev/null || true
fi

# Source 工作空间
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "已加载工作空间: $WS_DIR"
else
    echo "错误: 未找到工作空间安装文件: $WS_DIR/install/setup.bash"
    echo "请先运行: colcon build"
    exit 1
fi

# 运行可视化脚本
cd "$SCRIPT_DIR"
python3 teleop_visualizer.py "$@"

