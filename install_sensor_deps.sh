#!/bin/bash
# Lite3 Gazebo 传感器依赖安装脚本
# 安装Livox Mid-360雷达和Realsense D435i相机所需的插件

set -e

echo "=== Lite3 Gazebo 传感器依赖安装 ==="
echo "工作空间: ${QUADRUPED_WS:-$HOME/quadruped_ws}"

WORKSPACE="${QUADRUPED_WS:-$HOME/quadruped_ws}"
SRC_DIR="$WORKSPACE/src"

if [ ! -d "$SRC_DIR" ]; then
    echo "错误: 工作空间目录不存在: $SRC_DIR"
    echo "请先创建工作空间: mkdir -p $SRC_DIR"
    exit 1
fi

cd "$SRC_DIR"

# 1. 安装Livox激光雷达仿真插件
echo "1. 安装Livox激光雷达仿真插件..."
if [ ! -d "livox_laser_simulation_ros2" ]; then
    echo "  克隆 livox_laser_simulation_ros2..."
    git clone https://github.com/LihanChen2004/livox_laser_simulation_ros2.git
    echo "  ✓ 已克隆"
else
    echo "  ✓ 已存在，跳过"
fi

# 2. 安装Realsense D435i相机插件
echo "2. 安装Realsense D435i相机插件..."
if [ ! -d "realsense_gazebo_plugin" ]; then
    echo "  克隆 realsense_gazebo_plugin..."
    git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
    echo "  ✓ 已克隆"
else
    echo "  ✓ 已存在，跳过"
fi

# 3. 安装Livox ROS2驱动程序
echo "3. 安装Livox ROS2驱动程序..."
if [ ! -d "livox_ros_driver2" ]; then
    echo "  克隆 livox_ros_driver2..."
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    echo "  ✓ 已克隆"
else
    echo "  ✓ 已存在，跳过"
fi

# 4. 编译所有包
echo "4. 编译所有包..."
cd "$WORKSPACE"
if command -v colcon &> /dev/null; then
    echo "  运行 colcon build..."
    colcon build --symlink-install
    echo "  ✓ 编译完成"
else
    echo "  ⚠  colcon未找到，请手动编译"
    echo "  运行: cd $WORKSPACE && colcon build --symlink-install"
fi

echo ""
echo "=== 安装完成 ==="
echo "1. 设置环境: source $WORKSPACE/install/setup.bash"
echo "2. 启动仿真: ros2 launch lite3_description gazebo_classic.launch.py"
echo "3. 验证传感器话题: ros2 topic list | grep -E 'mid360|camera'"
echo ""
echo "详细文档: $SRC_DIR/lite3_gazebo_classic/docs/gazebo_livox_integration.md"
