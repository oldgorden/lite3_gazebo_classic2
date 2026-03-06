#!/bin/bash

# Gazebo 仿真启动脚本
# 功能：清理残留进程并启动 Gazebo 仿真环境

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 获取脚本所在目录的父目录（即工作空间根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 【修正点】这里需要向上跳两级：lite3_gazebo_classic -> src -> quadruped_ws
WORKSPACE_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Gazebo 仿真启动脚本${NC}"
echo -e "${GREEN}  工作空间：${WORKSPACE_ROOT}${NC}"
echo -e "${GREEN}========================================${NC}"

# 步骤 1: 清理残留的 Gazebo 进程
echo -e "${YELLOW}[1/5] 清理残留的 Gazebo 进程...${NC}"
pkill -9 -x gzserver 2>/dev/null || true
pkill -9 -x gzclient 2>/dev/null || true
pkill -9 -x gazebo 2>/dev/null || true
ps aux | grep -E "gazebo|gzserver|gzclient" | grep -v grep | grep -v run_gazebo.sh | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
sleep 1

REMAINING=$(ps aux | grep -E "(gazebo|gzserver|gzclient)" | grep -v grep | wc -l)
if [ "$REMAINING" -gt 0 ]; then
    echo -e "${RED}警告：仍有 $REMAINING 个 Gazebo 相关进程在运行${NC}"
    ps aux | grep -E "(gazebo|gzserver|gzclient)" | grep -v grep
    echo -e "${YELLOW}尝试使用 fuser 清理端口...${NC}"
    fuser -k 11345/tcp 2>/dev/null || true
    sleep 1
fi

# 步骤 2: 清理 Gazebo 共享内存
echo -e "${YELLOW}[2/5] 清理 Gazebo 共享内存...${NC}"
rm -f /tmp/gazebo_* 2>/dev/null || true

# 步骤 3: 设置环境变量
echo -e "${YELLOW}[3/5] 设置环境变量...${NC}"

if [ -f "/usr/share/gazebo/setup.sh" ]; then
    source /usr/share/gazebo/setup.sh
    echo -e "${GREEN}  ✓ 已加载 Gazebo 环境${NC}"
else
    echo -e "${YELLOW}  ! /usr/share/gazebo/setup.sh 不存在，跳过${NC}"
fi

LIVOX_PLUGIN_PATH="${WORKSPACE_ROOT}/install/ros2_livox_simulation/lib"
if [ -d "$LIVOX_PLUGIN_PATH" ]; then
    export GAZEBO_PLUGIN_PATH="${LIVOX_PLUGIN_PATH}:${GAZEBO_PLUGIN_PATH:-}"
    echo -e "${GREEN}  ✓ GAZEBO_PLUGIN_PATH: ${GAZEBO_PLUGIN_PATH}${NC}"
else
    echo -e "${YELLOW}  ! 目录不存在：${LIVOX_PLUGIN_PATH}${NC}"
    echo -e "${YELLOW}    请确保已编译工作空间：colcon build${NC}"
fi

# 步骤 4: 设置其他必要的环境变量
echo -e "${YELLOW}[4/5] 设置其他环境变量...${NC}"
export QT_QPA_PLATFORM=xcb
export GTK_IM_MODULE=
export QT_IM_MODULE=
export XMODIFIERS=
echo -e "${GREEN}  ✓ 已设置 GUI 相关环境变量${NC}"

# 步骤 5: 启动 Gazebo
echo -e "${YELLOW}[5/5] 启动 Gazebo 仿真...${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  启动命令：ros2 launch lite3_description gazebo_classic.launch.py${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 【重要】修正了路径后，这里就能正确找到 setup.bash 了
cd "$WORKSPACE_ROOT"
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py
