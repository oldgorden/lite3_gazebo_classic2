# Lite3 Gazebo Classic 仿真

Lite3 四足机器人的 Gazebo Classic 仿真环境，支持 ROS2 Humble。

## 快速开始

### 1. 安装依赖

```bash
# 安装系统依赖
sudo apt update
sudo apt install libgoogle-glog-dev

# 安装 ROS2 依赖
cd ~/quadruped_ws
rosdep install --from-paths src --ignore-src -r -y

# 初始化子模块：
# 本仓库使用 Git 子模块管理依赖：
# - `livox_ros_driver2` - Livox ROS2 驱动(这个包依赖Livox-SDK2)
# - `livox_laser_simulation_ros2` - Livox 激光雷达仿真
git submodule update --init --recursive
```

### 2. 编译

```bash
cd ~/quadruped_ws
colcon build --symlink-install
```

### 3. 启动仿真

**推荐方式 - 使用启动脚本：**

```bash
#cd进入到脚本所在的目录
./run_gazebo.sh
```

启动脚本会自动：
1. 清理残留的 Gazebo 进程
2. 清理 Gazebo 共享内存
3. 设置必要的环境变量（包括 `GAZEBO_PLUGIN_PATH`）
4. 启动 Gazebo 仿真

**手动启动方式：**

```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py
```

### 4. 键盘控制

在另一个终端中启动键盘控制节点：

```bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

**控制说明**：
- `1-8`：切换控制模式
- `W/S/A/D`：前后左右移动
- `I/K/J/L`：旋转控制
- `空格`：停止运动

详细键盘控制说明请参考 [键盘控制使用指南](./docs/keyboard_control_guide.md)

## 传感器配置

### Livox Mid-360 激光雷达

- 话题：`/mid360/points`、`/mid360/points_PointCloud2`
- 非重复扫描模式，真实点云数据仿真

### Realsense D435i 相机

- 话题：`/camera/color/image_raw`、`/camera/color/camera_info`
- 仅 RGB 模式，用于 FAST-LIVO 雷达点着色

详细传感器配置请参考 [传感器配置文档](./docs/sensor_configuration.md)

## 文档

- [传感器配置](./docs/sensor_configuration.md) - 传感器配置和使用
- [键盘控制指南](./docs/keyboard_control_guide.md) - 键盘控制使用说明
- [Gazebo Livox 集成](./docs/gazebo_livox_integration.md) - Livox 雷达集成指南
- [Realsense 配置总结](./docs/realsense_configuration_summary.md) - Realsense 相机配置
- [故障排除](./docs/troubleshooting.md) - 常见问题及解决方案

## 系统要求

- **操作系统**：Ubuntu 22.04
- **ROS2 版本**：Humble
- **Gazebo 版本**：Gazebo 11 (Classic)

## 常见问题

### Gazebo 插件路径

**使用启动脚本会自动设置此环境变量**，无需手动配置。

手动启动时需要设置：
```bash
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
```

### 进程清理

**使用启动脚本**（推荐）：启动脚本会自动清理残留进程

```bash
./run_gazebo.sh
```

**手动清理：**

```bash
pkill -9 -f gazebo && pkill -9 -f gzserver && pkill -9 -f gzclient && pkill -9 -f gz
```

如果仍有残留，使用以下命令检查并清理：
```bash
ps aux | grep -E "(gazebo|gzserver|gzclient)" | grep -v grep
fuser -k 11345/tcp
```

更多故障排除方法请参考 [故障排除指南](./docs/troubleshooting.md)
