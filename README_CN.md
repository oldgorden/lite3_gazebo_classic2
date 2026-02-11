# Lite3 Gazebo Classic 中文使用指南

## 功能特性

本仓库支持完整的Lite3四足机器人仿真，包括：
- **Livox MID360激光雷达**：非重复扫描模式，真实点云数据仿真
- **Realsense D435i深度相机**：RGB-D数据
- **IMU传感器**：惯性测量单元
- **完整控制器**：PD控制器、Unitree Guide Controller等

## 安装步骤

### 1. 克隆仓库

```bash
# 在ROS2工作空间的src目录中执行
cd your_ros2_workspace/src
git clone https://github.com/oldgorden/lite3_gazebo_classic2.git
```

### 2. 初始化子模块（推荐方式）

```bash
# 进入仓库目录
cd your_ros2_workspace/src/lite3_gazebo_classic

# 初始化并更新所有子模块
git submodule update --init --recursive

# 如果子模块克隆失败，可以手动添加（可选）
# git submodule add https://github.com/LihanChen2004/livox_laser_simulation_ros2.git src/livox_laser_simulation_ros2
# git submodule add https://github.com/pal-robotics/realsense_gazebo_plugin.git src/realsense_gazebo_plugin  
# git submodule add https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
```

### 3. 安装依赖

```bash
# 安装系统依赖
sudo apt update
sudo apt install liblivox-sdk2-dev libgoogle-glog-dev

# 安装ROS2依赖
cd your_ros2_workspace
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 编译工作空间

```bash
# 编译所有包（包括Livox相关依赖）
colcon build --symlink-install

# 或者只编译lite3_description包
colcon build --packages-select lite3_description --symlink-install
```

### 4. 设置环境变量

```bash
# 在每次运行前都需要设置
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
source install/setup.bash
```

## 运行仿真

### 带GUI界面的完整仿真

```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py
```

这将启动：
- Gazebo仿真环境（带GUI界面）
- RViz可视化工具
- 所有必要的控制器
- Livox MID360激光雷达点云发布到 `/mid360/points` 话题

### 仅服务器模式（无GUI）

```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
source install/setup.bash
ros2 launch lite3_description gazebo_server_only.launch.py
```

## 常见问题及解决方案

### 1. Gazebo服务器崩溃（exit code -11）

**问题现象**：
```
[ERROR] [gzserver-3]: process has died [pid XXX, exit code -11, cmd 'gzserver ...']
```

**原因**：Livox插件与Gazebo传感器噪声配置不兼容

**解决方案**：已通过修改传感器配置文件解决。确保使用最新版本的代码，其中 `src/lite3_gazebo_classic/lite3_description/xacro/sensors/mid360_sensor.xacro` 文件中已移除噪声配置部分。

### 2. "[Err] [Sensor.cc:510] Get noise index not valid" 错误

**问题现象**：
```
[gzserver-3] [Err] [Sensor.cc:510] Get noise index not valid
```

**原因**：Gazebo 11中的传感器噪声配置格式与Livox插件不兼容

**解决方案**：已在MID360传感器XACRO文件中移除`<noise>`配置块，避免此错误。虽然仍有此警告，但不影响Livox插件正常工作。

### 3. Livox插件无法加载

**问题现象**：
Livox点云数据未发布，RVIZ中看不到点云

**解决方案**：
1. 确保正确设置了Gazebo插件路径：
   ```bash
   export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
   ```
2. 验证插件文件存在：
   ```bash
   ls $(pwd)/install/ros2_livox_simulation/lib/libros2_livox.so
   ```

### 4. Gazebo进程管理

**问题现象**：
- 端口被占用：`[Err] [Master.cc:96] EXCEPTION: Unable to start server[bind: Address already in use]`
- 多个Gazebo实例冲突
- 仿真运行后无法正常关闭

**完整进程清理方案**：
```bash
# 方法1：快速清理所有相关进程（推荐）
pkill -f gz && pkill -f gazebo && pkill -f ros2

# 方法2：逐步清理（更安全）
# 终止Gazebo服务器和客户端
pkill -f gzserver
pkill -f gzclient
# 终止ROS2相关进程
pkill -f ros2
# 清理可能的残留进程
pkill -f rviz2
pkill -f robot_state_publisher

# 方法3：强制清理（当其他方法无效时）
sudo pkill -9 -f gz
sudo pkill -9 -f gazebo
sudo pkill -9 -f ros2

# 清理Gazebo缓存和临时文件
rm -rf ~/.gazebo
rm -rf /tmp/gazebo-*
```

**预防措施**：
- 每次运行仿真前先执行清理命令
- 使用 `Ctrl+C` 正常终止仿真，避免直接关闭终端
- 如果仿真卡死，先尝试 `ros2 lifecycle set` 命令，再进行进程清理

## 验证Livox功能

### 检查点云话题

```bash
# 查看Livox点云话题
ros2 topic list | grep mid360

# 查看点云数据（在另一个终端中）
ros2 topic echo /mid360/points --no-arr | head -20
```

### RVIZ中显示点云

1. 启动RVIZ：`ros2 run rviz2 rviz2`
2. 添加PointCloud2显示
3. 设置Topic为：`/mid360/points`
4. 设置Fixed Frame为：`base`

## 系统要求

- **操作系统**：Ubuntu 22.04
- **ROS2版本**：Humble
- **Gazebo版本**：Gazebo 11 (Classic)
- **硬件要求**：支持OpenGL的显卡，建议8GB以上内存

## 目录结构说明

- `lite3_description/`：机器人URDF描述和配置文件
- `lite3_description/xacro/sensors/`：传感器XACRO文件（包括已修复的MID360配置）
- `lite3_description/launch/`：启动文件
- `lite3_description/config/`：控制器配置文件

## 联系方式

如果有任何问题，请联系项目维护者或在GitHub上提交issue。