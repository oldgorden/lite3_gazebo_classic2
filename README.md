Lite3 simulation on Gazebo Classic (ROS 2 Humble). Adapted from https://github.com/legubiao/quadruped_ros2_control/.

## 系统要求 (System Requirements)

### 环境依赖
- **Ubuntu 22.04** 
- **ROS 2 Humble** 
- **Gazebo Classic 11**

## 工作空间设置 (Workspace Setup)

### 1. 创建工作空间
```bash
mkdir -p ~/quadruped_ws/src
cd ~/quadruped_ws/src
```

### 2. 克隆项目
```bash
git clone https://github.com/LaoGordon/lite3_gazebo_classic.git
cd lite3_gazebo_classic
```

### 3. 初始化子模块（传感器依赖）
```bash
# 初始化并更新子模块
git submodule update --init --recursive
```

如果子模块未预先配置，请手动安装依赖：

## 传感器依赖安装（Livox Mid-360雷达 + Realsense D435i相机）

本仓库已集成Livox Mid-360激光雷达和Intel Realsense D435i相机，可提供真实的非重复扫描点云和深度图像。

### 手动安装依赖（如果未使用子模块）

#### 1. 克隆必需的插件
```bash
cd ~/quadruped_ws/src

# Livox激光雷达仿真插件（ROS2版本）
git clone https://github.com/LihanChen2004/livox_laser_simulation_ros2.git

# Realsense D435i相机插件
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
```

#### 2. 编译所有包
```bash
cd ~/quadruped_ws
colcon build --symlink-install
source install/setup.bash
```

#### 3. 启动仿真验证
```bash
# 启动Gazebo仿真
ros2 launch lite3_description gazebo_classic.launch.py

# 在新终端检查话题
ros2 topic list | grep -E "mid360|camera"
# 应看到：/mid360/points, /camera/depth/image_rect_raw, /camera/color/image_raw等话题
```

### 验证方法

#### 检查点云数据
```bash
# 检查点云话题
ros2 topic echo /mid360/points_PointCloud2 --field width --no-arr | head -1
# 应显示点云数量（如：3000）

# 检查数据频率
ros2 topic hz /mid360/points_PointCloud2 --window 10
# 应显示约2-10Hz的数据流
```

#### 检查TF变换
```bash
# 验证静态TF变换
ros2 run tf2_ros tf2_echo mid360_link mid360_livox_laser
# 应显示零变换
```

### 详细文档
完整集成指南和问题解决方案：[docs/gazebo_livox_integration.md](docs/gazebo_livox_integration.md)

### 应用场景配置建议

| 应用场景 | samples | downsample | update_rate | 说明 |
|----------|---------|------------|-------------|------|
| 实时SLAM | 6000 | 4 | 10Hz | 高密度、高频率，适合建图 |
| 导航避障 | 3000 | 8 | 5Hz | 中等密度，平衡性能与精度 |
| 演示展示 | 12000 | 2 | 5Hz | 最高密度，视觉效果最佳 |
| 性能测试 | 1500 | 16 | 2Hz | 最低负载，性能基准测试 |

**注意**：更高密度和频率会增加CPU/GPU负载，请根据系统性能调整参数。

## 编译运行 (Build and Run)

### 构建项目
```bash
cd ~/quadruped_ws
colcon build --symlink-install
```

### 配置环境
```bash
source ~/quadruped_ws/install/setup.bash
```

### 启动Gazebo Classic仿真
```bash
ros2 launch lite3_description gazebo_classic.launch.py
```

## 控制机器人 (Robot Control)

### 键盘控制 (Keyboard Control)
打开新终端：
```bash
cd ~/quadruped_ws
. install/setup.bash
ros2 run keyboard_input keyboard_input
```
## 四足机器人参数调整 
参数调整一般在const.xacro中调整
查看整体的urdf文件：
```bash
cd ~/quadruped_ws
source ~/quadruped_ws/install/setup.bash
ros2 run xacro xacro src/lite3_gazebo_classic/lite3_description/xacro/robot.xacro GAZEBO:=true CLASSIC:=true > src/lite3_gazebo_classic/lite3_description/urdf/robot.urdf
```

## 更新日志

- **2026-01-30**: 集成Livox Mid-360雷达和Realsense D435i相机
- **2026-01-30**: 添加静态TF变换解决RViz队列溢出问题
- **2026-01-30**: 提供详细集成文档和配置指南
