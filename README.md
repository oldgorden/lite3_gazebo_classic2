# Lite3 Gazebo Classic Usage Guide

## Features

This repository supports complete Lite3 quadruped robot simulation, including:
- **Livox MID360 LiDAR**: Non-repetitive scanning mode with realistic point cloud data simulation
- **Realsense D435i Depth Camera**: RGB-D data
- **IMU Sensor**: Inertial Measurement Unit
- **Complete Controllers**: PD controller, Unitree Guide Controller, etc.

## Installation Steps

### 1. Clone Repository

```bash
# Execute in the src directory of your ROS2 workspace
cd your_ros2_workspace/src
git clone https://github.com/LaoGordon/lite3_gazebo_classic.git
```

### 2. Initialize Submodules (Recommended)

```bash
# Enter the repository directory
cd your_ros2_workspace/src/lite3_gazebo_classic

# Initialize and update all submodules
git submodule update --init --recursive

# If submodule cloning fails, you can add manually (optional)
# git submodule add https://github.com/LihanChen2004/livox_laser_simulation_ros2.git src/livox_laser_simulation_ros2
# git submodule add https://github.com/pal-robotics/realsense_gazebo_plugin.git src/realsense_gazebo_plugin  
# git submodule add https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
```

### 3. Install Dependencies

```bash
# Install system dependencies
sudo apt update
sudo apt install liblivox-sdk2-dev libgoogle-glog-dev

# Install ROS2 dependencies
cd your_ros2_workspace
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build Workspace

```bash
# Build all packages (including Livox-related dependencies)
colcon build --symlink-install

# Or build only the lite3_description package
colcon build --packages-select lite3_description --symlink-install
```

### 4. Set Environment Variables

```bash
# Must be set before each run
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
source install/setup.bash
```

## Run Simulation

### Full Simulation with GUI

```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py
```

This will start:
- Gazebo simulation environment (with GUI)
- RViz visualization tool
- All necessary controllers
- Livox MID360 LiDAR point cloud published to `/mid360/points` topic

### Server-only Mode (No GUI)

```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
source install/setup.bash
ros2 launch lite3_description gazebo_server_only.launch.py
```

## Common Issues and Solutions

### 1. Gazebo Server Crash (exit code -11)

**Symptom**:
```
[ERROR] [gzserver-3]: process has died [pid XXX, exit code -11, cmd 'gzserver ...']
```

**Cause**: Livox plugin incompatible with Gazebo sensor noise configuration

**Solution**: Fixed by modifying sensor configuration file. Ensure you're using the latest version where `src/lite3_gazebo_classic/lite3_description/xacro/sensors/mid360_sensor.xacro` has the noise configuration removed.

### 2. "[Err] [Sensor.cc:510] Get noise index not valid" Error

**Symptom**:
```
[gzserver-3] [Err] [Sensor.cc:510] Get noise index not valid
```

**Cause**: Gazebo 11 sensor noise configuration format incompatible with Livox plugin

**Solution**: The `<noise>` configuration block has been removed from the MID360 sensor XACRO file. This warning may still appear but doesn't affect Livox plugin functionality.

### 3. Livox Plugin Fails to Load

**Symptom**:
Livox point cloud data not published, no point cloud visible in RVIZ

**Solution**:
1. Ensure Gazebo plugin path is correctly set:
   ```bash
   export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
   ```
2. Verify plugin file exists:
   ```bash
   ls $(pwd)/install/ros2_livox_simulation/lib/libros2_livox.so
   ```

### 4. Gazebo Process Management

**Symptoms**:
- Port already in use: `[Err] [Master.cc:96] EXCEPTION: Unable to start server[bind: Address already in use]`
- Multiple Gazebo instance conflicts
- Simulation fails to shut down properly

**Complete Process Cleanup Solutions**:
```bash
# Method 1: Quick cleanup of all related processes (Recommended)
pkill -f gz && pkill -f gazebo && pkill -f ros2

# Method 2: Step-by-step cleanup (Safer)
# Terminate Gazebo server and client
pkill -f gzserver
pkill -f gzclient
# Terminate ROS2 related processes
pkill -f ros2
# Clean up potential residual processes
pkill -f rviz2
pkill -f robot_state_publisher

# Method 3: Force cleanup (When other methods fail)
sudo pkill -9 -f gz
sudo pkill -9 -f gazebo
sudo pkill -9 -f ros2

# Clean up Gazebo cache and temporary files
rm -rf ~/.gazebo
rm -rf /tmp/gazebo-*
```

**Prevention Tips**:
- Always run cleanup commands before starting simulation
- Use `Ctrl+C` to properly terminate simulation instead of closing terminal directly
- If simulation hangs, try `ros2 lifecycle set` commands first, then proceed with process cleanup

## Verify Livox Functionality

### Check Point Cloud Topic

```bash
# List Livox point cloud topics
ros2 topic list | grep mid360

# View point cloud data (in another terminal)
ros2 topic echo /mid360/points --no-arr | head -20
```

### Display Point Cloud in RVIZ

1. Launch RVIZ: `ros2 run rviz2 rviz2`
2. Add PointCloud2 display
3. Set Topic to: `/mid360/points`
4. Set Fixed Frame to: `base`

## System Requirements

- **Operating System**: Ubuntu 22.04
- **ROS2 Version**: Humble
- **Gazebo Version**: Gazebo 11 (Classic)
- **Hardware Requirements**: OpenGL-capable graphics card, 8GB+ RAM recommended

## Directory Structure

- `lite3_description/`: Robot URDF descriptions and configuration files
- `lite3_description/xacro/sensors/`: Sensor XACRO files (including fixed MID360 configuration)
- `lite3_description/launch/`: Launch files
- `lite3_description/config/`: Controller configuration files

## Contact

If you have any questions, please contact the project maintainer or submit an issue on GitHub.# lite3_gazebo_classic2
