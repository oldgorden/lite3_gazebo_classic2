# lite3_description

Lite3 机器人描述包，包含 URDF/Xacro、mesh、launch 文件和控制器配置。

## 作用

- 提供 Lite3 机器人模型
- 提供 Gazebo Classic 与可视化启动文件
- 存放控制器 YAML 和传感器 Xacro 配置

## 常用命令

```bash
source install/setup.bash
ros2 launch lite3_description visualize.launch.py
ros2 launch lite3_description gazebo_classic.launch.py
```

## 说明

- 系统级使用方式见顶层 [README](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/README.md)
- 控制接口约定见 [control_interface_spec.md](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/control_interface_spec.md)
