# unitree_joystick_input

Lite3 主控制链的 Unitree 无线手柄输入节点。

## 输出话题

- `/cmd_vel`
- `/robot_mode`

## 运行

```bash
source install/setup.bash
ros2 launch unitree_joystick_input joystick.launch.py
```

## 说明

- 使用前需要在 launch 参数中设置正确的网卡名称
- 标准接口定义见 [control_interface_spec.md](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/control_interface_spec.md)
