# joystick_input

Lite3 主控制链的普通手柄遥操作节点。

## 输出话题

- `/cmd_vel`
- `/robot_mode`

## 运行

```bash
source install/setup.bash
ros2 launch joystick_input joystick.launch.py
```

## 参考

- 标准接口定义见 [control_interface_spec.md](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/control_interface_spec.md)
