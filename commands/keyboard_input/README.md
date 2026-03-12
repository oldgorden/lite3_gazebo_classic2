# keyboard_input

Lite3 主控制链的键盘遥操作节点。

## 输出话题

- `/cmd_vel`
- `/robot_mode`

## 运行

```bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

## 参考

- 完整按键说明见 [keyboard_control_guide.md](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/keyboard_control_guide.md)
- 标准接口定义见 [control_interface_spec.md](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/control_interface_spec.md)
