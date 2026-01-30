from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_vel_bridge',
            executable='cmd_vel_bridge_node',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{
                'max_vx': 0.4,          # 前进速度 m/s
                'max_vy': 0.3,          # 横移速度 m/s
                'max_wz': 0.2,          # 转向速度 rad/s
                'watchdog_timeout': 0.2 # 看门狗超时时间
            }]
        )
    ])