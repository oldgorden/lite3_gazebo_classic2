from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_input',
            executable='keyboard_input',
            name='keyboard_input_node',
            output='screen',
        )
    ])
