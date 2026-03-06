#!/usr/bin/env python3
"""
FASTLIVO2 IMU 话题重映射启动文件

功能：将 ROS2-Control 的 IMU 话题重映射到 FASTLIVO2 期望的话题名
- /imu_sensor_broadcaster/imu  →  /livox/imu

使用方法：
  ros2 launch gz_quadruped_playground fastlivo2_remap.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # IMU 话题重映射节点
        # 使用 topic_tools 的 relay 节点转发 IMU 数据
        Node(
            package='topic_tools',
            executable='relay',
            name='imu_relay',
            namespace='',
            arguments=['/imu_sensor_broadcaster/imu', '/livox/imu'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
