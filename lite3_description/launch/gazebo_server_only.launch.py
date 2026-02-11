import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):

    package_description = context.launch_configurations['pkg_description']
    init_height = context.launch_configurations['height']
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={'GAZEBO': 'true', 'CLASSIC': 'true'}).toxml()

    # 只启动Gazebo服务器，不启动GUI客户端
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])]
        ),
        launch_arguments={"verbose": "true"}.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robot", "-z", init_height],
        output="screen",
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_pd_controller",
                   "--controller-manager", "/controller_manager"],
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    unitree_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["unitree_guide_controller", "--controller-manager", "/controller_manager"],
    )

    return [
        #设置环境变量
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        SetEnvironmentVariable(name='GTK_IM_MODULE', value=''),
        SetEnvironmentVariable(name='QT_IM_MODULE', value=''),
        SetEnvironmentVariable(name='XMODIFIERS', value=''),

        robot_state_publisher,
        gazebo,
        spawn_entity,
        leg_pd_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=[imu_sensor_broadcaster, joint_state_publisher, unitree_guide_controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='lite3_description',
        description='package for robot description'
    )

    height = DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Init height in simulation'
    )

    return LaunchDescription([
        pkg_description,
        height,
        OpaqueFunction(function=launch_setup),
    ])