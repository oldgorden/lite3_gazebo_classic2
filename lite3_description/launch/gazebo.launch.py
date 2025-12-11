import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'lite3_description'
    
    # 1. 解析 Xacro
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 2. 启动 Gazebo (暂停模式 -u，这样你可以看清它初始的状态，不会一进去就乱飞)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'pause': 'false'}.items() 
    )

    # 3. 把机器人“生”出来
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'lite3_robot',
                   '-z', '0.5'], # 也就是出生高度，0.5米
        output='screen'
    )

    # 4. 发布状态 (必须要，否则 Gazebo 里模型可能是白的或者乱的)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    return LaunchDescription([
        #设置环境变量
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        SetEnvironmentVariable(name='GTK_IM_MODULE', value=''),
        SetEnvironmentVariable(name='QT_IM_MODULE', value=''),
        SetEnvironmentVariable(name='XMODIFIERS', value=''),

        #启动节点
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])