#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'task_amr'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    display_launch_file_path = os.path.join(get_package_share_directory(package_name), 'launch', 'display.launch.py')

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_launch_file_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'house_1.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    x_pose = LaunchConfiguration('x_pose', default='-4.0')
    y_pose = LaunchConfiguration('y_pose', default='-1.0')
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', '/robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(display_launch)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
