#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'task_amr_bringup'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory(package_name),
            'map',
            'house1.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 'rviz', 'nav2_default_view.rviz')

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory(package_name),
            'config',
            'my_amr.yaml'))

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load'
    ))

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to Params file to load'
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    ))

    return ld
