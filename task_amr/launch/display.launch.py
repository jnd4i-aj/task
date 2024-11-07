#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import xacro
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    package_name = 'task_amr'

    xacro_file = os.path.join(get_package_share_directory(package_name),
                              'urdf', 'robot.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(get_package_share_directory(package_name),
                                    'config', 'display.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # use only for .urdf file not for .xacro file
    # with open(xacro_file, 'r') as infp:
    #     robot_desc = infp.read()

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_urdf
        }],
    )

    # joint_state_pub = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui'
    #     )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_pub)
    # ld.add_action(rviz)

    return ld
