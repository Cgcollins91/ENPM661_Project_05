
# Project: ENPM661_Project_05
# License: MIT
# The code in this file represents the collective work of Group 2.

# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.

import os

import launch
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    gui_arg = launch.actions.DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui')
    sim_time_arg = launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time')

    # RVIZ Configuration
    rviz_config_dir = PathJoinSubstitution([FindPackageShare("terp2"), "rviz", "terp2.rviz"])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    joint_state_gui=Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [
            gui_arg,
            sim_time_arg,
            rviz_node,
            joint_state_gui
        ]
    )
