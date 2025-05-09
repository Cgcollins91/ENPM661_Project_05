
# Project: ENPM661_Project_05
# License: MIT
# The code in this file represents the collective work of Group 2.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.

import os
from launch.actions import TimerAction
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro
import random

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    xacro_file = "terp2.urdf.xacro"
    robot_name = "terp2"

    robot_pkg = get_package_share_directory(robot_name)
    slam_pkg = get_package_share_directory('terp2_slam')
    rviz_config_dir = PathJoinSubstitution([slam_pkg, "rviz", "slam_custom.rviz"])

    laser_filter_config = PathJoinSubstitution([robot_pkg, 'config', 'scan_filter.yaml' ])
    

    entity_name = robot_name # +"-"+str(random.random())
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # controller_node = Node(
    #     package   = 'terp2_controller_py',
    #     executable = 'controller_py',      # ← entry-point name in setup.py
    #     name      = 'controller_py',
    #     output    = 'screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    explorer = Node(
        package   = 'terp2_controller_py',
        executable = 'explorer',        # ← entry-point name in setup.py
        name      = 'explorer',
        output    = 'screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    terp2_slam =    Node(package="terp2_slam", executable="scan_matcher",
             name="scan_matcher", output="screen"
    )
    
    mapper = Node(package="terp2_slam",
             executable="mapper_node",
             parameters=[{"use_sim_time": True}],
             output="screen"
    )

    slam_rviz =    Node(package="rviz2",
             executable="rviz2",
             output='screen',
             name='rviz_node_slam',
             parameters=[{'use_sim_time': True}],
             arguments=['-d', rviz_config_dir]
    )
    filtered_scan = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_filter",
            parameters=[laser_filter_config,
                         {'use_sim_time': True} ],
            remappings=[
                ("scan",          "/scan"),          # input
                ("scan_filtered", "/scan_filtered")  # output
            ]
        ),

 
    # delayed_pid_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=imu_to_odom_node,
    #         on_exit=[controller_node],
    #     )
    # )


    # delayed_slam_spawner =RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=imu_to_odom_node,
    #         on_exit=[terp2_slam],
    #     )
    # )

    # delayed_mapper_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=terp2_slam,
    #         on_exit=[mapper],
    #     )
    # )

    # delayed_rviz_slam = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=mapper,
    #         on_exit=[slam_rviz],
    #     )
    # )


    gui_arg = launch.actions.DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui')
    sim_time_arg = launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time')

 

    return LaunchDescription(
        [
            gui_arg,
            sim_time_arg,
            explorer,
            terp2_slam,
            mapper,
            filtered_scan


            # rviz_node
        ]
    )
