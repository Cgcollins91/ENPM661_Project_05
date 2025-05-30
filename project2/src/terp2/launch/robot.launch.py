
# Project: ENPM661_Project_05
# License: MIT
# The code in this file represents the collective work of Group 2.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.

import os

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

    robot_pkg  = get_package_share_directory(robot_name)

    position    = [0.0, 0.0, 0.8]
    orientation = [0.0, 0.0, -90.0]


    robot_urdf = os.path.join(robot_pkg, "urdf", xacro_file)
    xml        = xacro.process_file(robot_urdf).toxml()
    controller_params_file = os.path.join(robot_pkg, 'config', 'control.yaml')


    entity_name = robot_name # +"-"+str(random.random())
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name,
                   '-x', str(position[0]),    '-y', str(position[1] ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1] ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="screen",
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': xml}, controller_params_file],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    delayed_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    delayed_velocity_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )
    
    delayed_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delayed_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    gui_arg      = launch.actions.DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui')
    sim_time_arg = launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time')

    controller_node = Node(
        package    = 'terp2_controller_py',
        executable = 'controller_py',      # ← entry-point name in setup.py
        name       = 'controller_py',
        output     = 'screen',
        parameters = [{'use_sim_time': use_sim_time}],
    )


    model_state_to_odom_node = Node(
        package    = 'terp2_controller_py',
        executable = 'model_state_to_odom',        # ← entry-point name in setup.py
        name       = 'model_state_to_odom',
        output     = 'screen',
        parameters = [{'use_sim_time': use_sim_time}]
    )

   
    # joint_state_gui=Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )
    # rviz_arg = launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_dir,
    #                                  description='Absolute path to rviz config file')
    # Static TF Transform
    # tf=Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     output='screen',
    #     arguments=['1', '0', '0', '0', '0', '0', '1', '/base_link',  '/base_link'  ],
    # )

    return LaunchDescription(
        [
            gui_arg,
            sim_time_arg,
            robot_state_publisher,
            robot_node,
            controller_manager,  
            joint_state_broadcaster_spawner,
            delayed_position_controller_spawner,
            delayed_velocity_controller_spawner,
            delayed_arm_controller_spawner,  
            delayed_gripper_controller_spawner,  
            controller_node,
            model_state_to_odom_node 
             # rviz_node
        ]
    )