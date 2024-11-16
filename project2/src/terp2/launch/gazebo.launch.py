
# Project: ENPM662-Project1-Group1
# License: MIT
# The code in this file represents the collective work of Group 1.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    gazebo = get_package_share_directory('gazebo_ros')
    robot = get_package_share_directory('terp2')

    world = os.path.join(
        get_package_share_directory('terp2'),
        'worlds',
        'empty_world.world'
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot, 'launch', 'robot.launch.py')
        )
    )

    return LaunchDescription([
        gz_server,
        gz_client,
        spawn_robot
        
    ])
