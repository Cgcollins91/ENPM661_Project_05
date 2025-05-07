from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(package="terp2_slam", executable="scan_matcher",
             name="scan_matcher", output="screen"),

        Node(package="terp2_slam",
             executable="mapper_node",
             parameters=[{"use_sim_time": True}],
             output="screen"),

        Node(package="rviz2",
             executable="rviz2",
             arguments=["-d", 
                 str(__file__).replace("slam_custom.launch.py",
                                       "slam_custom.rviz")],
             output="screen"),
    ])
