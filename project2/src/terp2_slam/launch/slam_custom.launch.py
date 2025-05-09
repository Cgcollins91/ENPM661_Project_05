from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = get_package_share_directory('terp2_slam')
    robot = get_package_share_directory('terp2')
    rviz_config_dir = PathJoinSubstitution([pkg, "rviz", "slam_custom.rviz"])
    laser_filter_config = PathJoinSubstitution([robot, 'config', 'scan_filter.yaml' ])
    return LaunchDescription([

        Node(
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

                # ---------- scan matcher (odometry) ----------
        Node(
            package   = 'terp2_slam',
            executable= 'scan_matcher',
            name      = 'scan_matcher',
            parameters=[{'use_sim_time': True}],
            # remappings=[('scan', '/scan_filtered')],   # <── use filtered scan
            output    = 'screen'
        ),

        # ---------- occupancy-grid mapper ----------
        Node(
            package   = 'terp2_slam',
            executable= 'mapper_node',
            name      = 'mapper',
            parameters=[{'use_sim_time': True}],
            # remappings=[('scan', '/scan_filtered')],   # <── use filtered scan
            output    = 'screen'
        ),


        Node(package="rviz2",
             executable="rviz2",
             output='screen',
             name='rviz_node_slam',
             parameters=[{'use_sim_time': True}],
             arguments=['-d', rviz_config_dir]
    )
])
