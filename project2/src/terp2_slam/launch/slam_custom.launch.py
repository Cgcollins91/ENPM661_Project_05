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
    scan_filter_params = {
        # this dict becomes the "ros__parameters" namespace
        "scan_filter_chain": [
            {
                "name":  "drop_self_hits",
                "type":  "laser_filters/LaserScanBoxFilter",
                "params": {
                    "box_frame": "laser_link",
                    "min_x": -0.30,
                    "max_x":  0.30,
                    "min_y": -0.30,
                    "max_y":  0.30,
                    "min_z": -0.40,
                    "max_z":  0.80,
                },
            }]}
    
    return LaunchDescription([

        Node(
            package    = 'terp2_slam',
            executable = 'box_filter',
            name       = 'box_filter',
            parameters = [{
                'box_frame': 'base_link',   # exact TF frame
                'min_x': -0.30, 'max_x': 0.30,
                'min_y': -0.30, 'max_y': 0.30,
                'min_z': -0.40, 'max_z': 1.50,
                'use_sim_time': True,
            }],
            remappings = [('scan','/scan'), ('scan_filtered','/scan_filtered')],
            output     = 'screen',
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
