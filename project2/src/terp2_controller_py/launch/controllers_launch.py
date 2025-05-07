#  file: terp2_controller_py/launch/controllers_launch.py
#  usage: ros2 launch terp2_controller_py controllers_launch.py

from launch               import LaunchDescription
from launch_ros.actions    import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions     import IfCondition

def generate_launch_description():

    # ───────────────── configurable arguments ──────────────────
    use_sim_time   = LaunchConfiguration('use_sim_time', default='true')

    controller_node = Node(
        package   = 'terp2_controller_py',
        executable = 'controller_py',      # ← entry-point name in setup.py
        name      = 'controller_py',
        output    = 'screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    imu_to_odom_node = Node(
        package   = 'terp2_controller_py',
        executable = 'imu_to_odom',        # ← entry-point name in setup.py
        name      = 'imu_to_odom',
        output    = 'screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        # you can expose the arg on the command line:
        #   ros2 launch terp2_controller_py controllers_launch.py use_sim_time:=false
        controller_node,
        imu_to_odom_node,
    ])
