# terp2/launch/library_world.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('terp2')
    world = os.path.join(pkg_share, 'worlds', 'library.world')
    model_path = os.path.join(pkg_share, 'models')

    # prepend our model path to whatever the user already has
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items())

    return LaunchDescription([
        set_model_path,
        gazebo
    ])