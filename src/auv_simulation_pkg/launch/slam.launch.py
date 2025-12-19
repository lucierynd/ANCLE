import os

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory("auv_simulation_pkg")

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')]),
                launch_arguments={'slam_params_file': os.path.join(
                    get_package_share_directory('ancle_pkg'), 'config', 'slam_toolbox_rplidar_config.yaml')}.items()
    )

    return LaunchDescription([slam_toolbox])
