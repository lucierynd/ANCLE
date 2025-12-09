import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # Define package names
    custom_pkg='ancle_pkg' 
    lidar_pkg='rplidar_ros'
    tf_publisher_pkg='tf2_ros'
    rf2o_pkg='rf2o_laser_odometry'
    slam_pkg='slam_toolbox'

    # Declare the launch argument for RViz
    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Start RPlidar
    rplidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(lidar_pkg),'launch','rplidar_c1_launch.py')])
    )

    # transform from lidar to base_link
    transform_lidar_base_link = Node(
        package=tf_publisher_pkg,
        executable='static_transform_publisher',
        name='transform_lidar_base_link',
        arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    # rf2o Node
    rf2o = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(rf2o_pkg),'launch','rf2o_laser_odometry.launch.py')])
    )

    # SLAM Toolbox Node
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(slam_pkg),'launch','online_async_launch.py')]),
                launch_arguments={'slam_params_file': os.path.join(
                    get_package_share_directory(custom_pkg), 'config', 'slam_toolbox_rplidar_config.yaml')}.items()
    )

    # RViz Node 
    rviz = Node(
        package='rviz2' ,
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory(custom_pkg), 'rviz', 'rviz_slam_tool_box_config.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Launch them all!
    return LaunchDescription([
        declare_rviz_arg,
        rplidar,
        transform_lidar_base_link,
        rf2o,
        slam_toolbox,
        rviz
    ])