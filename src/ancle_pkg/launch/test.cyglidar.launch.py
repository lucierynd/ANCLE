import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    # Declare the launch argument for RViz
    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Start cyglidar
    cyglidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ancle_pkg'),'launch','cyglidar.launch.py')])
    )

    # transform from 3D lidar to base_link
    transform_cyglidar_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='transform_cyglidar_base_link',
        arguments=['0.0', '0.0', '0.025', '0.0', '0.0', '0.0', 'base_link', 'laser_frame']
    )

    # KISS-ICP Odometry
    kiss_icp = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", "/scan_3D"),
        ],
        parameters=[
            {
                # ROS node configuration
                "base_frame": "base_link",
                "lidar_odom_frame": "odom",
                "publish_odom_tf": True,
                "invert_odom_tf": False,
                # ROS CLI arguments
                "publish_debug_clouds": True,
                "use_sim_time": False,
                "position_covariance": 0.01,
                "orientation_covariance": 0.01,
            },
            os.path.join(get_package_share_directory("ancle_pkg"), "params", "kiss_icp_config.yaml")
        ],
    )

    # RViz Node 
    rviz = Node(
        package='rviz2' ,
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('ancle_pkg'), 'rviz', 'rviz_test_kiss_icp_config.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        cyglidar,
        transform_cyglidar_base_link,
        kiss_icp,
        rviz,
    ])