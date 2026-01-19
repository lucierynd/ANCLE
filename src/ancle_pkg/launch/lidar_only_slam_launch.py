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
                    get_package_share_directory(custom_pkg),'launch','rplidar_launch.py')])
    )

    # transform from lidar to base_link
    transform_lidar_base_link = Node(
        package=tf_publisher_pkg,
        executable='static_transform_publisher',
        name='transform_lidar_base_link',
        arguments=['0.0', '0.0', '0.1', '3.14', '0.0', '0.0', 'base_link', 'laser']
    )

    # rf2o Node
    # RF2O Node
    rf2o = Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            arguments=["--ros-args", "--log-level", "rf2o_laser_odometry:=error"],
            parameters=[
                {"laser_scan_topic": "/scan"},
                {"odom_topic": "/odom_rf2o"},
                {"publish_tf": True},      
                {"base_frame_id": "base_link"},
                {"odom_frame_id": "odom"},
                {"init_pose_from_topic": ""},
                {"freq": 20.0},
                {"use_sim_time": True}
            ]
    )

    # SLAM Toolbox Node
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(slam_pkg),'launch','online_async_launch.py')]),
                launch_arguments={'slam_params_file': os.path.join(
                    get_package_share_directory(custom_pkg), 'params', 'slam_toolbox_rplidar_config.yaml')}.items()
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