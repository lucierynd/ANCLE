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
    tf_publisher_pkg='tf2_ros'
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

    # transform from imu to base_link
    transform_imu_base_link = Node(
        package=tf_publisher_pkg,
        executable='static_transform_publisher',
        name='transform_imu_base_link',
        arguments=['0.0', '0.0', '0.0', '3.14', '0.0', '0.0', 'base_link', 'imu_link']
    )

    # IMU python publisher 
    imu_publisher = Node(
        package=custom_pkg,
        executable='imu_publisher.py',
        name='imu_publisher'
    )


    # RF2O node
    rf2o = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(custom_pkg),'launch','rf2o_no_tf_launch.py')])
    )

    # EKF node
    ekf = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(custom_pkg),'launch','ekf_launch.py')])
    )

    # SLAM Toolbox node
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(slam_pkg),'launch','online_async_launch.py')]),
                launch_arguments={'slam_params_file': os.path.join(
                    get_package_share_directory(custom_pkg), 'config', 'slam_toolbox_rplidar_config.yaml')}.items()
    )

    # Start cyglidar

    cyglidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(custom_pkg),'launch','cyglidar_launch.py')])
    )

    # # Nav2 node (too heavy to launch together with SLAM Toolbox)

    # nav2 = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(nav2_pkg),'launch','navigation_launch.py')]),
    #             launch_arguments={'params_file': os.path.join(
    #                 get_package_share_directory(custom_pkg), 'config', 'nav2_config.yaml')}.items()
    # )

    # RViz node 
    rviz = Node(
        package='rviz2' ,
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory(custom_pkg), 'rviz', 'rviz_nav2_multi_lidar_inertial_slam_config.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )



    # Launch them all!
    return LaunchDescription([
        declare_rviz_arg,
        rplidar,
        transform_lidar_base_link,
        transform_imu_base_link,
        imu_publisher,
        rf2o,
        ekf,
        slam_toolbox,
        cyglidar,
        rviz
    ])