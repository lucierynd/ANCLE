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

    # Start RPlidar
    rplidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ancle_pkg'),'launch','rplidar.launch.py')])
    )

    # Start cyglidar
    cyglidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ancle_pkg'),'launch','cyglidar.launch.py')])
    )


    # transform from 2d lidar to base_link
    transform_rplidar_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='transform_rplidar_base_link',
        arguments=['0.0', '0.0', '0.1', '3.14', '0.0', '0.0', 'base_link', 'laser']
    )

    # transform from imu to base_link
    transform_imu_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='transform_imu_base_link',
        arguments=['0.0', '0.0', '0.0', '3.14', '0.0', '0.0', 'base_link', 'imu_link']
    )

    # transform from 3D lidar to base_link
    transform_cyglidar_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='transform_cyglidar_base_link',
        arguments=['0.0', '0.0', '0.025', '0.0', '0.0', '0.0', 'base_link', 'laser_frame']
    )


    # IMU python publisher 
    imu_publisher = Node(
        package='ancle_pkg',
        executable='imu_publisher.py',
        name='imu_publisher'
    )


    # RF2O Node
    rf2o = Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            arguments=["--ros-args", "--log-level", "rf2o_laser_odometry:=error"],
            parameters=[
                {"laser_scan_topic": "/scan"},
                {"odom_topic": "/odom_rf2o_raw"},
                {"publish_tf": False},      
                {"base_frame_id": "base_link"},
                {"odom_frame_id": "odom"},
                {"init_pose_from_topic": ""},
                {"freq": 20.0},
                {"use_sim_time": False}
            ]
    )

    # Projects rf2o estimate depending on imu attitude
    project_rf2o = Node(
            package="ancle_pkg",
            executable="project_rf2o.py",
            name="project_rf2o",
            parameters=[{"use_sim_time": False}],
            output="screen"
    )


    # Publishes static altitude data (no pressure sensor yet)
    static_altimeter_odom = Node(
            package="ancle_pkg",
            executable="static_altimeter_odom.py",
            name="static_altimeter_odom",
            parameters=[{"use_sim_time": False}],
            output="screen",
    )

    # EKF Node: 3D mode, uses 2D lidar + IMU + 3D Lidar
    ekf_3D = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory('ancle_pkg'),"params","ekf_3D_config.yaml")],
    )

    fuse_odom = Node(
            package="ancle_pkg",
            executable="fuse_odom.py",
            name="fuse_odom",
            parameters=[{"use_sim_time": False}],
            output="screen"
    )

    lidar_icp = Node(
            package="ancle_pkg",
            executable="lidar_icp.py",
            name="lidar_icp",
            parameters=[{"use_sim_time": False}],
            output="screen"
    )

    # RViz Node 
    rviz = Node(
        package='rviz2' ,
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('ancle_pkg'), 'rviz', 'rviz_octomap_slam_config.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')]),
                launch_arguments={'slam_params_file': os.path.join(
                    get_package_share_directory('ancle_pkg'), 'params', 'slam_toolbox_rplidar_config.yaml')}.items()
    )

    octomap = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ancle_pkg'), 'launch', 'octomap.launch.py')]),
    )

    # Wait that everything is ready before launching slam and octomap
    delayedNodes = TimerAction(
        period=10.0,
        actions=[slam_toolbox, octomap],
    )

    return LaunchDescription([
        declare_rviz_arg,
        rplidar,
        #cyglidar,
        transform_rplidar_base_link,
        transform_imu_base_link,
        #transform_cyglidar_base_link,
        imu_publisher,
        rf2o,
        project_rf2o,
        #fuse_odom,
        #lidar_icp,
        static_altimeter_odom,
        ekf_3D,
        rviz,
        # delayedNodes
    ])