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

    declare_use_3d_ekf_arg = DeclareLaunchArgument(
        'use_3d_ekf',
        default_value='false',
        description='Whether to use 3D ekf (2D lidar + IMU + kiss-icp/3D Lidar + static altitude publisher) vs 2D mode (2D lidar + IMU only)'
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
                {"odom_topic": "/odom_rf2o"},
                {"publish_tf": False},      
                {"base_frame_id": "base_link"},
                {"odom_frame_id": "odom"},
                {"init_pose_from_topic": ""},
                {"freq": 20.0},
                {"use_sim_time": False}
            ]
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
                "publish_odom_tf": False,
                "invert_odom_tf": False,
                # ROS CLI arguments
                "publish_debug_clouds": True,
                "use_sim_time": False,
                "position_covariance": 0.01,
                "orientation_covariance": 0.01,
            },
            os.path.join(get_package_share_directory("ancle_pkg"), "params", "kiss_icp_config.yaml")
        ],
        condition=IfCondition(LaunchConfiguration('use_3d_ekf'))
    )

    # Publishes static altitude data (no pressure sensor yet)
    static_altimeter_odom = Node(
            package="ancle_pkg",
            executable="static_altimeter_odom.py",
            name="static_altimeter_odom",
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(LaunchConfiguration('use_3d_ekf'))
    )

    # EKF Node: 2D mode, uses 2D lidar + IMU
    ekf = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory('ancle_pkg'),"params","ekf_config.yaml")],
            condition=UnlessCondition(LaunchConfiguration('use_3d_ekf'))
    )

    # EKF Node: 3D mode, uses 2D lidar + IMU + 3D Lidar
    ekf_3D = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory('ancle_pkg'),"params","ekf_3D_config.yaml")],
            condition=IfCondition(LaunchConfiguration('use_3d_ekf'))
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

    # Wait that everything is settled before launching slam and octomap
    delayedNodes = TimerAction(
        period=10.0,
        actions=[slam_toolbox, octomap],
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_use_3d_ekf_arg,
        rplidar,
        cyglidar,
        transform_rplidar_base_link,
        transform_imu_base_link,
        transform_cyglidar_base_link,
        imu_publisher,
        rf2o,
        kiss_icp,
        static_altimeter_odom,
        ekf,
        ekf_3D,
        rviz,
        delayedNodes
    ])