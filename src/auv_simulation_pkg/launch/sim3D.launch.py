import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

package_name = "auv_simulation_pkg"


def generate_static_tf_publisher_node(parentFrame, childFrame):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_publisher_{parentFrame}",
        arguments=["0", "0", "0", "0", "0", "0", parentFrame, childFrame],
        parameters=[{"use_sim_time": True}],
    )


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_path = get_package_share_directory(package_name)

    rviz_config_file = LaunchConfiguration("rviz_config")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_path, "rviz", "basic.rviz"),
        description="Path to rviz config file",
    )

    model = os.path.join(pkg_path, "models", "tethys_rviz", "model.sdf")

    with open(model, "r") as infp:
        robot_desc = infp.read().replace(
            "<uri>", f"<uri>package://{package_name}/models/tethys_rviz/"
        )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": f"-r \"/ros2_ws/src/auv_simulation_pkg/models/pool/pool_party.sdf\""
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tethys_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/world/auv_world/model/tethys/link/camera_front/sensor/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/world/auv_world/model/tethys/link/camera_front/sensor/color/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/world/auv_world/model/tethys/link/camera_front/sensor/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/world/auv_world/model/tethys/link/camera_front/sensor/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/world/auv_world/model/tethys/link/camera_front/sensor/depth/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/world/auv_world/model/tethys/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/auv_world/model/tethys/link/altimeter_link/sensor/altimeter/altimeter@ros_gz_interfaces/msg/Altimeter[gz.msgs.Altimeter",
            "/model/tethys/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/tethys/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/world/auv_world/model/tethys/link/scan_omni/sensor/scan_omni/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/world/auv_world/model/tethys/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/model/tethys/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/tethys/pose@geometry_msgs/msg/TransformStamped[gz.msgs.Pose",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        remappings=[
            (
                "/world/auv_world/model/tethys/link/camera_front/sensor/color/camera_info",
                "camera_front/camera_info",
            ),
            (
                "/world/auv_world/model/tethys/link/camera_front/sensor/color/image",
                "camera_front/image",
            ),
            (
                "/world/auv_world/model/tethys/link/camera_front/sensor/depth/camera_info",
                "camera_front/depth/camera_info",
            ),
            (
                "/world/auv_world/model/tethys/link/camera_front/sensor/depth/depth_image",
                "camera_front/depth/image",
            ),
            (
                "/world/auv_world/model/tethys/link/camera_front/sensor/depth/depth_image/points",
                "camera_front/depth/scan_3D",
            ),
            ("/model/tethys/odometry", "odom"),
            ("/model/tethys/cmd_vel", "cmd_vel"),
            (
                "/world/auv_world/model/tethys/link/imu_link/sensor/imu/imu",
                "imu/data",
            ),
            (
                "/world/auv_world/model/tethys/link/altimeter_link/sensor/altimeter/altimeter",
                "altimeter/data",
            ),
            (
                "/world/auv_world/model/tethys/link/scan_omni/sensor/scan_omni/scan",
                "scan",
            ),
            ("/world/auv_world/model/tethys/joint_state", "joint_states"),
        ],
        output="screen",
    )

    delayedNodes = TimerAction(
        period=2.5,
        actions=[rviz, bridge],
    )

    robotState = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    # IMU + lidar odometry without TF publishing
    # RF2O Node - Lidar only odometry without TF publishing
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
                {"use_sim_time": True}
            ]
    )

    # KISS-ICP Odometry
    kiss_icp = Node(
            package="kiss_icp",
            executable="kiss_icp_node",
            name="kiss_icp",
            remappings=[
            ("pointcloud_topic", "/camera_front/depth/scan_3D"),
            ],
            parameters=[
                {"publish_odom_tf": False},   
                {"lidar_odom_frame": "odom"},
                {"base_frame": "base_link"},
                {"use_sim_time": True}
            ],
            output="screen"
    )

    altimeter_odom = Node(
            package="auv_simulation_pkg",
            executable="altimeter_to_odom.py",
            name="altimeter_odom",
            parameters=[{"use_sim_time": True}],
            output="screen"
    )

    # EKF Node
    ekf = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory("auv_simulation_pkg"),"params","ekf_3D_config.yaml")]
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            gazebo,
            robotState,
            delayedNodes,
            generate_static_tf_publisher_node(
                "scan_omni", "tethys/scan_omni/scan_omni"
            ),
            generate_static_tf_publisher_node(
                "camera_front", "tethys/camera_front/color"
            ),
            generate_static_tf_publisher_node(
                "camera_front", "tethys/camera_front/depth"
            ),
            generate_static_tf_publisher_node(
                "imu_link", "tethys/imu_link/imu"
            ),
            rf2o,
            kiss_icp,
            altimeter_odom,
            ekf
        ]
    )
   