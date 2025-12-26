from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,

                # Frames
                'frame_id': 'map',
                'base_frame_id': 'base_link',

                # Map resolution (meters)
                'resolution': 0.5,

                # Sensor model
                'sensor_model/max_range': 20.0,

                # Filtering
                'pointcloud_min_z': -2.0,
                'pointcloud_max_z':  2.0,

                # Publish options
                'publish_octomap_binary': True,
                'publish_octomap_full': False,
                'publish_pointcloud': False,
            }],
            remappings=[
                ('cloud_in', 'camera_front/depth/scan_3D')
            ]
        )

    ])
