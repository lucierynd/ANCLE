#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation
import signal
import sys
from datetime import datetime


class GroundTruthPoseNode(Node):
    def __init__(self):
        super().__init__('ground_truth_pose_node')
        
        # Initialize variables
        self.initial_pose = None
        self.trajectory_data = []
        self.initial_position = None
        self.initial_orientation = None
        
        # Create subscriber to the gazebo model pose topic
        self.subscription = self.create_subscription(
            TransformStamped,
            '/model/tethys/pose',
            self.pose_callback,
            10
        )
        
        # Create publisher for relative pose
        self.publisher = self.create_publisher(
            PoseStamped,
            '/simulation/pose',
            10
        )
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info('Ground Truth Pose Node started')
        self.get_logger().info('Waiting for initial pose...')
    
    def pose_callback(self, msg):
        # Filter messages based on frame_id and child_frame_id
        if msg.header.frame_id != 'auv_world' or msg.child_frame_id != 'tethys':
            return

        # Store initial pose
        if self.initial_pose is None:
            self.initial_pose = msg
            self.initial_position = np.array([
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z
            ])
            self.initial_orientation = np.array([
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w
            ])
            self.get_logger().info(f'Initial pose set: x={self.initial_position[0]:.3f}, '
                                 f'y={self.initial_position[1]:.3f}, '
                                 f'z={self.initial_position[2]:.3f}')
            self.get_logger().info('Publishing relative pose to /simulation/pose')
            return
        
        # Calculate relative position
        current_position = np.array([
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z
        ])
        
        current_orientation = np.array([
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w
        ])
        
        # Calculate relative position (world frame)
        relative_position = current_position - self.initial_position
        
        # Calculate relative orientation
        # Convert quaternions to rotation objects
        initial_rot = Rotation.from_quat([
            self.initial_orientation[0],
            self.initial_orientation[1],
            self.initial_orientation[2],
            self.initial_orientation[3]
        ])
        
        current_rot = Rotation.from_quat([
            current_orientation[0],
            current_orientation[1],
            current_orientation[2],
            current_orientation[3]
        ])
        
        # Relative rotation: R_relative = R_initial^-1 * R_current
        relative_rot = initial_rot.inv() * current_rot
        relative_quat = relative_rot.as_quat()  # [x, y, z, w]
        
        # transform position to initial frame
        # relative_position_body = initial_rot.inv().apply(relative_position)
        
        # Create and publish relative pose message
        relative_pose_msg = PoseStamped()
        relative_pose_msg.header.stamp = msg.header.stamp
        relative_pose_msg.header.frame_id = 'tethys_initial'
        
        relative_pose_msg.pose.position.x = relative_position[0]
        relative_pose_msg.pose.position.y = relative_position[1]
        relative_pose_msg.pose.position.z = relative_position[2]
        
        relative_pose_msg.pose.orientation.x = relative_quat[0]
        relative_pose_msg.pose.orientation.y = relative_quat[1]
        relative_pose_msg.pose.orientation.z = relative_quat[2]
        relative_pose_msg.pose.orientation.w = relative_quat[3]
        
        self.publisher.publish(relative_pose_msg)
        
        # Store trajectory data
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.trajectory_data.append([
            timestamp,
            relative_position[0],
            relative_position[1],
            relative_position[2],
            relative_quat[0],
            relative_quat[1],
            relative_quat[2],
            relative_quat[3]
        ])
    
    def save_trajectory(self):
        """Save trajectory data to text file"""
        if len(self.trajectory_data) == 0:
            self.get_logger().warn('No trajectory data to save')
            return
        
        # Generate filename with timestamp
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'/ros2_ws/src/auv_simulation_pkg/trajectories/ground_truth_trajectory_{timestamp_str}.txt'
        
        try:
            with open(filename, 'w') as f:
                # Write header
                f.write('# timestamp x y z q0 q1 q2 q3\n')
                
                # Write trajectory data
                for data in self.trajectory_data:
                    f.write(f'{data[0]:.9f} {data[1]:.6f} {data[2]:.6f} {data[3]:.6f} '
                           f'{data[4]:.6f} {data[5]:.6f} {data[6]:.6f} {data[7]:.6f}\n')
            
            self.get_logger().info(f'Trajectory saved to {filename} ({len(self.trajectory_data)} poses)')
        except Exception as e:
            self.get_logger().error(f'Failed to save trajectory: {str(e)}')
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutdown signal received, saving trajectory...')
        self.save_trajectory()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    
    node = GroundTruthPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down, saving trajectory...')
        node.save_trajectory()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()