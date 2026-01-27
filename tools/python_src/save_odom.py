#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation
import signal
import sys
from datetime import datetime


class SaveOdomNode(Node):
    def __init__(self):
        super().__init__('save_odom_node')
        
        # Initialize variables
        self.trajectory_data = []
        
        # Create subscriber to the gazebo model pose topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self.pose_callback,
            10
        )
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info('Listening to odom')
    
    def pose_callback(self, msg):
        
        # Store trajectory data
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.trajectory_data.append([
            timestamp,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
    
    def save_trajectory(self):
        """Save trajectory data to text file"""
        if len(self.trajectory_data) == 0:
            self.get_logger().warn('No trajectory data to save')
            return
        
        # Generate filename with timestamp
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'/ros2_ws/src/auv_simulation_pkg/trajectories/odom_trajectory_{timestamp_str}.txt'
        
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
    
    node = SaveOdomNode()
    
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