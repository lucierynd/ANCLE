#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation
import signal
import sys
import argparse
from datetime import datetime


class SaveOdomNode(Node):
    def __init__(self, odom_topic):  # Accept odom_topic as parameter
        super().__init__('save_odom_node')
        
        # Initialize variables
        self.trajectory_data = []
        self.odom_topic = odom_topic  # Use the parameter
        
        # Create subscriber to the gazebo model pose topic
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.pose_callback,
            10
        )
        
        self.get_logger().info(f'Listening to odom on topic: {self.odom_topic}')
    
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
        # Clean up topic name for filename (remove slashes)
        topic_clean = self.odom_topic.replace('/', '_')
        filename = f'/ros2_ws/src/auv_simulation_pkg/trajectories/odom_trajectory{topic_clean}_{timestamp_str}.txt'
        
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

def main():
    parser = argparse.ArgumentParser(description='Save trajectory obtained from odometry topic to a .txt file')
    parser.add_argument('odom_topic', type=str, help='odometry topic')
    
    args = parser.parse_args()

    rclpy.init()
    
    node = SaveOdomNode(args.odom_topic)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Save Odom Node')
    finally:
        node.get_logger().info('Saving trajectory...')
        node.save_trajectory()
        node.get_logger().info('All good, bye !')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()