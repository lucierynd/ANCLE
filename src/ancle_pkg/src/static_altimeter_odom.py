#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Altimeter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance


class AltimeterToOdom(Node):
    def __init__(self):
        super().__init__('altimeter_to_odom')
        
        # Parameters
        self.declare_parameter('altimeter_topic', '/altimeter')
        self.declare_parameter('odom_topic', '/altimeter_odom')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('position_covariance', 0.00001)  # Tune based on sensor accuracy
        self.declare_parameter('velocity_covariance', 0.00005)
        
        altimeter_topic = self.get_parameter('altimeter_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.pos_cov = self.get_parameter('position_covariance').value
        self.vel_cov = self.get_parameter('velocity_covariance').value
        
        # Publisher
        self.publisher = self.create_publisher(Odometry, odom_topic, 10)
        
        self.get_logger().info(f'Publishing static altitude data in {odom_topic}')

    def altimeter_callback(self, msg:  Altimeter):
        odom = Odometry()
        
        # Header
        odom.header.stamp = self.get_clock.now().to_msg()
        odom.header.frame_id = self.output_frame
        odom.child_frame_id = self.child_frame
        
        # Position (only Z is valid from altimeter)
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0 # Publishing Fake altitude data
        
        # Orientation (identity quaternion: no orientation from altimeter)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        
        # Pose covariance (6x6 matrix, row-major)
        # [x, y, z, roll, pitch, yaw]
        odom.pose.covariance = [
            1e9,  0.0,  0.0,           0.0,  0.0,  0.0,   # x
            0.0,  1e9,  0.0,           0.0,  0.0,  0.0,   # y
            0.0,  0.0,  self.pos_cov,  0.0,  0.0,  0.0,   # z
            0.0,  0.0,  0.0,           1e9,  0.0,  0.0,   # roll
            0.0,  0.0,  0.0,           0.0,  1e9,  0.0,   # pitch
            0.0,  0.0,  0.0,           0.0,  0.0,  1e9    # yaw
        ]
        
        # Velocity (only vz is valid)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        # Twist covariance
        odom.twist.covariance = [
            1e9,  0.0,  0.0,           0.0,  0.0,  0.0,   # vx
            0.0,  1e9,  0.0,           0.0,  0.0,  0.0,   # vy
            0.0,  0.0,  self.vel_cov,  0.0,  0.0,  0.0,   # vz
            0.0,  0.0,  0.0,           1e9,  0.0,  0.0,   # vroll
            0.0,  0.0,  0.0,           0.0,  1e9,  0.0,   # vpitch 
            0.0,  0.0,  0.0,           0.0,  0.0,  1e9    # vyaw 
        ]
        
        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = AltimeterToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__': 
    main()