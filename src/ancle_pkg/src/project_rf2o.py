#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import numpy as np


class RF2OCovarianceGate(Node):

    def __init__(self):
        super().__init__('rf2o_covariance_gate')

        self.tilt_tolerance = 0.05
        self.pitch = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0

        # Subscribers
        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom_rf2o_raw',
            self.rf2o_callback,
            10
        )

        # Publisher
        self.rf2o_pub = self.create_publisher(
            Odometry,
            '/odom_rf2o_projected',
            10
        )

        self.get_logger().info('rf2o covariance gate running')

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        _, self.pitch, _ = self.quaternion_to_euler(
            q.x, q.y, q.z, q.w
        )

    def rf2o_callback(self, msg: Odometry):

        # check if auv is tilted
        tilted = (
            abs(self.pitch) > self.tilt_tolerance
        )

        # Project x,y pose and twist
        cos_pitch_angle = math.cos(self.pitch)

        msg.pose.pose.position.x = self.previous_x + (msg.pose.pose.position.x - self.previous_x) * cos_pitch_angle
        msg.pose.pose.position.y = self.previous_y + (msg.pose.pose.position.y - self.previous_y) * cos_pitch_angle

        msg.twist.twist.linear.x = msg.twist.twist.linear.x * cos_pitch_angle
        msg.twist.twist.linear.y = msg.twist.twist.linear.y * cos_pitch_angle

        # store previous value
        self.previous_x = msg.pose.pose.position.x
        self.previous_y = msg.pose.pose.position.y
        
        # associate covariance
        BAD = 1e6
        GOOD_POS = 0.5
        GOOD_VEL = 0.1
        GOOD_YAW = 0.5

        pose_cov = np.array(msg.pose.covariance).reshape(6, 6)
        twist_cov = np.array(msg.twist.covariance).reshape(6, 6)

        pose_cov[0, 0] = GOOD_POS
        pose_cov[1, 1] = GOOD_POS

        twist_cov[0, 0] = GOOD_VEL
        twist_cov[1, 1] = GOOD_VEL

        if tilted:
            pose_cov[5, 5] = BAD
            twist_cov[5, 5] = BAD
        else:
            pose_cov[5, 5] = GOOD_YAW
            twist_cov[5, 5] = GOOD_YAW            

        msg.pose.covariance = pose_cov.flatten().tolist()
        msg.twist.covariance = twist_cov.flatten().tolist()

        self.rf2o_pub.publish(msg)

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main():
    rclpy.init()
    node = RF2OCovarianceGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
