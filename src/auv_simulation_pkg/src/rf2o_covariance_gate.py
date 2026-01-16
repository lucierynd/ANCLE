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

        self.declare_parameter('tilt_tolerance', 0.05)  # rad
        self.tilt_tolerance = self.get_parameter('tilt_tolerance').value

        self.roll = 0.0
        self.pitch = 0.0

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
            '/odom_rf2o',
            10
        )

        self.get_logger().info('rf2o covariance gate running')

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.roll, self.pitch, _ = self.quaternion_to_euler(
            q.x, q.y, q.z, q.w
        )

    def rf2o_callback(self, msg: Odometry):
        tilted = (
            abs(self.roll) > self.tilt_tolerance or
            abs(self.pitch) > self.tilt_tolerance
        )

        BAD = 1e6
        GOOD_POS = 0.05
        GOOD_VEL = 0.1
        GOOD_YAW = 0.05

        pose_cov = np.array(msg.pose.covariance).reshape(6, 6)
        twist_cov = np.array(msg.twist.covariance).reshape(6, 6)

        if tilted:
            # Pose
            pose_cov[0, 0] = BAD  # x
            pose_cov[1, 1] = BAD  # y
            pose_cov[5, 5] = BAD  # yaw

            # Twist
            twist_cov[0, 0] = BAD  # vx
            twist_cov[1, 1] = BAD  # vy
            twist_cov[5, 5] = BAD  # vyaw
        else:
            pose_cov[0, 0] = GOOD_POS
            pose_cov[1, 1] = GOOD_POS
            pose_cov[5, 5] = GOOD_YAW

            twist_cov[0, 0] = GOOD_VEL
            twist_cov[1, 1] = GOOD_VEL
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
