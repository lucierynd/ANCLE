#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import struct
import time
import math


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # ROS 2 publisher
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        timer_period = 0.00125  # seconds (800 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("IMU publisher node started")

    def timer_callback(self):
        try:
            # Create and publish Imu message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = 1.0

            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0

            self.publisher_.publish(msg)
            self.get_logger().info(
                f"Accel [m/s²]: X={msg.linear_acceleration.x:.3f}, Y={msg.linear_acceleration.y:.3f}, Z={msg.linear_acceleration.z:.3f} | "
                f"Gyro [rad/s]: X={msg.angular_velocity.x:.3f}, Y={msg.angular_velocity.y:.3f}, Z={msg.angular_velocity.z:.3f}"
            )

        except Exception as e:
            self.get_logger().error(f"Read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU publisher stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
