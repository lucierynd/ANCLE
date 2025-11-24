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

        # I2C setup
        self.I2C_BUS = 7
        self.ADDRESS = 0x6B
        self.bus = SMBus(self.I2C_BUS)

        # Sensor configuration
        who_am_i = self.bus.read_byte_data(self.ADDRESS, 0x0F)
        self.get_logger().info(f"WHO_AM_I: 0x{who_am_i:02X}")
        if who_am_i != 0x6B:
            self.get_logger().warn("Unexpected device ID — check wiring or address")

        # Configure accelerometer:  833 Hz, ±2 g
        self.bus.write_byte_data(self.ADDRESS, 0x10, 0b01110000)
        # Configure gyroscope: 833 Hz, 2000 dps
        self.bus.write_byte_data(self.ADDRESS, 0x11, 0b01111100)

        # --- Conversion factors ---
        self.ACC_SENS = 0.061 / 1000.0   # g/LSB
        self.GYRO_SENS = 70 / 1000.0     # dps/LSB

        # ROS 2 publisher
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        timer_period = 0.00125  # seconds (800 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("IMU publisher node started")

    def read_xyz(self, base_addr):
        data = self.bus.read_i2c_block_data(self.ADDRESS, base_addr, 6)
        x, y, z = struct.unpack('<hhh', bytes(data))
        return x, y, z

    def timer_callback(self):
        try:
            # Read raw data
            gx, gy, gz = self.read_xyz(0x22)
            ax, ay, az = self.read_xyz(0x28)

            # Convert to physical units
            ax_g = ax * self.ACC_SENS  # g
            ay_g = ay * self.ACC_SENS 
            az_g = az * self.ACC_SENS 

            gx_rps = math.radians(gx * self.GYRO_SENS)  # dps to rad/s
            gy_rps = math.radians(gy * self.GYRO_SENS)
            gz_rps = math.radians(gz * self.GYRO_SENS)

            # Create and publish Imu message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            msg.linear_acceleration.x = ax_g
            msg.linear_acceleration.y = ay_g
            msg.linear_acceleration.z = az_g

            msg.angular_velocity.x = gx_rps
            msg.angular_velocity.y = gy_rps
            msg.angular_velocity.z = gz_rps

            self.publisher_.publish(msg)
            self.get_logger().info(
                f"Accel [m/s²]: X={ax_g:.3f}, Y={ay_g:.3f}, Z={az_g:.3f} | "
                f"Gyro [rad/s]: X={gx_rps:.3f}, Y={gy_rps:.3f}, Z={gz_rps:.3f}"
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
        node.bus.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
