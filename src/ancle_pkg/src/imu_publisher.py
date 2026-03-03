#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import struct
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

        # Configure accelerometer: 833 Hz, ±2 g
        self.bus.write_byte_data(self.ADDRESS, 0x10, 0b01110000)
        # Configure gyroscope: 833 Hz, 2000 dps
        self.bus.write_byte_data(self.ADDRESS, 0x11, 0b01111100)

        # Conversion factors
        self.ACC_SENS = 0.061 / 1000.0   # g/LSB
        self.GYRO_SENS = 70 / 1000.0     # dps/LSB

        # Covariance values
        self.ORIENTATION_COV = 0.1
        self.ORIENTATION_COV_YAW = 1.0
        self.GYRO_COV = 0.5
        self.ACCEL_COV = 0.5

        # Complementary filter state
        self.alpha = 0.98  # weight for gyroscope integration (0.0–1.0)
        self.roll = 0.0    # radians
        self.pitch = 0.0   # radians
        self.yaw = 0.0     # radians (will drift bc no magnetometer correction)
        self.last_time = None

        # ROS 2 publisher
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        timer_period = 0.00125  # seconds (800 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("IMU publisher node started")

    def read_xyz(self, base_addr):
        data = self.bus.read_i2c_block_data(self.ADDRESS, base_addr, 6)
        x, y, z = struct.unpack('<hhh', bytes(data))
        return x, y, z

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles (ZYX convention) to quaternion (x, y, z, w)."""
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w

    def timer_callback(self):
        try:
            now = self.get_clock().now()
            current_time = now.nanoseconds * 1e-9  # seconds

            # Compute dt
            if self.last_time is None:
                self.last_time = current_time
                return  # no dt yet
            dt = current_time - self.last_time
            self.last_time = current_time

            if dt <= 0.0 or dt > 0.5:
                return  # skip bogus intervals

            # Read raw data
            gx, gy, gz = self.read_xyz(0x22)
            ax, ay, az = self.read_xyz(0x28)

            # Convert to physical units
            ax_ms2 = ax * self.ACC_SENS * 9.81  # m/s²
            ay_ms2 = ay * self.ACC_SENS * 9.81
            az_ms2 = az * self.ACC_SENS * 9.81

            gx_rps = math.radians(gx * self.GYRO_SENS)  # rad/s
            gy_rps = math.radians(gy * self.GYRO_SENS)
            gz_rps = math.radians(gz * self.GYRO_SENS)

            # Complementary filter to output orientation

            # Roll & pitch from accelerometer 
            accel_roll = math.atan2(ay_ms2, az_ms2)
            accel_pitch = math.atan2(
                -ax_ms2,
                math.sqrt(ay_ms2 * ay_ms2 + az_ms2 * az_ms2)
            )

            # Integrate gyroscope
            gyro_roll = self.roll + gx_rps * dt
            gyro_pitch = self.pitch + gy_rps * dt
            gyro_yaw = self.yaw + gz_rps * dt

            # Fuse: trust gyro short-term, accel long-term
            self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll
            self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch
            self.yaw = gyro_yaw  # no magnetometer → gyro-only (will drift)

            # Convert to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(
                self.roll, self.pitch, self.yaw
            )

            # Build Imu message
            msg = Imu()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'imu_link'

            # Orientation
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            msg.orientation_covariance[0] = self.ORIENTATION_COV
            msg.orientation_covariance[4] = self.ORIENTATION_COV
            msg.orientation_covariance[8] = self.ORIENTATION_COV_YAW

            # Angular velocity
            msg.angular_velocity.x = gx_rps
            msg.angular_velocity.y = gy_rps
            msg.angular_velocity.z = gz_rps

            msg.angular_velocity_covariance[0] = self.GYRO_COV
            msg.angular_velocity_covariance[4] = self.GYRO_COV
            msg.angular_velocity_covariance[8] = self.GYRO_COV

            # Linear acceleration
            msg.linear_acceleration.x = ax_ms2
            msg.linear_acceleration.y = ay_ms2
            msg.linear_acceleration.z = az_ms2

            msg.linear_acceleration_covariance[0] = self.ACCEL_COV
            msg.linear_acceleration_covariance[4] = self.ACCEL_COV
            msg.linear_acceleration_covariance[8] = self.ACCEL_COV

            self.publisher_.publish(msg)

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