#!/usr/bin/env python3

import rclpy
import yaml
import cv2
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, CompressedImage


class PeCalibrationPublisher(Node):
    def __init__(self):
        super().__init__('pe_calibration_publisher')
        self._filename = self.declare_parameter('file', '').value
        self._input_topic = self.declare_parameter('input_topic', '/amplitude_image').value
        self._use_raw = self.declare_parameter('use_raw', True).value
        self.__alpha = self.declare_parameter('alpha', 0).value

        if self._filename == '':
            self.get_logger().error("Empty YAML file. Terminating")
            return

        self.__camera_info_msg = self.__parse_yaml(self._filename)
        self.info_publisher_ = self.create_publisher(CameraInfo, 'camera_info', 10)

        # Create QoS profile matching the camera publisher 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        if self._use_raw:
            self.image_sub_ = self.create_subscription(
                Image,
                self._input_topic,
                self.image_callback,
                qos_profile)  
        else:
            self.image_sub_ = self.create_subscription(
                CompressedImage,
                self._input_topic+'/compressed',
                self.image_callback,
                qos_profile)  

        self.get_logger().info('Loaded file: {}'.format(self._filename))
        self.counter = 0

    def image_callback(self, msg):
        self.__camera_info_msg.header = msg.header
        self.info_publisher_.publish(self.__camera_info_msg)

    def __parse_yaml(self, yaml_fname):
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.safe_load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.k = calib_data["camera_matrix"]["data"]
        camera_info_msg.d = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.r = calib_data["rectification_matrix"]["data"]
        if "projection_matrix" in calib_data:
            camera_info_msg.p = calib_data["projection_matrix"]["data"]
        else:
            P = np.zeros((3, 4), dtype=np.float64)
            intrinsics = np.array(camera_info_msg.k, dtype=np.float64, copy=True).reshape((3, 3))
            distortion = np.array(camera_info_msg.d, dtype=np.float64, copy=True).reshape((len(camera_info_msg.d), 1))
            self.get_logger().info("Missing Projection Matrix - Pinhole - Calculating ...")
            ncm, _ = cv2.getOptimalNewCameraMatrix(intrinsics,
                                                   distortion,
                                                   (camera_info_msg.width, camera_info_msg.height),
                                                   self.__alpha)
            for j in range(3):
                for i in range(3):
                    P[j, i] = ncm[j, i]

            camera_info_msg.p = P.flatten().tolist()

        if "camera_model" in calib_data:
            camera_info_msg.distortion_model = calib_data["camera_model"]
        elif "distortion_model" in calib_data:
            camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg


def main(args=None):
    rclpy.init(args=args)

    calibration_publisher = PeCalibrationPublisher()

    rclpy.spin(calibration_publisher)

    calibration_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()