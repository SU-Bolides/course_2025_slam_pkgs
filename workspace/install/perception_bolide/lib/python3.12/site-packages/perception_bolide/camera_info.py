#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

# Import ROS 2 message types
from bolide_interfaces.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def rgb2hsv(rgb: tuple) -> tuple:
    """Convert the RGB color to HSV."""
    return cv2.cvtColor(np.uint8([[rgb]]), cv2.COLOR_RGB2HSV)[0][0]


def hsv_dist(hsv1: tuple, hsv2: tuple) -> float:
    """Return the distance between two HSV colors (between 0 and 1)."""
    coef_H, coef_S, coef_V = 0.7, 0.2, 0.1

    loss_H = min(abs(hsv1[0] - hsv2[0]) % 180, abs(hsv2[0] - hsv1[0]) % 180) / 180
    loss_S = abs(hsv1[1] - hsv2[1]) / 255
    loss_V = abs(hsv1[2] - hsv2[2]) / 255

    return coef_H * loss_H + coef_S * loss_S + coef_V * loss_V


class CameraInfoNode(Node):
    def __init__(self):
        super().__init__("camera_info")

        self.get_logger().info("Initializing the camera_info node")

        # Subscribers
        self.create_subscription(Image, "/raw_image_data", self.callback_image, 10)
        self.create_subscription(Bool, "/param_change_alert", self.get_ros_params, 10)

        # Publisher
        self.pub = self.create_publisher(CameraInfo, "/camera_info", 10)

        self.get_ros_params()

        # Internal attributes
        self.image_matrix = None
        self.wrong_way = False

        self.mid_range = 3
        self.side_range = 3

    def callback_image(self, image_data: Image):
        """Callback function for the image data subscriber."""
        self.image_matrix = np.frombuffer(image_data.data, dtype=np.uint8).reshape(
            image_data.height, image_data.width, 3
        )

        middle_color, left_color, right_color = self.middle_and_side_colors()
        self.is_wrong_way(left_color, right_color)

        # Publish CameraInfo message
        msg = CameraInfo()
        msg.wrong_way = self.wrong_way
        msg.front_color = middle_color
        msg.left_color = left_color
        msg.right_color = right_color

        self.pub.publish(msg)

    def nearest_color(self, pixel: np.ndarray) -> str:
        """Return the nearest color of the pixel if within tolerance threshold, else 'unknown'."""
        hsv_pixel = rgb2hsv(pixel)
        differences = {color: hsv_dist(hsv_pixel, self.HSV_COLORS[color]) for color in self.HSV_COLORS}

        min_color, min_dist = min(differences.items(), key=lambda x: x[1])
        return min_color if min_dist < self.tolerance else "unknown"

    def is_wrong_way(self, left_color: str, right_color: str):
        """Determine if the robot is going the wrong way."""
        if left_color == "green" and right_color == "red":
            self.wrong_way = not self.green_is_left
        elif left_color == "red" and right_color == "green":
            self.wrong_way = self.green_is_left

    def middle_and_side_colors(self) -> tuple:
        """Return the color of the middle and side pixels of the image."""
        if self.image_matrix is None:
            return "white", "white", "white"

        mid_h, mid_w = self.image_matrix.shape[0] // 2, self.image_matrix.shape[1] // 2

        middle_pixel = np.mean(self.image_matrix[mid_h - self.mid_range : mid_h + self.mid_range,
                                                 mid_w - self.mid_range : mid_w + self.mid_range], axis=(0, 1))

        left_pixel = np.mean(self.image_matrix[mid_h - self.side_range : mid_h + self.side_range, : self.side_range], axis=(0, 1))

        right_pixel = np.mean(self.image_matrix[mid_h - self.side_range : mid_h + self.side_range, -self.side_range:], axis=(0, 1))

        return self.nearest_color(middle_pixel), self.nearest_color(left_pixel), self.nearest_color(right_pixel)

    def get_ros_params(self, msg=None):
        """Update parameters when receiving an alert or at startup."""
        self.RGB_COLORS = {
            "red": self.get_parameter_or("/red_RGB", [255, 0, 0]),
            "green": self.get_parameter_or("/green_RGB", [0, 255, 0]),
        }
        self.HSV_COLORS = {color: rgb2hsv(tuple(self.RGB_COLORS[color])) for color in self.RGB_COLORS}

        self.tolerance = self.get_parameter_or("/color_detection_tolerance", 0.8)
        self.green_is_left = self.get_parameter_or("/green_is_left", True)

    def get_parameter_or(self, name, default):
        """Helper function to get a parameter or return a default value."""
        param = self.get_parameter(name)
        return param.value if param.type_ != rclpy.Parameter.Type.NOT_SET else default


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
