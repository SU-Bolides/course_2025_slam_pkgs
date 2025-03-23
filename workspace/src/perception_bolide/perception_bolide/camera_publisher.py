import threading

import rclpy
import time
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import cv2
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import Bool


class CameraPublisher(Node):
    def __init__(self, w, h, fr):
        super().__init__("camera_publisher")

        # Initialize camera parameters
        self.w, self.h, self.fr = w, h, fr

        self.subscription = self.create_subscription(Bool, '/param_change_alert', self.callback, 10)
        
        self.get_enable_camera()
        # Log camera initialization
        self.get_logger().info(f"[INFO] -- Camera init. : {self.w}x{self.h}px - Frame rate : {self.fr}")
        
        # Set up camera capture
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self.camera.set(cv2.CAP_PROP_FPS, self.fr)
        time.sleep(0.1)
  
        # Log camera initialization completion
        self.get_logger().info(f"[INFO] -- Camera initialization done.")
        
        # Initialize image data for publishing
        self.image_data = SensorImage()
        self.image_data.encoding = "rgb8"
        self.image_data.is_bigendian = False
        self.image_data.height, self.image_data.width = self.h, self.w
        self.image_data.step = 3 * self.w # This is the full row length in bytes. It's 3 times the width because each pixel has three color channels (RGB).
        
        # Set up publisher for image data
        self.image_pub = self.create_publisher(SensorImage, '\raw_image_data', 1)
        
    def publish_scan(self):
        """ Publishing image array in ROS topic at rate : self.fr """
		# Log start of image publishing
        rospy.loginfo("[INFO] -- Images are published in topic : /raw_image_data")

        # Continuously capture and publish images until ROS is shutdown
        while rclpy.ok():
            if self.enable_camera :
                ret, frame = self.camera.read()
                if not ret:
                    break
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
                self.image_data.header.stamp = rospy.Time.now()
                self.image_data.data = frame.tobytes()

                # Publish the image
                self.image_pub.publish(self.image_data)

        # Log end of node
        self.get_logger().info("[INFO] -- Stopping node")
    
    def get_enable_camera(self, value = True) :
        self.enable_camera = self.get_param("/enable_camera_bool").get_parameter_value().bool_value # pas sur de cela

def main(args=None):
    rclpy.init(args=args)
    
    # Get parameters for image dimensions and frame rate
    width  = rclpy.declare_param("image_width", default=160)
    height = rclpy.declare_param("image_height", default=128)
    framerate = rclpy.declare_param("frame_rate", default=20)

    # Initialize and run CameraPublisher
    cam_pub = CameraPublisher(width, height, framerate)
    cam_pub.publish_scan()
        