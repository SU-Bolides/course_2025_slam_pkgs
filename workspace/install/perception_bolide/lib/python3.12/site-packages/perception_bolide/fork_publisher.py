import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from bolide_interfaces.msg import ForkSpeed
from std_msgs.msg import Float32MultiArray

class Optical_Fork(Node):
    def __init__(self):
        super().__init__("fork_publisher")

        # SUBSCRIBER
        self.subscription = self.create_subscription(Float32MultiArray, '/stm32_sensors', self.callback, 10)
        # PUBLISHER
        self.pub = self.create_publisher(ForkSpeed, '/raw_fork_data', 10)
  
    def callback(self, data:Float32MultiArray):
        """ Callback function called when a message is received on the subscribed topic"""

        # retrieving Fork data from the STM32_sensors msg
        fork_data = ForkSpeed()
        fork_data.speed = data.data[1] # speed of the car in mm/s

        # Process fork data
        fork_data.speed = fork_data.speed/1000 # passage de mm/s à m/s

        fork_data.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(fork_data)

def main(args=None):
    rclpy.init(args=args)

    fork_data = Optical_Fork()
    try:
        rclpy.spin(fork_data)
    except ExternalShutdownException:
        # If a ROSInterruptException occurs, exit the program
        exit(0)
