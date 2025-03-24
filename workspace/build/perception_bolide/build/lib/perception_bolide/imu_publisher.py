import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

class ImuPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        
        # SUBSCRIBER
        self.subscription = self.create_subscription(Float32MultiArray, '/stm32_sensors', self.callback, 10)
        
        # PUBLISHER
        self.pub = self.create_publisher(Imu, '/raw_imu_data', 10)
        
    def callback(self, data:Float32MultiArray):
        """ Callback function called when a message is received on the subscribed topic"""
        
        # retrieving IMU data from the STM32_sensors msg
        imu_data = Imu()
        imu_data.orientation.z = data.data[0] # not the Quaternion but the Euler_yaw
        # imu_data.linear_acceleration.x = data.data[1] # x acceleration

        # Process IMU data
        #The real angles are given in a clockwise way but in anticlockwise way in the simulation 
        imu_data.orientation.z = imu_data.orientation.z/900 # conversion in radian 
        # imu_data.linear_acceleration.x = imu_data.linear_acceleration.x/100 # conversion en m/s^2


        imu_data.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(imu_data)

def main(args=None):
    rclpy.init(args=args)
    
    # Create an Optical_Fork and start it
    imuPublisher = ImuPublisher()
    try:
        rclpy.spin(imuPublisher)
    except ExternalShutdownException:
        # If a ROSInterruptException occurs, exit the program
        exit(0)