import numpy as np
import sys
import tf

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from bolide_interfaces.msg import ForkSpeed, SpeedDirection

# Odometry is based on the ackermann steering geometry.

class OdomForkImu(Node):
    def __init__(self):
        super().__init__('odom_fork_imu')

        #SUBSCRIBER
        self.sub_raw_fork_data = self.create_subscription(ForkSpeed, '/raw_fork_data', get_fork)
        self.sub_vel = self.create_subscription(SpeedDirection, '/cmd_vel', get_speed)
        self.sub_raw_imu_data = self.create_subscription(Imu, '/raw_imu_data', get_imu)
        # PUBLISHER
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        self.Odom = Odometry()
        self.Odom_broadcaster = tf.TransformBroadcaster() # TransformBroadcaster is used to send a transformation between odom and base_link
        self.L = 0.257 # Distance between location point and wheel point (back and front wheels) (m)
        self.fork = 0 # speed from the fork (m/s)
        self.forksign = 1.0
        self.lastdir = 1.0

        self.current_time = self.get_clock().now().to_msg() # current_time and last_time are used to compute dt
        self.last_time = self.get_clock().now().to_msg()

        self.x_pos = 0
        self.y_pos = 0
        self.theta_pos = 0
        self.dx = 0
        self.dy = 0
        self.dtheta =0

    def compute_position(self):   # the reference is the center of the back wheels
        self.current_time = self.get_clock().now().to_msg()
        dt = (self.current_time - self.last_time).to_sec()

        # Linear update
        self.dx = self.fork*self.lastdir*np.cos(self.theta_pos)
        self.dy = self.fork*self.lastdir*np.sin(self.theta_pos)
        self.x_pos += dt*self.dx
        self.y_pos += dt*self.dy

        self.last_time = self.current_time

    def update(self):
        # Odom position
        Odom_quat = tf.transformations.quaternion_from_euler(0,0,self.theta_pos) # Euler to Quaternion
        self.Odom.pose.pose = Pose(Point(self.x_pos,self.y_pos,0.0),Quaternion(*Odom_quat))

        # Odom speed
        self.Odom.twist.twist = Twist(Vector3(self.dx,self.dy,0.0),Vector3(0.0,0.0,self.dtheta))

        # Transform
        self.Odom_broadcaster.sendTransform((self.x_pos,self.y_pos,0.0),Odom_quat,self.current_time,"base_link","odom") # send the transformation from odom to base_link
        self.Odom.header.stamp = self.current_time
        self.Odom.header.frame_id = "odom"  # to precise that we create a frame from odom to base_link
        self.Odom.child_frame_id = "base_link"

        # Publish Topic
        self.pub.publish(self.Odom)

    def get_fork(self,msg:ForkSpeed):
        self.fork = msg.speed

    def get_speedsign(self,msg:SpeedDirection):
        sign = int(round((msg.speed / (abs(msg.speed)+1e-6))))
        if abs(sign) != 1:
            pass
        else:
            self.lastdir = sign


    def get_dir(self,msg:Imu):
        explicit_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.theta_pos = tf.transformations.euler_from_quaternion(explicit_quat)[2]
        print(self.theta_pos)
        self.compute_position()
        self.update()

def main(args=None):
    rclpy.init(args=args)

    s = OdomForkImu()
    try:
        rclpy.spin(s)
    except ExternalShutdownException:
        exit(0)  
