#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Clément MIZZI and Loris OUMBICHE"
__status__ = "Tested"


import numpy as np
import rospy
import sys
import tf
from sensor_msgs.msg import Imu
from perception_bolide.msg import ForkSpeed
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from control_bolide.msg import SpeedDirection



# Odometry is based on the ackermann steering geometry.


class OdomForkImu:
    def __init__(self):
        self.pub = rospy.Publisher('odom',Odometry,queue_size=10)
        self.Odom = Odometry()
        self.Odom_broadcaster = tf.TransformBroadcaster() # TransformBroadcaster is used to send a transformation between odom and base_link
        self.L = 0.257 # Distance between location point and wheel point (back and front wheels) (m)
        self.fork = 0 # speed from the fork (m/s)
        self.forksign = 1.0
        self.lastdir = 1.0

        self.current_time = rospy.Time.now() # current_time and last_time are used to compute dt
        self.last_time = rospy.Time.now()

        self.gps_x = None
        self.gps_y = None

        self.x_pos = 0
        self.y_pos = 0
        self.theta_pos = 0
        self.dx = 0
        self.dy = 0
        self.dtheta =0

    def compute_position(self):   # the reference is the center of the back wheels
        self.current_time = rospy.Time.now()

        # Linear update
        self.dx = self.fork*self.lastdir*np.cos(self.theta_pos)
        self.dy = self.fork*self.lastdir*np.sin(self.theta_pos)


        self.last_time = self.current_time

    def update(self):
        # Odom position
        Odom_quat = tf.transformations.quaternion_from_euler(0,0,self.theta_pos) # Euler to Quaternion

        position = [self.gps_x, self.gps_y]

        if position[0] is None:
            return
        self.Odom.pose.pose = Pose(Point(position[0],position[1],0.0),Quaternion(*Odom_quat))

        # Odom speed
        self.Odom.twist.twist = Twist(Vector3(self.dx,self.dy,0.0),Vector3(0.0,0.0,self.dtheta))

        # Transform
        self.Odom_broadcaster.sendTransform((position[0],position[1],0.0),Odom_quat,self.current_time,"base_link","odom") # send the transformation from odom to base_link
        self.Odom.header.stamp = self.current_time
        self.Odom.header.frame_id = "odom"  # to precise that we create a frame from odom to base_link
        self.Odom.child_frame_id = "base_link"

        # Publish Topic
        self.pub.publish(self.Odom)
    
    def get_gps(self, gps:Odometry):
        self.gps_x = gps.pose.pose.position.x
        self.gps_y = gps.pose.pose.position.y


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
        s.compute_position()
        s.update()

def listener(s:OdomForkImu):
    rospy.Subscriber('raw_fork_data',ForkSpeed,s.get_fork)
    rospy.Subscriber('cmd_vel',SpeedDirection,s.get_speedsign)
    rospy.Subscriber('raw_imu_data',Imu,s.get_dir)
    rospy.Subscriber('/gps/odom',Odometry,s.get_gps)

    rospy.spin()   

if __name__ == '__main__' :
    rospy.init_node('odom_fork_imu')
    s = OdomForkImu()
    try : 
        listener(s)
    except rospy.ROSInterruptException:
        exit(0)
