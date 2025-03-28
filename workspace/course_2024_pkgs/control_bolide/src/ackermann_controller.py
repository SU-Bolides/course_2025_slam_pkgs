#!/usr/bin/env python3

__author__ = "Nicolas Hammje"
__email__ = "me@nicolashammje.com"
__status__ = "In Developpement"

import rospy
from control_bolide.msg import SpeedDirection
from geometry_msgs.msg import Point, Pose, Twist, Vector3
from perception_bolide.msg import ForkSpeed
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int16
import tf
import math
from dynamixel_sdk import *


import atexit
from collections import deque
import csv
import time


### Here, we try to establish a controller that takes an input in m/s and rad (or deg)
### We also want to have an estimation of the current state of the front wheels' angle, by means
### of modelling their angle as a function where the angle changes linearly, as the servos have 
### a max speed which makes instantaneous changes in steering angle impossible. 

## We could also establish a steering angle controller that would take the yaw rate and set it to 
## a value which we woudl deduce from the desired steering angle. Those values can usually be measured
## from the IMU. 

def get_sign(x):
    return (x > 0) - (x < 0)

def pos2psi(pos):
    # Psi is the steering angle (in radians) 
    # Theta is the motor angle (in radians) 
    # Pos is the motor angle (in DXL units)
    theta_rad = (pos * (5.24/1023.)) + 0.524
    A = math.cos(theta_rad) * 15 - 4.35
    psi_rad = math.asin(A/25.)
    return(psi_rad)

def degrees2pos(degrees):
    return int((degrees - 30.) * (1023./300.))

def set_dir_deg(angle_degre) :
    psi = math.radians(angle_degre)
    A = 25. * math.sin(psi) + 4.35
    theta = math.acos(A/15.)
    pos = degrees2pos(math.degrees(theta))
    return pos



class ControllerListener:
    def __init__(self):

        self.esc_period = 20000 #us

        self.init = False


        # TO LOG FILES (for param ident)
        self.log_buffer = deque()
        self.log_file = "param_ident.csv"

        self.MAX_STEERING_ANGLE_DEG = 15.5 # deg
        self.DIR_VEL_THRESHOLD_M_S  = 0.05 # Threshold under which we can consider the car to change directions (fw <-> bw). 

        self.SPEED_FILTER    = 0.4   # Smooths out commanded speeds.
        self.WHEELBASE       = 0.257 # m See TT02 specs.

        self.MS              = False #Should velocity be -1, 1 (False), or in ms^1 (True?


        # Dynamixel stuff:
        # Protocol version
        self.PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID                      = 1                 
        self.BAUDRATE                    = 115200            
        self.DEVICENAME                  = "/dev/ttyUSB1" #str(rospy.get_param("~u2d2_topic", "/dev/ttyUSB0"))    # Symlink it in the udev to ttyU2D2


        # init propulsion pwm and direction pwm
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)


        self.start_time = time.time()


        if self.portHandler.openPort():
            rospy.loginfo("Succeeded to open the port for the U2D2")
            print("Succeeded to open the port for the U2D2")
        else:
            print("Failed to open the port for the U2D2")
            rospy.logerr("Failed to open the port for the U2D2")

        #Set the baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            rospy.loginfo("Succeeded to change the baudrate")
            print("Succeeded to change the baudrate")

        else:
            rospy.logerr("Failed to change the baudrate")


        # Node initialisation
        rospy.init_node('ackermann_controller')
        
        # emergency brake boolean
        self.emergency_brake = False

        
        self.last_command_time = rospy.get_time()

        # initialise state machine
        self.speed_controller = SpeedController(self)

        self.curr_odom = Odometry()
        self.curr_odom.header.frame_id = "odom"  # to precise that we create a frame from odom to base_link
        self.curr_odom.child_frame_id = "base_link"

        self.curr_odom.twist.covariance = [1e-4, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                           0.0, 1e-6, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                           0.0, 0.0, 0.0, 0.0, 0.0, 5e-2]

        self.curr_velocity_m_s = 0.0 
        self.curr_steering_angle_deg = 0.0

        self.count = 0


        self.target_velocity_m_s = 0.0
        self.cmd_velocity_m_s = 0.0
        self.target_steering_angle_deg = 0.0

        self.cur_dir = 1

        self.curr_yaw = 0
        self.curr_quat = None

        self.x_pos = 0
        self.y_pos = 0
        

        # rospy.Subscriber("raw_fork_data", ForkSpeed, self.speed_callback) # Subscribe to speed_callback to get the current speed from the fork
        # rospy.Subscriber("raw_imu_data", Imu, self.imu_callback) # Subscribe to speed_callback to get the current speed from the fork
        self.odom_pub = rospy.Publisher("ackermann_odom", Odometry, queue_size=1) # Publish the car's current state (velocity and steering angle) for odometry. 
        self.odom_tf =  tf.TransformBroadcaster()
        self.car_state_pub = rospy.Publisher("car_state", SpeedDirection, queue_size=1) # Publish the car's current state (velocity and steering angle) for odometry. 
        self.stm32_pub = rospy.Publisher("stm32_data", Int16, queue_size=1) # Publish the car's current state (velocity and steering angle) for odometry. 
        
        
        # Initialize watchdog timer
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.5), self.watchdog_callback)
        self.update = rospy.Timer(rospy.Duration(0.005), self.update_callback)
        self.dynamixel_comms = rospy.Timer(rospy.Duration(0.03), self.dxl_callback)


        #LED on
        if self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, 25, 1)[0]:
            rospy.logerr("Could not communicate with U2D2. You probably need to switch out the LiDAR and U2D2 ports (from /dev/ttyUBS0 to /dev/ttyUBS1) in ackermann_controller.py and rplidar_a2m12.launch")

        #Torque on
        print(self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, 24, 1))

        if self.MS:
            rospy.logwarn("EXPECTING SPEED IN m/s")
        else: 
            rospy.logwarn("EXPECTING SPEED IN [-1, 1]")

        rospy.Subscriber("cmd_vel", SpeedDirection, self.cmd_callback) # Subscribe to cmd_vel for velocities and directions command
        rospy.Subscriber("stm32_sensors", Float32MultiArray, self.stm32_callback)


        # atexit.register(self.flush_to_file)
        self.init = True


    # def log_data(self, timestamp, var1, var2, var3=None, var4=None):

    #     self.log_buffer.append((timestamp, var1,var2))


    # def flush_to_file(self):
    #     with open(self.log_file, "w", newline="") as f:
    #         writer = csv.writer(f)
    #         writer.writerow(["Timestamp", "Current speed", "Duty Cycle", "Var3", "Var4"])  # Header
    #         writer.writerows(self.log_buffer)
    #     rospy.loginfo(f"Flushed {len(self.log_buffer)} entries to {self.log_file}")






    
    def publish_stm32_data(self, cyclic_ratio):
        if self.init:
            tx_data = Int16()
            tx_data.data = int(cyclic_ratio*0.00938 * self.esc_period) 

            # The previous implementation used RPi PWM which was unreliable.
            # Experiments showed the RPi to overshoot duration by 1.066. 

            # Cyclic ratio is the time of the period spent high. So 100 would be constantly high, 50 would be half high half low, etc.
            # The esc period is 20000 ns, and we send the actual pulse duration to the stm32. We also convert from RPi to "true".

            if (not rospy.is_shutdown()):
                self.stm32_pub.publish(tx_data)

        

    def cmd_callback(self, data):  
        # Apply a cmd_vel, in m/s and deg 

        self.cmd_velocity_m_s = data.speed

        if self.MS:
            self.target_steering_angle_deg = -data.direction
        else:
            self.target_steering_angle_deg = -data.direction * self.MAX_STEERING_ANGLE_DEG
        
        #Speed is not signed. We assume the car to only move under its own power, so we know the sign by looking at the commands.

        if (not (get_sign(data.speed) == self.cur_dir)):
            if (not self.cur_dir) or (abs(self.curr_velocity_m_s) < self.DIR_VEL_THRESHOLD_M_S): 

                #We can command reverse while the car is going forwards
                #However, the car needs to stop and then start reversing before it actually goes in reverse.
                #So we only switch the sign if the car is going slow enough to be considered "stationnary".
                self.cur_dir = get_sign(data.speed)
                # rospy.loginfo("Switched direction (speed sign): %s", str(self.cur_dir))

        # Update the last command time
        self.last_command_time = rospy.get_time()

        
    def dxl_callback(self, _):
        self.target_steering_angle_deg = max(min(self.target_steering_angle_deg, self.MAX_STEERING_ANGLE_DEG), -self.MAX_STEERING_ANGLE_DEG) 
        try:
            pos, _ , _ = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, 36)
            self.curr_steering_angle_deg= -180/3.14159 * pos2psi(pos)
            self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, 30, set_dir_deg(self.target_steering_angle_deg))
        except:
            rospy.logwarn("DYNAMIXEL PROBLEM")
            pass


    def stm32_callback(self, data):
        self.curr_yaw = data.data[0]
        self.curr_velocity_m_s = self.cur_dir * data.data[1]
    
    def update_callback(self, _):

        # self.log_data(time.time() - self.start_time, self.curr_velocity_m_s, self.cmd_velocity_m_s)


        if self.cmd_velocity_m_s != 2.0:
            self.target_velocity_m_s += self.SPEED_FILTER * (self.cmd_velocity_m_s - self.target_velocity_m_s)
        else: 
            self.target_velocity_m_s = 0.0

        #Odom
        angular_rate =  self.curr_velocity_m_s * (-1 * math.tan(self.curr_steering_angle_deg*(3.1415926535/180.))) / self.WHEELBASE
        # self.x_pos += math.cos(self.curr_yaw) * self.curr_velocity_m_s * 0.01 #Updated at 10ms 
        # self.y_pos += math.sin(self.curr_yaw) * self.curr_velocity_m_s * 0.01 #Updated at 10ms



        self.curr_odom.header.stamp = rospy.Time.now()
        # if self.curr_quat:
        #     self.curr_odom.pose.pose = Pose(Point(self.x_pos,self.y_pos,0.0),self.curr_quat)
        #     tf_quat = [self.curr_quat.x, self.curr_quat.y, self.curr_quat.z, self.curr_quat.w]
        #     self.odom_tf.sendTransform((self.x_pos,self.y_pos,0.0),tf_quat,self.curr_odom.header.stamp,"base_link","odom") # send the transformation from odom to base_link
    
        self.curr_odom.twist.twist = Twist(Vector3(self.curr_velocity_m_s,0.0,0.0),Vector3(0.0,0.0,angular_rate))


        if (not rospy.is_shutdown()):

            if self.count>=3:
                self.car_state_pub.publish(SpeedDirection(self.curr_velocity_m_s, self.curr_steering_angle_deg))
                self.count = 0
            self.count += 1
            self.odom_pub.publish(self.curr_odom)

        if self.cmd_velocity_m_s == 2.0:
            esc_cmd = 2.0
        else:
            esc_cmd = self.target_velocity_m_s

        #Change direction
        # self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, 30, set_dir_deg(self.target_steering_angle_deg))

        if self.MS:
            self.speed_controller.command_pid(esc_cmd)
        else:
            self.speed_controller.command(esc_cmd)
        

    def watchdog_callback(self, _):
        # If it's been more than 0.5 seconds since the last command, stop the robot
        # This is to prevent the robot from moving if the controller or computer crashes or disconnects
        if ((rospy.get_time() - self.last_command_time) > 0.5) and (not self.emergency_brake) and not rospy.is_shutdown():
            # print("Watchdog callback neutraled. (ooga booga)")
            self.cmd_velocity_m_s = 0.0
            self.speed_controller.neutral()

class SpeedController:

    def __init__(self, controller : ControllerListener):
        self.ctrl = controller

        self.MAXSPEED        = 10.0 #Calibrated ESC to 10 max, 6.5 min
        self.MINSPEED        = 8.4
        self.NEUTRAL         = 8.0  
        self.REVERSEMINSPEED = 7.6
        self.REVERSEMAXSPEED = 6.5
        self.BRAKE           = 5.0

        self.BRAKE_THRESHOLD = 0.5
        self.VMAX_M_S        = 5    #m/s



        self.Kp              = 2e-2
        self.Kd              = 0
        self.Ki              = 0.0
        
        self.prev_e_m_s      = 0.0
        self.integral        = 0.0

        self.dt              = 0.01

        self.throttle        = self.NEUTRAL
        self.cmd_speed_esc   = 0

        self.state           = -1 #0 is Neutral, 1 is Fw, -1 is Bw, 2 is brake
        self.old_dir         = 0 

        self.block           = False

        self.ctrl.publish_stm32_data(self.throttle)

    

    def neutral_transition(self, _):
        self.neutral()
        self.block = True
        rospy.Timer(rospy.Duration(0.15), self.backward, oneshot=True)


    def command(self, cmd_speed_m_s):
        #No PID controller just approximate mappings
        self.cmd_speed_esc = cmd_speed_m_s #We assume this to only be used when using teleop
        if self.block:
            return
        if self.cmd_speed_esc != 2.0:
            self.cmd_speed_esc = min(1, max(-1, self.cmd_speed_esc))

        #Forwards
        if (1>=self.cmd_speed_esc>1e-2):
            if self.state == -1: 
                self.block = True
                self.neutral()
                rospy.Timer(rospy.Duration(0.25), self.forward, oneshot=True)
                return
            self.forward(self.cmd_speed_esc)

        #Neutral
        elif (1e-2> self.cmd_speed_esc > -1e-2):
            self.neutral()

        #Reverse
        elif (0.3>self.cmd_speed_esc>=-1):
            if self.state == 1 or (not self.state and self.old_dir == 1):
                self.ctrl.publish_stm32_data(self.REVERSEMINSPEED)
                self.block = True
                rospy.Timer(rospy.Duration(0.25), self.neutral_transition, oneshot=True)
                return
            self.backward(self.cmd_speed_esc)

        #Brake
        elif (self.cmd_speed_esc == 2):
            if self.state != -1:
                self.brake()
            else:
                self.neutral()

    def forward(self, _):
        if self.state != 1: 
            self.block = False
            self.state = 1
            self.old_dir = 1
        self.ctrl.publish_stm32_data(self.MINSPEED + self.cmd_speed_esc*(self.MAXSPEED - self.MINSPEED))

    def backward(self, _):
        if self.state != -1:
            self.block = False
            self.state = -1
            self.old_dir = -1
        self.ctrl.publish_stm32_data(self.REVERSEMINSPEED + self.cmd_speed_esc*(self.REVERSEMINSPEED - self.REVERSEMAXSPEED))
    
    
    def neutral(self):
        # print("N")
        self.ctrl.publish_stm32_data(self.NEUTRAL)

    def brake(self):
        # print("BRK")
        if self.state != 1 or (self.state and self.old_dir != 1):
            self.ctrl.publish_stm32_data(self.BRAKE)


    def command_pid(self, cmd_speed_m_s):
        #PID Controller to respect a given speed. 

        #WORK IN PROGRESS

        if cmd_speed_m_s>0:

            e_m_s = cmd_speed_m_s - self.ctrl.curr_velocity_m_s
            self.integral += self.dt * e_m_s
            derivative = (e_m_s - self.prev_e_m_s) / self.dt
            pid_output = self.Kp * e_m_s  + self.integral * self.Ki + derivative * self.Kd
            self.prev_e_m_s = e_m_s

            rospy.loginfo("PID %f", pid_output)
            
            pid_output = min(max(pid_output,-1.), 1.)
            self.throttle += pid_output
            self.throttle = max(self.MINSPEED, self.throttle)

            rospy.loginfo("ERR / P %f", e_m_s)
            rospy.loginfo("D %f", derivative)
            rospy.loginfo("THROTTLE %f", self.throttle)
            rospy.loginfo("curr speed %f", self.ctrl.curr_velocity_m_s)
            rospy.loginfo("cmd speed %f \n", cmd_speed_m_s)

        if cmd_speed_m_s>0:
            self.forward_speed()
        elif (not cmd_speed_m_s):
            self.neutral()
        elif cmd_speed_m_s < 0:
            self.reverse_speed()
        pass

    def forward_speed(self):
        self.ctrl.publish_stm32_data(min(self.throttle, self.MAXSPEED))
        pass
    

    def reverse_speed(self):
        #Not implemented yet
        pass

if __name__ == '__main__':
    listener = ControllerListener()
    rospy.spin()
    print("Terminated ackermann_controller")
