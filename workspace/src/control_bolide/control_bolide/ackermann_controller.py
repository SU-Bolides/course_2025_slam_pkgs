import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Twist, Vector3
from bolide_interfaces.msg import ForkSpeed, SpeedDirection
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int16
import tf2_ros
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

'''This controller is used to regulate the speed and direction of the car.
It takes in a desired speed and direction, and outputs the necessary PWM signals to the motors.'''

def get_sign(val):
    return (val > 0) - (val < 0)

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

class ControllerListener(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        # Servo PWM cyclic ratio values for the direction
        # This corresponds to about 15 deg either direction. 

        self.esc_period = 20000 #us

        self.init = False

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
        self.DEVICENAME                  = '/dev/ttyUSB0'    # Symlink it in the udev to ttyU2D2

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.start_time = time.time()

        if self.portHandler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().error("Failed to open the port")

        # Setting the baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().error("Failed to change the baudrate")

        self.emergency_brake = False
        self.last_command_time = self.get_clock().now()

        # Initialise state machine 
        self.speed_controller = SpeedController(self)

        self.current_odom = Odometry()
        self.current_odom.header.frame_id = 'odom'
        self.current_odom.child_frame_id = 'base_link'

        self.current_odom.twist.covariance = [1e-4, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 1e-6, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 5e-2]
        

        self.curr_velocity_m_s = 0.0
        self.curr_steering_angle_deg = 0.0

        self.target_velocity_m_s = 0.0
        self.cmd_velocity_m_s = 0.0
        self.target_steering_angle_deg = 0.0

        self.cur_dir = 1

        self.curr_yaw = 0
        self.curr_quat = None

        self.x_pos = 0
        self.y_pos = 0

        #Subscribers and publishers
        self.create_subscription(SpeedDirection, "/cmd_vel", self.cmd_callback, 10)  # Subscribe to cmd_vel for speed and direction commands
        self.create_subscription(Float32MultiArray, "/stm32_sensors", self.stm32_callback, 10)   # Subscribe to the STM32 for the current speed and direction

        self.odom_pub = self.create_publisher(Odometry, "/ackermann_odom", 10)   # Publish the car's current state (speed and steering angle) for odometry
        self.odom_tf = tf2_ros.TransformBroadcaster(self)
        self.car_state_pub = self.create_publisher(SpeedDirection, "/car_state", 10)# Publish the car's current state (speed and steering angle) for odometry
        self.stm32_pub = self.create_publisher(Int16, "/stm32_data", 1) #  Publish the car's current state (velocity and steering angle) for odometry. 

        # Initialize watchdog timer
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)    # Check if the car is still receiving commands
        self.update = self.create_timer(0.005, self.update_callback)        # Update the car's state every 5ms (200Hz)
        self.dynamixels_comms = self.create_timer(0.03, self.dxl_callback)  # Update the dynamixels every 30ms (33Hz)

        # LED on
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, 25, 1)

        # Torque on
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, 24, 1)
        
        if self.MS:
            self.get_logger().warn("Excpeting speed in m/s")
        else:
            self.get_logger().warn("Expecting speed in [-1, 1]")
    
    def publish_stm32_data(self, cycle_ratio):
        if self.init:
            tx_data = Int16()
            tx_data.data = int(cycle_ratio*0.00938 * self.esc_period)
        
        # The previous implementation used RPi PWM which was unreliable.
            # Experiments showed the RPi to overshoot duration by 1.066. 

            # Cyclic ratio is the time of the period spent high. So 100 would be constantly high, 50 would be half high half low, etc.
            # The esc period is 20000 ns, and we send the actual pulse duration to the stm32. We also convert from RPi to "true".

        if (rclpy.is_ok()):
            self.stm32_pub.publish(tx_data)
    
    def cmd_callback(self, data):
        # Update the target speed and direction by applying a cmd_vel message

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
        self.last_command_time = self.get_clock().now()

    def dxl_callback(self):
        print("-----------------------------")
        # Update the dynamixels
        # We check if the target steering angle is within the limits of the steering angle
        self.target_steering_angle_deg = max(min(self.target_steering_angle_deg, self.MAX_STEERING_ANGLE_DEG), -self.MAX_STEERING_ANGLE_DEG)
        try:
            pos,_,_ = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, 36)   # Read the current position of the steering servo
            self.curr_steering_angle_deg = -180/3.14159*pos2psi(pos)    # Convert the position to an angle in degrees
            self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, 30, set_dir_deg(self.target_steering_angle_deg))   # Set the target position of the steering servo
        except:
            self.get_logger().warn("[WARNING] -- DYNAMIXEL PROBLEM")
            pass
    
    def stm32_callback(self, data):
        # Update the current speed and direction by applying the data from the STM32
        self.curr_yaw = data.data[0]
        self.curr_velocity_m_s = self.cur_dir * data.data[1]
    
    def update_callback(self):
        # Update the car's state

        if self.cmd_velocity_m_s != 2.0:    # If the car is not in emergency brake mode
            self.target_velocity_m_s += self.SPEED_FILTER * (self.cmd_velocity_m_s - self.target_velocity_m_s)
        else:
            self.target_velocity_m_s = 0.0
        
        # Odometry
        angular_rate = self.curr_velocity_m_s * (-1 * math.tan(self.curr_steering_angle_deg*(3.14159/180.))) / self.WHEELBASE

        self.current_odom.header.stamp = self.get_clock().now().to_msg()
        self.current_odom.twist.twist = Twist(linear = Vector3(x= self.curr_velocity_m_s, y = 0., z= 0.), angular = Vector3(x= 0., y=0.,z=angular_rate))

        self.car_state_pub.publish(SpeedDirection(speed=self.curr_velocity_m_s, direction=self.curr_steering_angle_deg))
        self.odom_pub.publish(self.current_odom)

        # Publish the odometry transform

        if (rclpy.is_ok()):
            if self.count>=3:
                self.car_state_pub.publish(SpeedDirection(self.curr_velocity_m_s, self.curr_steering_angle_deg))
                self.count = 0
            self.count += 1
            self.odom_pub.publish(self.current_odom)

        if self.cmd_velocity_m_s == 2.0:
            esc_cmd = 2.0
        else:
            esc_cmd = self.target_velocity_m_s
        
        if self.MS:
            self.speed_controller.command_pid(esc_cmd)
        else:
            self.speed_controller.command(esc_cmd)
    

    def watchdog_callback(self):
        # If it's been more than 0.5s since the last command, stop the robot
        # This is to prevent the robot from moving if the controller crashes
        if ((self.get_clock().now() - self.last_command_time).nanoseconds > 0.5*1e9) and rclpy.is_ok():
            self.cmd_velocity_m_s = 0.0
            self.speed_controller.neutral()

class SpeedController:

    def __init__(self, controller : ControllerListener):
        self.controller = controller

        self.MAX_SPEED = 9.5
        self.MIN_SPEED = 8.4

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

        self.state           = 0 #0 is Neutral, 1 is Fw, -1 is Bw, 2 is brake
        self.old_dir         = 0 

        self.block           = False    

        self.controller.publish_stm32_data(self.throttle)
    
    def neutral_transition(self):
        self.neutral()
        self.block = True
        self.controller.create_timer(0.15, self.backward, oneshot=True)
    
    def command(self, cmd_speed_m_s):
        # No PID control, just set the speed
        self.cmd_speed_esc = cmd_speed_m_s  # used only with teleop
        if self.block:
            return
        if self.cmd_speed_esc != 2.0:
            self.cmd_speed_esc = min(1,max(-1,self.cmd_speed_esc))

        # Forward
        if (1>=self.cmd_speed_esc>=1e-2):
            if self.state == -1:
                self.block = True
                self.neutral()
                self.controller.create_timer(0.25, self.forward, oneshot=True)
                return 
            self.forward(self.cmd_speed_esc)
        
        # Reverse
        elif (0.3>self.cmd_speed_esc>=-1):
            if self.state == 1 or (not self.state and self.old_dir == 1):
                self.controller.publish_stm32_data(self.REVERSEMINSPEED)
                self.block = True
                self.controller.create_timer(0.25, self.neutral_transition, oneshot=True)
                return
            self.backward(self.cmd_speed_esc)
        
        # Neutral
        elif (1e-2> self.cmd_speed_esc > -1e-2):
            self.neutral()

        # Brake
        elif (self.cmd_speed_esc == 2):
            # rospy.loginfo("brake")
            if self.state != -1:
                self.brake()
            else:
                self.neutral()

    def forward(self):
        if self.state != 1:
            self.block = False
            self.state = 1
            self.old_dir = 1
        self.controller.publish_stm32_data(self.MINSPEED + self.cmd_speed_esc*(self.MAXSPEED - self.MINSPEED))
    def backward(self):  
        if self.state != -1:
            self.block = False
            self.state = -1
            self.old_dir = -1
        self.controller.publish_stm32_data(self.REVERSEMINSPEED + self.cmd_speed_esc * (self.REVERSEMINSPEED - self.REVERSEMAXSPEED))

    def neutral(self):
        pass
        # print("N")
        self.controller.publish_stm32_data(self.NEUTRAL)

    def brake(self):
        # print("BRK")
        if self.state != 1 or (self.state and self.old_dir != 1):
            self.controller.publish_stm32(self.BRAKE)
            pass

    def command_pid(self, cmd_speed_m_s):
        # PID controller to respect a given speed

        if cmd_speed_m_s > 0 :

            e_m_s = cmd_speed_m_s - self.controller.curr_velocity_m_s # Error in m/s
            self.integral += self.dt * e_m_s    # Integral of the error
            derivative = (e_m_s - self.prev_e_m_s) / self.dt
            pid_output = self.Kp * e_m_s + self.Ki * self.integral + self.Kd * derivative
            self.prev_e_m_s = e_m_s

            rclpy.get_logger().info("PID output: {}".format(pid_output))

            pid_output = min(max(pid_output, -1), 1)
            self.throttle += pid_output
            self.throttle = max(self.MIN_SPEED, self.throttle)

            self.controller.get_logger().info("ERR / P %f", e_m_s)
            self.controller.get_logger().info("D %f", derivative)
            self.controller.get_logger().info("Throttle %f", self.throttle)
            self.controller.get_logger().info("Current speed %f", self.controller.curr_velocity_m_s)
            self.controller.get_logger().info("Target spesudo-1] Error in ackermann_controllered %f", cmd_speed_m_s)

            self.forward_speed()
        elif (not cmd_speed_m_s):
            self.neutral()
        elif cmd_speed_m_s < 0:
            self.reverse_speed()
        pass

    def forward_speed(self):
        self.controller.publish_stm32(min(self.throttle, self.MAX_SPEED))
        pass

    def reverse_speed(self):
        self.controller.publish_stm32(max(self.throttle, self.REVERSEMAXSPEED))
        pass

def main():
    try:
        rclpy.init()
        listener = ControllerListener()
        rclpy.spin(listener)
        print("Terminated ackermann_controller")
    except Exception as e:
        print(f"Error in ackermann_controller : {e}")
        pass

            