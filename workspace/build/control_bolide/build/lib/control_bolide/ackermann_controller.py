import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Twist, Vector3
from bolide_interfaces.msg import ForkSpeed, SpeedDirection
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import tf_transformations
import math
from dynamixel_sdk import *
from rpi_hardware_pwm import HardwarePWM

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
    def __init__(self, pwm_prop):
        super().__init__('ackermann_controller')
        # Servo PWM cyclic ratio values for the direction
        # This corresponds to about 15 deg either direction. 
        self.CENTER          = 8.7
        self.LEFT            = 7.2
        self.RIGHT           = 10.2

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
        self.DEVICENAME                  = '/dev/ttyUSB1'    # Symlink it in the udev to ttyU2D2


        # init propulsion pwm and direction pwm
        self.pwm_prop = pwm_prop

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.get_logger.info("Succeeded to open the port")

        if self.portHandler.openPort():
            self.get_logger.info("Succeeded to open the port")
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

        self.curr_odom.twist.covariance = [1e-4, 0.0, 0.0, 0.0, 0.0, 0.0,
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
        self.odom_tf = tf_transformations.TransformBroadcaster(self)
        self.car_state_pub = self.create_publisher(SpeedDirection, "/car_state", 10)# Publish the car's current state (speed and steering angle) for odometry
        
        # Initialize watchdog timer
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)    # Check if the car is still receiving commands
        self.update = self.create_timer(0.005, self.update_callback)        # Update the car's state every 5ms (200Hz)
        self.dynamixels_comms = self.create_timer(0.03, self.dxl_callback)  # Update the dynamixels every 30ms (33Hz)

        # LED on
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, 25, 1)

        # Torque on
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, 24, 1)
        
        if self.MS:
            self.get_logger().info("Excpeting speed in m/s")
        else:
            self.get_logger().info("Expecting speed in [-1, 1]")
    
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

    def dxl_callback(self,_):
        # Update the dynamixels
        # We check if the target steering angle is within the limits of the steering angle
        self.target_steering_angle_deg = max(min(self.target_steering_angle_deg, self.MAX_STEERING_ANGLE_DEG), -self.MAX_STEERING_ANGLE_DEG)
        try:
            pos,_,_ = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, 36)   # Read the current position of the steering servo
            self.curr_steering_angle_deg = -180/3.14159*pos2psi(pos)    # Convert the position to an angle in degrees
            self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, 30, set_dir_deg(self.target_steering_angle_deg))   # Set the target position of the steering servo
        except:
            pass
    
    def stm32_callback(self, data):
        # Update the current speed and direction by applying the data from the STM32
        self.curr_yaw = data.data[0]
        self.curr_velocity_m_s = self.cur_dir * data.data[1]
    
    def update_callback(self, _):
        # Update the car's state

        if self.cmd_velocity_m_s != 2.0:    # If the car is not in emergency brake mode
            self.target_velocity_m_s += self.self.SPEED_FILTER * (self.cmd_velocity_m_s - self.target_velocity_m_s)
        else:
            self.target_velocity_m_s = 0.0
        
        # Odometry
        angular_rate = self.curr_velocity_m_s * (-1 * math.tan(self.curr_steering_angle_deg*(3.14159/180.))) / self.WHEELBASE

        self.curr_odom.header.stamp = self.get_clock().now().to_msg()
        self.curr_odom.twist.twist = Twist(Vector3(self.curr_velocity_m_s, 0, 0), Vector3(0, 0, angular_rate))

        self.car_state_pub.publish(SpeedDirection(speed=self.curr_velocity_m_s, direction=self.curr_steering_angle_deg))
        self.odom_pub.publish(self.curr_odom)

        # Publish the odometry transform
        if self.cmd_velocity_m_s == 2.0:
            esc_cmd = 2.0
        else:
            esc_cmd = self.target_velocity_m_s
        
        if self.MS:
            self.speed_controller.command_pid(esc_cmd)
        else:
            self.speed_controller.command(esc_cmd)
    

    def watchdog_callback(self, _):
        # If it's been more than 0.5s since the last command, stop the robot
        # This is to prevent the robot from moving if the controller crashes
        if ((self.get_clock().now() - self.last_command_time).seconds > 0.5):
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

        self.ctrl.pwm_prop.start(self.throttle)
    
    def neutral_transition(self,_):
        self.neutral()
        self.block = True
        self.create_timer(0.15, self.backward, oneshot=True)
    
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
                self.create_timer(0.15, self.forward, oneshot=True)
                return 
            self.forward(self.cmd_speed_esc)
        
        # Reverse
        elif (-1e-2>self.cmd_speed_esc>=-1):
            if self.state == 1 or (not self.state and self.old_dir == 1):
                self.ctrl.pwm_prop.change_duty_cycle(self.REVERSEMINSPEED)
                self.block = True
                self.create_timer(0.15, self.neutral_transition, oneshot=True)
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

    def forward(self,_):
        if self.state != 1:
            self.block = False
            self.state = 1
            self.old_dir = 1
        self.ctrl.pwm_prop.change_duty_cycle(self.MIN_SPEED + self.cmd_speed_esc * (self.MAX_SPEED - self.MIN_SPEED))

    def backward(self,_):  
        if self.state != -1:
            self.block = False
            self.state = -1
            self.old_dir = -1
        self.ctrl.pwm_prop.change_duty_cycle(self.REVERSEMINSPEED + self.cmd_speed_esc * (self.REVERSEMINSPEED - self.REVERSEMAXSPEED))

    def neutral(self):
        # print("N")
        self.ctrl.pwm_prop.change_duty_cycle(self.NEUTRAL)

    def brake(self):
        # print("BRK")
        if self.state != 1 or (self.state and self.old_dir != 1):
            self.ctrl.pwm_prop.change_duty_cycle(self.BRAKE)

    def command_pid(self, cmd_speed_m_s):
        # PID controller to respect a given speed

        if cmd_speed_m_s > 0 :

            e_m_s = cmd_speed_m_s - self.ctrl.curr_velocity_m_s # Error in m/s
            self.integral += self.dt * e_m_s    # Integral of the error
            derivative = (e_m_s - self.prev_e_m_s) / self.dt
            pid_output = self.Kp * e_m_s + self.Ki * self.integral + self.Kd * derivative
            self.prev_e_m_s = e_m_s

            rclpy.get_logger().info("PID output: {}".format(pid_output))

            pid_output = min(max(pid_output, -1), 1)
            self.throttle += pid_output
            self.throttle = max(self.MINSPEED, self.throttle)

            self.get_logger().info("ERR / P %f", e_m_s)
            self.get_logger().info("D %f", derivative)
            self.get_logger().info("Throttle %f", self.throttle)
            self.get_logger().info("Current speed %f", self.ctrl.curr_velocity_m_s)
            self.get_logger().info("Target speed %f", cmd_speed_m_s)

            self.forward_speed()
        elif (not cmd_speed_m_s):
            self.neutral()
        elif cmd_speed_m_s < 0:
            self.reverse_speed()
        pass

    def forward_speed(self):
        self.ctrl.pwm_prop.change_duty_cycle(min(self.throttle, self.MAX_SPEED))
        pass

    def reverse_speed(self):
        self.ctrl.pwm_prop.change_duty_cycle(max(self.throttle, self.REVERSEMAXSPEED))
        pass

def main():
    pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
    try:
        rclpy.init()
        listener = ControllerListener(pwm_prop)
        rclpy.spin(listener)
        print("Terminated ackermann_controller")
    except Exception:
        print("Error in ackermann_controller")
        pass

            