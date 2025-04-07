""" The node to command the speed of the car """

import math
from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node
from bolide_interfaces.msg import SpeedDirection

class CommandSpeed(Node):
    def __init__(self):
        super().__init__('cmd_dir')

        # Dynamixel stuff:
        # Protocol version
        self.PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID                      = 1                 
        self.BAUDRATE                    = 115200            
        self.DEVICENAME                  = '/dev/ttyUSB0'    # Symlink it in the udev to ttyU2D2

        self.MAX_STEERING_ANGLE_DEG = 15.5 # deg

        self.target_steering_angle_deg = 0.0
        self.curr_steering_angle_deg = 0.0

        self.MS              = False


        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if self.portHandler.openPort():
            self.get_logger().info("[INFO] -- Succeeded to open the port")
        else:
            self.get_logger().error("[ERROR] -- Failed to open the port")

        # Setting the baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().info("[INFO] -- Succeeded to change the baudrate")
        else:
            self.get_logger().error("[ERROR] -- Failed to change the baudrate")
        
        self.sub = self.create_subscription(SpeedDirection, "/cmd_vel", self.cmd_callback, 10 )
        self.stm32_publish = self.create_publisher(Int16, "/stm32_data", 10)

    def cmd_callback(self, data):
        if (not (get_sign(data.speed) == self.cur_dir)):
            if (not self.cur_dir) or (abs(self.curr_velocity_m_s) < self.DIR_VEL_THRESHOLD_M_S): 

                #We can command reverse while the car is going forwards
                #However, the car needs to stop and then start reversing before it actually goes in reverse.
                #So we only switch the sign if the car is going slow enough to be considered "stationnary".
                self.cur_dir = get_sign(data.speed)
                # rospy.loginfo("Switched direction (speed sign): %s", str(self.cur_dir))

        # Update the last command time
        self.last_command_time = self.get_clock().now()

    def publish_stm32_data(self, cycle_ratio):
        """Send to stm32 the cycle_ration of the motors

        Args:
            cycle_ratio (float): the cycle of the cars (netural, reverse, forward,...)
        """
        if self.init:
            self.tx_data = Int16()
            self.tx_data.data = int(cycle_ratio*0.00938 * self.esc_period)

        # The previous implementation used RPi PWM which was unreliable.
            # Experiments showed the RPi to overshoot duration by 1.066. 

        # Cyclic ratio is the time of the period spent high. So 100 would be constantly high, 50 would be half high half low, etc.
        # The esc period is 20000 ns, and we send the actual pulse duration to the stm32. We also convert from RPi to "true".

        if rclpy.ok():
            print("Tx_data = ", self.tx_data)
            self.stm32_publish.publish(self.tx_data)

def main(args=None):
    rclpy.init(args=args)
    listener = CommandSpeed()
    try:
        rclpy.spin(listener)
    except Exception as e:
        print(f"Error in Command Speed : {e}")