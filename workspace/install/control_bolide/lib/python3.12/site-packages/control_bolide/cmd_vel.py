""" The node to command the speed of the car """

import math
from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from bolide_interfaces.msg import SpeedDirection


def get_sign(val):
    """get the sign of a value

    Args:
        val (any): any int or float value

    Returns:
        int: 1 or -1 if positive or negative
    """
    return (val > 0) - (val < 0)


class CommandSpeed(Node):
    def __init__(self):
        super().__init__('cmd_vel')

        self.init = 0

        self.MAXSPEED = 10
        self.MINSPEED = 8.4
        self.NEUTRAL         = 8.0
        self.REVERSEMINSPEED = 7.6
        self.REVERSEMAXSPEED = 6.5
        self.DIR_VEL_THRESHOLD_M_S  = 0.05
        self.curr_velocity_m_s = 0.0

        self.esc_period = 20000 #ns

        self.curr_dir = 1
        self.tx_data = Int16()

        self.last_command_time = self.get_clock().now()


        self.sub = self.create_subscription(SpeedDirection, "/cmd_vel", self.cmd_callback, 10 )
        self.stm32_publish = self.create_publisher(Int16, "/stm32_data", 10)

        self.init = 1

    def cmd_callback(self, data):
        self.cmd_velocity = data.speed
        if (not (get_sign(data.speed) == self.curr_dir)):
            if (not self.curr_dir) or (abs(self.curr_velocity_m_s) < self.DIR_VEL_THRESHOLD_M_S): 

                #We can command reverse while the car is going forwards
                #However, the car needs to stop and then start reversing before it actually goes in reverse.
                #So we only switch the sign if the car is going slow enough to be considered "stationnary".
                self.curr_dir = get_sign(data.speed)
                # rospy.loginfo("Switched direction (speed sign): %s", str(self.curr_dir))

        # Update the last command time
        self.last_command_time = self.get_clock().now()
        self.command(self.cmd_velocity)

    def command(self, cmd_speed):
        if  1 > cmd_speed > 0.02:
            self.forward()
        
        elif -0.01 < cmd_speed < 0.01:
            self.neutral()
        
        elif -1 < cmd_speed < -0.02:
            self.reverse()

    def forward(self):
        self.publish_stm32_data(self.MINSPEED + self.cmd_velocity * (self.MAXSPEED - self.MINSPEED))

    def reverse(self):
        self.publish_stm32_data(self.REVERSEMINSPEED + self.cmd_velocity * (self.REVERSEMINSPEED - self.REVERSEMAXSPEED))

    def neutral(self):
        self.publish_stm32_data(self.NEUTRAL)

        
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