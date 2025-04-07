""" The node to command the direction """

import math
from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node
from bolide_interfaces.msg import SpeedDirection


def pos2psi(pos):
    """Transform the position information in the steering angle. Psi is the steering angle (rad),
    Theta is the motor angle (rad), Pos is the motor angle (DXL Units)

    Args:
        pos (float): the motor angle in DXL Units
    """
    theta_rad = (pos * (5.24/1023.)) + 0.524
    A = math.cos(theta_rad) * 15 - 4.35
    psi_rad = math.asin(A/25.)
    return psi_rad


def degrees2pos(degrees):
    """Transforming the degree (deg) into motor angle in DXL Units

    Args:
        degrees (float): a degree value

    Returns:
        int: motor angle in DXL Units
    """
    return int((degrees - 30.) * (1023./300.))


def set_dir_deg(angle_degre):
    """Set the direction in degree

    Args:
        angle_degre (float): the angle in degree

    Returns:
        int: the position in DXL Units
    """
    psi = math.radians(angle_degre)
    A = 25. * math.sin(psi) + 4.35
    theta = math.acos(A/15.)
    pos = degrees2pos(math.degrees(theta))
    return pos


class CommandDirection(Node):
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

        self.dynamixels_comms = self.create_timer(0.03, self.dxl_callback)  # Update the dynamixels every 30ms (33Hz)

    def dxl_callback(self):
        """Update the dynamixels. 
        We check if the target steering angle is within the limits of the steering angle
        """
        self.target_steering_angle_deg = max(min(self.target_steering_angle_deg, self.MAX_STEERING_ANGLE_DEG), -self.MAX_STEERING_ANGLE_DEG)
        try:
            pos, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, 36)   # Read the current position of the steering servo
            self.curr_steering_angle_deg = -180/3.14159*pos2psi(pos)    # Convert the position to an angle in degrees
            self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, 30, set_dir_deg(self.target_steering_angle_deg))   # Set the target position of the steering servo
        except:
            self.get_logger().warn("[WARNING] -- DYNAMIXEL PROBLEM")
    
    def cmd_callback(self, data):
        if self.MS:
            self.target_steering_angle_deg = -data.direction
        else:
            self.target_steering_angle_deg = -data.direction * self.MAX_STEERING_ANGLE_DEG

        self.last_command_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    listener = CommandDirection()
    try:
        rclpy.spin(listener)
    except Exception as e:
        print(f"Error in Command Direction : {e}")
    
