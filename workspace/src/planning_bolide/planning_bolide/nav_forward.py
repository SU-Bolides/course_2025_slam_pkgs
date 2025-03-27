import rclpy
from rclpy.node import Node
from bolide_interfaces.msg import SpeedDirection, LaserScan
from std_msgs.msg import Bool
from nav_module.forward_functions import forward_3_dials, forward_n_dials

class NavForward(Node):
    def __init__(self):
        super().__init__("nav_forward")

        self.get_logger().info("Initializing the nav_forward node")

        # Init publisher
        self.pub = self.create_publisher(SpeedDirection, "/cmd_vel", 10)
        self.n_dials = 11   # Number of dials

        self.cmd = SpeedDirection()
        self.navigation_dict = {
            "3Dials":forward_3_dials,
            "NDials":forward_n_dials
        }
        self.navigation_choice = "3Dials_spaced"
        self.get_all_params()

        # Subscribers
        self.create_subscription(LaserScan, "/lidar_data", self.lidar_data_callback, 10)
        self.create_subscription(Bool, "/param_change_alert", self.get_all_params, 10)

    def lidar_data_callback(self, msg:LaserScan):
        # Get the navigation function
        navigation_function = self.navigation_dict[self.navigation_choice]

        # Get the command
        self.cmd = navigation_function(
            lidar_data=msg,
            Kspeed=0.005,
            Kdir=self.Kd,
            Karg=self.Ka,
            mode=self.mode,
            n_dials=self.n_dials,
            FrontRatio = self.front_dial_ratio
        )

        # Publish the command
        self.pub.publish(self.cmd)

    def get_all_params(self, value = True):
        # Get all the parameters
        self.Kd = self.declare_parameter("gain_direction", 0.8).value
        self.Kv = 0.005
        self.Ka = self.declare_parameter("gain_direction_arg_max", 0.8).value
        navigation_mode = self.declare_parameter("navigation_mode", "3Dials_Classic").value
        
        self.navigation_choice, self.mode = navigation_mode.split("_")

        self.feature = self.declare_parameter("navigation_feature", "mean").value

        self.use_dials = self.declare_parameter("use_dials", False).value
        self.n_dials = self.declare_parameter("navigation_n_dials", 11).value
        self.front_dial_ratio = self.declare_parameter("front_dial_ratio", 0.1).value

def main(args=None):
    rclpy.init(args=args)
    nav_forward = NavForward()
    rclpy.spin(nav_forward)
    nav_forward.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()