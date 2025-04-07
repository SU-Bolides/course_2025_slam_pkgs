import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import numpy as np
import time

from bolides_interfaces.msg import SpeedDirection, MultipleRange, CameraInfo, LaserScan

from nav_module.forward_functions import forward_3_dials, forward_n_dials
from nav_module.backward_functions import backward_with_color_turn
from nav_module.utils import get_dials_ranges

class NavSensors(Node):
    def __init__(self):
        super().__init__("nav_3states")

        self.get_logger().info("Initializing the nav_sensors node")

        # Init subscribers
        self.create_subscription(LaserScan, "/lidar_data", self.callback_lidar)
        self.create_subscription(CameraInfo, "/camera_info", self.callback_camera)
        self.create_subscription(MultipleRange, "/raw_rear_range_data", self.callback_multiple_range)
        self.create_subscription(Bool, "/param_change_alert", self.get_params)

        # Init publisher
        self.pub = self.create_publisher(SpeedDirection, "/cmd_vel", 10)
        
        # Init the stored data
        self.lidar_data = LaserScan()
        self.camera_data = CameraInfo()
        self.rear_range_data = MultipleRange()

        # Init the parameters
        self.treshold_front_too_close = 0.9
        self.threshold_rear_too_close = 0.2
        self.threshold_front_far_enough = 1.4

        # conditions
        self.wrong_way = False              # True if the robot is going the wrong way
        self.front_too_close = False        # True if the front of the robot is too close
        self.rear_too_close = False         # True if the rear of the robot is too close
        self.front_far_enough = True        # True if the front of the robot is far enough

        # stored values
        self.previous_state = "forward"      # The previous state of the robot
        self.current_state = "forward"       # The current state of the robot
        self.protocol_entry = ("forward", "backward")    # The entry of the protocol, generally the previous and current state when different. This is used to be sure that the protocol follows the good state transition.
        self.cmd_vel = SpeedDirection()     # The command to publish
        self.navigation_dict = {            # The navigation functions (see nav_functions.py)
            "3Dials":forward_3_dials,
            "NDials":forward_n_dials
        }
        self.nav_features = {               # The features to use for the navigation (generally the statistics of the dials)
            "mean": np.mean,
            "max": np.max,
            "min": np.min,
            "median": np.median,
            "q1" : lambda x: np.percentile(x, 25),
            "q3" : lambda x: np.percentile(x, 75),
        }

        self.get_params()

# PARAMS UPDATE ===============================================================
    def get_params(self, value = True):
        """Update the parameters when the parameter change alert is received."""
        self.get_logger().info("Updating the parameters")

        # Gains
        self.Kd = self.declare_parameter("gain_direction", 0.3).value
        self.Kv = self.declare_parameter("gain_vitesse", 0.05).value
        self.Ka = self.declare_parameter("gain_direction_arg_max", 0.2).value

        # Race direction
        self.green_is_left = self.declare_parameter("green_is_left", True).value

        # Tresholds for changing state
        self.treshold_front_too_close = self.declare_parameter("treshold_front_too_close", 0.9).value
        self.threshold_rear_too_close = self.declare_parameter("threshold_rear_too_close", 0.2).value
        self.threshold_front_far_enough = self.declare_parameter("threshold_front_far_enough", 1.4).value

        # Forward navigation mode
        navigation_mode = self.declare_parameter("navigation_mode", "3Dials_Classic").value
        self.nav_func, self.nav_mode = navigation_mode.split("_")
        self.nav_feature_choice = self.declare_parameter("navigation_feature", "median").value
        self.nav_is_spaced = self.declare_parameter("spaced_dials", True).value
        self.front_dial_ratio = self.declare_parameter("front_dial_ratio", 0.2).value
        self.use_Kv_as_constant = self.declare_parameter("use_Kv_as_constant", False).value
        # Maximize threshold
        self.use_maximize_treshold = self.declare_parameter("use_maximize_treshold", False).value
        self.maximize_treshold = self.declare_parameter("maximize_treshold", 0.5).value

# PROTOCOLS ===================================================================
    def protocol_through_neutral(self):
        """Protocol to go through the neutral point."""
        self.pub.publish(SpeedDirection(0, 0))
        time.sleep(0.15)
    
    def protocol_brake(self):
        """Protocol to brake."""
        self.pub.publish(SpeedDirection(2, 0))
        time.sleep(0.15)

    def protocol_inverse_prop(self):
        """Protocol to go to backward from forward."""
        self.protocol_brake()
        self.protocol_through_neutral()
    
    def apply_protocol(self):
        """Apply the protocol to go to the next state."""
        transitions = {
            ("forward"  , "backward"): self.protocol_inverse_prop,
            ("forward"  , "stop")    : None,
            ("backward", "forward")  : self.protocol_through_neutral,
            ("backward", "stop")    : None,
            ("stop"    , "forward")  : None,
            ("stop"    , "backward"): self.protocol_through_neutral,
        }
        protocol = transitions[self.protocol_entry]
        if protocol is not None:
            protocol()

# CALLBACKS ===================================================================
    def callback_lidar(self, data):
        self.lidar_data = data
        self.nav_step()
    
    def callback_camera_info(self, data):
        self.camera_info = data
        self.nav_step() # be careful, this is a fast callback
    
    def callback_multiple_range(self, data):
        self.rear_range_data = data
        self.nav_step()

 # CONDITIONS ==================================================================
    def update_conditions(self):
        """Update the conditions of the robot."""
        
        # compute the current distances (THIS SHOULD BE DONE IN A PROCESSING NODE)
        _, front_ranges, _ = get_dials_ranges(self.lidar_data, n_dials=3, proportion=[1, 0.5, 1])
        current_front_distance = np.percentile(front_ranges, 25)
        current_rear_distance = np.min([self.rear_range_data.IR_rear_left.range, self.rear_range_data.IR_rear_right.range])

        # update conditions
        self.front_too_close  = current_front_distance < self.threshold_front_too_close
        self.front_far_enough = current_front_distance > self.threshold_front_far_enough
        self.rear_too_close   = current_rear_distance  < self.threshold_rear_too_close
    
# STATES ======================================================================
    def forward_state(self):
        """Update the speed and direction when the robot is going forward."""
        # Get the navigation function
        navigation_function = self.navigation_dict[self.nav_func]

        # Get the command
        self.cmd_vel = navigation_function(
            lidar_data=self.lidar_data,
            Kspeed=self.Kv,
            Kdir=self.Kd,
            Karg=self.Ka,
            mode=self.nav_mode,
            is_spaced=self.nav_is_spaced,
            navigation_feature=self.nav_features[self.nav_feature_choice],
            FrontRatio = self.front_dial_ratio,
            use_maximise_threshold = self.use_maximize_threshold,
            maximise_threshold = self.maximize_threshold,
            use_Kv_as_constant = self.use_Kv_as_constant,
        )

    def backward_state(self):
        """Update the speed and direction when the robot is going backward."""
        self.cmd_vel = backward_with_color_turn(self.camera_info, self.green_is_left, backward_speed=-0.3)

    def stop_state(self):
        """Update the speed and direction when the robot is stopped."""
        speed = 0 if self.previous_state == "backward" else 2
        self.cmd_vel = SpeedDirection(speed, 0)

    def next_state(self):
        """Update the current state of the robot.
        
        FTC = Front Too Close
        RTC = Rear Too Close
        FFE = Front Far Enough

                    |      CURRENT TO NEXT STATE     |
        FTC RTC FFE | stop     | forward  | backward |
        ----------------------------------------------
        0   0   0   | forward  | forward  | backward |
        0   0   1   | forward  | forward  | forward  |
        0   1   0   | forward  | forward  | forward  |
        0   1   1   | forward  | forward  | forward  |
        1   0   0   | backward | backward | backward |
        1   0   1   |-----------IMPOSSIBLE-----------|
        1   1   0   | stop     | stop     | stop     |
        1   1   1   |-----------IMPOSSIBLE-----------|
        """
        if self.current_state == "backward":
            if not self.front_too_close and (self.rear_too_close or self.front_far_enough):
                self.current_state = "forward"
            elif self.front_too_close and self.rear_too_close:
                self.current_state = "stop"
        else: # forward or stop
            if not self.front_too_close:
                self.current_state = "forward"
            elif not self.rear_too_close: # and self.front_too_close
                self.current_state = "backward"
            else: # self.rear_too_close and self.front_too_close
                self.current_state = "stop"


# MAIN LOOP ===================================================================
    def nav_step(self):
        """Main loop step of the navigation."""

        self.update_conditions()

        self.previous_state = self.current_state
        self.next_state()

        if self.previous_state != self.current_state:
            self.protocol_entry = (self.previous_state, self.current_state)
            self.get_logger().info(f"From {self.previous_state} to {self.current_state}")
            self.get_logger().info(f" RTC = {self.rear_too_close}, FTC = {self.front_too_close}, FFE = {self.front_far_enough}")
            self.apply_protocol()

        state_actions = {
            "forward": self.forward_state,
            "backward": self.backward_state,
            "stop": self.stop_state
        }    

        state_actions[self.current_state]()
        self.pub.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    nav_sensors = NavSensors()
    rclpy.spin(nav_sensors)
    nav_sensors.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()