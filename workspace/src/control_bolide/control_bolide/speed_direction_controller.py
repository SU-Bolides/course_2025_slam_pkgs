import rclpy
from rclpy.node import Node
from bolide_interfaces.msg import SpeedDirection
from std_msgs.msg import Bool
from rpi_hardware_pwm import HardwarePWM
import time


class ControllerListener:
    def __init__(self):
        # Max speed authorized
        self.MAXSPEED = 10.0 # percentage cyclic ratio
        self.MINSPEED = 8.4
        self.NEUTRAL = 8.0  # 7.5 = neutral
        self.REVERSEMINSPEED = 7.6
        self.REVERSEMAXSPEED = 6.5
        self.BRAKE = 5

        # Values for the direction
        self.CENTER = 7.43
        self.LEFT = 5.8
        self.RIGHT = 8.4
        self.left_range = abs(self.CENTER - self.LEFT)
        self.right_range = abs(self.CENTER - self.RIGHT)

        # init propulsion pwm and direction pwm
        self.pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
        self.pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
        self.pwm_dir.start(self.CENTER)
        self.pwm_prop.start(self.NEUTRAL)

        # Node initialisation
        rclpy.init()
        self.node = rclpy.create_node('speed_direction_listener')
        self.node.create_subscription(SpeedDirection, "/cmd_vel", self.callback, 10)
        self.node.create_subscription(Bool, "/emergency_brake", self.emergency_brake_callback, 10)

        # emergency brake boolean
        self.emergency_brake = False

        # Initialize watchdog timer
        self.watchdog_timer = self.node.create_timer(0.5, self.watchdog_callback)
        self.last_command_time = time.time()

        # initialise state machine
        self.sm = StateMachine(self)

        rclpy.spin(self.node)

    def callback(self, data):
        # if emergency brake is activated, don't do anything
        if self.emergency_brake:
            return

        # get speed and direction from the message
        speed = data.speed
        direction = data.direction

        # crop speed between -1 and 1 (except if speed == 2 it's used for braking)
        if speed != 2:
            if speed < -1: speed = -1
            elif speed > 1: speed = 1
        # crop direction between -1 and 1
        if direction <-1: direction = -1
        elif direction > 1: direction = 1

        # change the states of the State machine
        if speed == 0:
            if self.sm.state in ["Backward", "Brake", "Neutral_After_Brake"]:
                self.sm.neutral_after_brake()
            else:
                self.sm.neutral_after_forward()
        elif 1 >= speed > 0:
            self.sm.forward(speed)
        elif speed < 0:
            self.sm.backward(speed)
        elif speed == 2:
            self.sm.brake_()

        # change the direction of the robot
        if direction < 0:
            self.pwm_dir.change_duty_cycle(self.CENTER+direction*self.left_range)
        else:
            self.pwm_dir.change_duty_cycle(self.CENTER+direction*self.right_range)

        # Update the last command time
        self.last_command_time = time.time()

    # callback for the emergency brake command
    def emergency_brake_callback(self, data):
        # if the emergency brake is activated, stop the robot
        if data.data:
            rclpy.get_logger().info("Emergency brake activated")
            self.emergency_brake = True
            if self.sm.state in ["Forward", "Brake"]:
                # if the robot is moving forward, brake
                self.sm.brake_()
            elif self.sm.state in ["Neutral_After_Forward"]:
                self.sm.neutral_after_forward()
            else:
                # else, just go neutral
                # sending a brake command when going backward or when neutral will make it move backward
                # can raise a warning message but doesn't affect the control
                self.sm.neutral_after_brake()
        else:
            rclpy.get_logger().info("Emergency brake deactivated")
            self.emergency_brake = False

    def watchdog_callback(self):
        # If it's been more than 0.5 seconds since the last command, stop the robot
        # This is to prevent the robot from moving if the controller or computer crashes or disconnects
        if time.time() - self.last_command_time > 0.5:
            if self.sm.state in ["Backward", "Brake", "Neutral_After_Brake"]:
                self.sm.neutral_after_brake()
            else:
                self.sm.neutral_after_forward()

class StateMachine:
    """
    State machine for the robot

    States:
    - Forward: Moving forward
    - Backward: Moving backward
    - Neutral_After_Forward: Stopped moving after moving forward
    - Neutral_After_Brake: Stopped moving after braking or moving backward
    - Brake: Brake applied

    Transitions:
    - Forward: Neutral_After_Forward, Brake, Neutral_After_Brake, Forward
    - Backward: Neutral_After_Brake, Backward
    - Neutral_After_Forward: Forward, Neutral_After_Forward
    - Neutral_After_Brake: Backward, Brake, Neutral_After_Brake
    - Brake: Forward, Neutral_After_Forward, Brake
    """

    def __init__(self, controller: ControllerListener):
        self.controller = controller
        self.state = "Neutral_After_Forward"

    def forward(self, speed):
        if self.state not in ["Neutral_After_Forward", "Brake", "Neutral_After_Brake", "Forward"]:
            self.controller.pwm_prop.change_duty_cycle(self.controller.NEUTRAL)
            time.sleep(0.15)

        if self.state != "Forward": self.state = "Forward"
        self.controller.pwm_prop.change_duty_cycle(self.controller.MINSPEED + speed*(self.controller.MAXSPEED-self.controller.MINSPEED))
       

    def backward(self, speed): 
        if self.state not in ["Neutral_After_Backward","Backward"]:
            self.controller.pwm_prop.change_duty_cycle(self.controller.REVERSEMINSPEED + speed*(self.controller.REVERSEMINSPEED - self.controller.REVERSEMAXSPEED))
            time.sleep(0.15)
            self.controller.pwm_prop.change_duty_cycle(self.controller.NEUTRAL)
            time.sleep(0.15)

        if self.state != "Backward": self.state = "Backward"
        self.controller.pwm_prop.change_duty_cycle(self.controller.REVERSEMINSPEED + speed*(self.controller.REVERSEMINSPEED-self.controller.REVERSEMAXSPEED))

    def neutral_after_forward(self):
        if self.state in ["Forward", "Neutral_After_Forward"]:
            if self.state != "Neutral_After_Forward": self.state = "Neutral_After_Forward"
            self.controller.pwm_prop.change_duty_cycle(self.controller.NEUTRAL)
        else:
            rclpy.get_logger().warn("Cannot go to Neutral_After_Forward from state: {}".format(self.state))

    def neutral_after_brake(self):
        if self.state in ["Backward", "Brake", "Neutral_After_Brake"]:
            if self.state != "Neutral_After_Brake": self.state = "Neutral_After_Brake"
            self.controller.pwm_prop.change_duty_cycle(self.controller.NEUTRAL)
        else:
            rclpy.get_logger().warn("Cannot go to Neutral_After_Brake from state: {}".format(self.state))

    def brake_(self):
        if self.state in ["Forward", "Neutral_After_Forward", "Brake"]:
            if self.state != "Brake": rclpy.get_logger().info("Brake applied"); self.state = "Brake"
            self.controller.pwm_prop.change_duty_cycle(self.controller.BRAKE)
        else:
            self.controller.pwm_prop.change_duty_cycle(self.controller.NEUTRAL)

def main(args=None):
    #rclpy.init(args=args)
    try:
        listener = ControllerListener()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
