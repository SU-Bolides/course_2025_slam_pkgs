#!/usr/bin/env python3

__author__ = "Maxime Chalumeau"
__email__ = "maxime.chalumeau@etu.sorbonne-universite.fr"
__status__ = "Tested"

import rclpy
from rclpy.node import Node
from pyPS4Controller.controller import Controller
from control_bolide.msg import SpeedDirection
import signal
import sys

def transf_trigger(raw):
    temp = (raw + 32767) / 65534
    return round(temp, 3)

def transf_joystick(raw):
    temp = raw / 32767
    return round(temp, 3)

class MyController(Node, Controller):
    def __init__(self, **kwargs):
        Node.__init__(self, 'speed_direction_controller')
        Controller.__init__(self, interface="/dev/input/js0", connecting_using_ds4drv=False, **kwargs)

        self.current_speed = 0.0
        self.current_direction = 0.0
        self.first_press_L2 = True
        self.braking = False
        self.NEUTRAL = 0.0
        self.BRAKE = 2.0
        self.CENTER = 0.0

        self.pub = self.create_publisher(SpeedDirection, 'cmd_vel', 10)
        self.timer = self.create_timer(0.4, self.timer_callback)

    def timer_callback(self):
        self.publish_speed_direction()

    def on_R2_press(self, value):
        if not self.braking:
            self.first_press_L2 = True
            value = transf_trigger(value)
            self.get_logger().info(f"R2 value: {value}")
            self.current_speed = value
            self.publish_speed_direction()

    def on_R2_release(self):
        if not self.braking:
            self.get_logger().info("R2 released")
            self.current_speed = 0.0
            self.publish_speed_direction()

    def on_L2_press(self, value):
        if self.first_press_L2:
            self.braking = True
            self.current_speed = self.BRAKE
            self.get_logger().info("braking")
        else:
            value = -transf_trigger(value)
            self.get_logger().info(f"L2 value: {value}")
            self.current_speed = value
        self.publish_speed_direction()

    def on_L2_release(self):
        self.braking = False
        self.first_press_L2 = True  # Reset for next brake
        self.get_logger().info("L2 released")
        self.current_speed = 0.0
        self.publish_speed_direction()

    def on_L3_x_at_rest(self):
        self.get_logger().info("straight")
        self.current_direction = 0.0
        self.publish_speed_direction()

    def on_L3_right(self, value):
        self.get_logger().info(f"right value: {value}")
        value = transf_joystick(value)
        self.current_direction = value
        self.publish_speed_direction()

    def on_L3_left(self, value):
        self.get_logger().info(f"left value: {value}")
        value = transf_joystick(value)
        self.current_direction = value
        self.publish_speed_direction()

    def publish_speed_direction(self):
        msg = SpeedDirection()
        msg.speed = float(self.current_speed)
        msg.direction = float(self.current_direction)
        self.pub.publish(msg)

    def on_options_press(self):
        self.current_speed = 0.0
        self.current_direction = 0.0
        self.publish_speed_direction()
        self.timer.destroy()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def on_disconnect(self):
        self.current_speed = 0.0
        self.current_direction = 0.0
        self.publish_speed_direction()
        self.timer.destroy()
        super(MyController, self).on_disconnect()

def main(args=None):
    rclpy.init(args=args)
    controller = MyController()

    def signal_handler(sig, frame):
        controller.on_options_press()

    signal.signal(signal.SIGINT, signal_handler)
    try:
        controller.listen(timeout=60)
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        controller.on_disconnect()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()