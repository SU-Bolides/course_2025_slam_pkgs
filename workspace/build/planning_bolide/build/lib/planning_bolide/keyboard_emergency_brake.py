import numpy as np
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener  # used for reading the keyboard

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_emergency_brake')
        self.pub = self.create_publisher(Bool, "/emergency_brake", 1)
        self.space_pressed = False
        self.last_space_pressed = False
        self.timer = self.create_timer(0.1, self.get_space_pressed)  # 10 Hz timer

    def on_press(self, key):
        if key == Key.space:
            self.last_space_pressed = self.space_pressed  # get the last state of the space key
            self.space_pressed = not self.space_pressed  # toggle the space_pressed variable on and off

    def get_space_pressed(self):  # publish the space_pressed variable if it has changed
        if self.space_pressed != self.last_space_pressed:
            self.pub.publish(Bool(data=self.space_pressed))
            self.last_space_pressed = self.space_pressed

    def register_keyboard_listener(self):
        listener = Listener(on_press=self.on_press)  # Register the listener to the keyboard
        listener.start()
        return listener

def main(args=None):
    rclpy.init(args=args)
    s = KeyboardControl()
    listener = s.register_keyboard_listener()

    try:
        rclpy.spin(s)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Shutting down...")
    finally:
        listener.stop()
        s.destroy_node()
        rclpy.shutdown()
        sys.exit()

if __name__ == '__main__':
    main()