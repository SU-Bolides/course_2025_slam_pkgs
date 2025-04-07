import threading
import click
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from bolide_interfaces.msg import SpeedDirection
from pynput.keyboard import Key, Listener


class KeyboardController(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.get_logger().info("Teleop node started, Keyboard interrupt (ctrl+c will stop the node)")
        
        self.pub = self.create_publisher(SpeedDirection, '/cmd_vel', 10)

        self.listener = Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        
        self.timer = self.create_timer(0.4, self.timer_callback)
        # init speed and direction
        self.current_speed = 0.0
        self.current_direction = 0.0
        # Define the keycodes
        self.key_mapping = {
            Key.up: 'UP',
            Key.down: 'DOWN',
            Key.left: 'LEFT',
            Key.right: 'RIGHT',
            Key.space: 'NEUTRAL',
            Key.enter: 'QUIT',
            'z': 'UP',
            'q': 'LEFT',
            's': 'DOWN',
            'b': 'BRAKE',
            'd': 'RIGHT',
        }
        # self.key_mapping = {'\x1b[A': 'UP', '\x1b[B': 'DOWN',
        #                 '\x1b[C': 'RIGHT', '\x1b[D': 'LEFT', 's': 'BRAKE', 'q': 'QUIT', 'n': 'NEUTRAL'}

        key_mapping_str = '\n'.join([f'\t{key}: {value}' for key, value in self.key_mapping.items()])
        print(f"Key control mapping:\n{key_mapping_str}\n")

    def timer_callback(self):
        self.publish_speed_direction()
    
    def publish_speed_direction(self):
        print("publish curr_speed = ", self.current_speed)
        print("publish curr_direction = ", self.current_direction)
        self.pub.publish(SpeedDirection(speed=self.current_speed, direction=self.current_direction))
        
    def on_key_press(self, in_key):
        key = self.get_key_char(in_key)
        if key in self.key_mapping.keys():
            self.perform_action(self.key_mapping[key])
        else:
            self.get_logger().warn(f"[WARN] -- No actions for key : {key}")
    
    def on_key_release(self, in_key):
        key = self.get_key_char(in_key)
        if key in self.key_mapping.keys():
            self.perform_action(self.key_mapping[key], coeff=0)
        
    def get_key_char(self, key):
        if hasattr(key, 'char'):
            return key.char
        else:
            return key
    
    def perform_action(self, action, coeff = 0.05):
        print("commande : ", action)
        if action == 'UP':
            self.current_speed = 1.0 * coeff
        elif action == 'DOWN':
            self.current_speed = -1.0 * coeff
        elif action == 'LEFT':
            self.current_direction = -1.0 * coeff
        elif action == 'RIGHT':
            self.current_direction = 1.0 * coeff
        elif action == 'BRAKE':
            self.current_speed = 2.0 * coeff
        elif action == 'QUIT':
            exit()
        elif action == 'NEUTRAL':
            self.current_speed = 0.0
        else:
            self.get_logger().warn(f"Unknown action: {action}")
        self.publish_speed_direction()
    
def main(args=None):
    rclpy.init(args=args)
    
    controller = KeyboardController()
    # thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    # thread.start()
    
    try:
        with controller.listener:
            rclpy.spin()
    except KeyboardInterrupt:
        controller.get_logger().info("KeyboardInterrupt received. Shutting down...")
    
    controller.destroy_node()
    rclpy.shutdown()
