import threading
import click
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from bolide_interfaces.msg import SpeedDirection

class KeyboardController(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.get_logger().info("Teleop node started, Keyboard interrupt (ctrl+c will stop the node)")
        
        self.pub = self.create_publisher(SpeedDirection, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.4, self.timer_callback)
        # init speed and direction
        self.current_speed = 0.0
        self.current_direction = 0.0
        # Define the keycodes
        self.key_mapping = {'\x1b[A': 'UP', '\x1b[B': 'DOWN',
                        '\x1b[C': 'RIGHT', '\x1b[D': 'LEFT', 's': 'BRAKE', 'q': 'QUIT'}
        
    def timer_callback(self):
        self.publish_speed_direction()
    
    def publish_speed_direction(self):
        self.pub.publish(SpeedDirection(speed=self.current_speed, direction=self.current_direction))
        
    def on_key_press(self, in_key):
        key = self.get_key_char(in_key)
        if key in self.key_mapping.keys():
            self.perform_action(self.key_mapping[key])
        else:
            self.get_logger().warn(f"[WARN] -- No actions for key : {key}")
        
    def get_key_char(self, key):
        if hasattr(key, 'char'):
            return key.char
        else:
            return key
    
    def perform_action(self, coeff = 1.0):
        mykey = click.getchar()
        action = self.key_mapping[mykey]
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
        else:
            self.get_logger().warn(f"Unknown action: {action}")
        self.publish_speed_direction()
    
def main(args=None):
    rclpy.init(args=args)
    
    controller = KeyboardController()
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()
    
    try:
        while(True):
            controller.perform_action()
    except KeyboardInterrupt:
        controller.get_logger().info("KeyboardInterrupt received. Shutting down...")
    
    controller.destroy_node()
    rclpy.shutdown()
