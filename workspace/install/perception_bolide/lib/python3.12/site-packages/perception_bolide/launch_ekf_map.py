import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from bolide_interfaces.srv import LaunchEkf

class LaunchEkf(Node):
    def __init__(self):
        super().__init__('launc_ekf_map')
        _ = self.create_service(LaunchEkf, 'launch_ekf_map', self.launch_ekf_map)
        _ = self.create_timer(rclpy.duration.Duration(seconds=0.5), self.close_node)

        print("Waiting for the particle filter to init so that we can launch the MAP EKF")

    def launch_ekf_map(self, req):
        if req.start:
            print("Launching MAP EKF (fusing odom_filtered and particle filter)")
            subprocess.Popen("roslaunch planning_bolide ekf_map.launch", shell=True)
            resp = LaunchEkf
            resp.gotcha = 1
            print("Launched")
            self.done = True
            return self.done
        else:
            print("what is this")

    def close_node(self, _):
        if self.done:
            rclpy.shutdown("Finished task")

def main(args=None):
    rclpy.init(args=args)
    L = LaunchEkf()
