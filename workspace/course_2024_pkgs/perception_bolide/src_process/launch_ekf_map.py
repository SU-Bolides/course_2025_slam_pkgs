#!/usr/bin/env python3

import rospy
import subprocess

from perception_bolide.srv import launch_ekf


class LaunchEkf:
    def __init__(self):

        self.done = False

        rospy.init_node('launch_ekf_map', disable_signals=True)
        _ = rospy.Service('launch_ekf_map', launch_ekf,self.launch_ekf_map)
        _ = rospy.Timer(rospy.Duration(0.5), self.close_node)

        print("Waiting for the particle filter to init so that we can launch the MAP EKF")

        rospy.spin()

    def launch_ekf_map(self, req):
        if req.start:
            print("Launching MAP EKF (fusing odom_filtered and particle filter)")
            subprocess.Popen("roslaunch planning_bolide ekf_map.launch", shell=True)
            resp = launch_ekf
            resp.gotcha = 1
            print("Launched")
            self.done = True
            return self.done
        else:
            print("what is this")

    def close_node(self, _):
        if self.done:
            rospy.signal_shutdown("Finished task")

    

if __name__=="__main__":
    L = LaunchEkf()

